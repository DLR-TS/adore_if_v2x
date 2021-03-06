/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Reza Dariani - initial API and implementation 
 ********************************************************************************/
#include <ros/ros.h>
#include <adore_if_ros/baseapp.h>
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore_if_ros/paramsfactory.h>
#include <adore/fun/setpointrequest.h>
#include <adore/mad/coordinateconversion.h>
#include <mcm_dmove_mcm_dmove/MCM.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>



/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
namespace if_ROS
{
class setpointrequest_to_mcm  : public Baseapp
{
  private: 
      mcm_dmove_mcm_dmove::MCM msg;
      adore::mad::AReader<adore::fun::SetPointRequest>* ntr_reader_;  
      adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
      adore::mad::AReader<adore::fun::PlatooningInformation>* platooningstate_reader;
      adore::fun::PlatooningInformation platoonInformation;       
      adore::params::APVehicle* ap_vehicle_;
      adore::fun::SetPointRequest spr_tmp_;  
      adore::fun::SetPointRequest spr_;
      adore::fun::VehicleMotionState9d state_;
      ros::Publisher setPointRequest_publisher; 
      ros::Publisher setPointRequest_publisher_sim; 
      mcm_dmove_mcm_dmove::TrajectoryPoint tj_point;
      mcm_dmove_mcm_dmove::PlannedTrajectory pl_tj ;  
      adore::params::APVehicle* pvehicle_;    
      double last_t_;  
      int utm_zone_; 
      bool southern_hemisphere;
      double lat, lon;
      int v2xStationID;
      int debug_level;
      double vehicle_a; //front axle to cog
      double vehicle_b; //rear axle to cog
      double vehicle_c; //front axle to front border
      double vehicle_d; //rear border to rear axle
  
    public:
      setpointrequest_to_mcm()
      {
        ROS_INFO("Constructor");
        
      }
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        v2xStationID = 0;      
        last_t_ = -1.0;
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        setPointRequest_publisher = getRosNodeHandle()->advertise<mcm_dmove_mcm_dmove::MCM>("MCM_out",1);
        setPointRequest_publisher_sim = getRosNodeHandle()->advertise<mcm_dmove_mcm_dmove::MCM>("v2x/outgoing/MCM",1);
        std::function<void()> run_fcn = (std::bind(&setpointrequest_to_mcm::run_func, this));
        adore::if_ROS::FUN_Factory fun_factory(getRosNodeHandle());
        adore::if_ROS::PARAMS_Factory params_factory(*getRosNodeHandle(),"");
        pvehicle_ = params_factory.getVehicle();
        ntr_reader_ = fun_factory.getNominalTrajectoryReader();
        state_reader_ = fun_factory.getVehicleMotionStateReader();
        platooningstate_reader = fun_factory.getPlatooningStateReader();
        vehicle_a = pvehicle_->get_a();
        vehicle_b = pvehicle_->get_b();
        vehicle_c = pvehicle_->get_c();  
        Baseapp::addTimerCallback(run_fcn);    
      }
      boost::posix_time::time_duration::tick_type milliseconds_since_epoch()
       {
        using boost::gregorian::date;
        using::boost::posix_time::ptime;
        using::boost::posix_time::microsec_clock;
        static ptime const epoch(date(2004,1,1));
        return (microsec_clock::universal_time() - epoch).total_milliseconds();
        } 
      int getGenerationDeltaTime()
      {
        return milliseconds_since_epoch()%65536;
      }     
      void readSetPointRequest()
      {
        getParam("PARAMS/UTMZone", utm_zone_);
        getParam("PARAMS/SouthHemi",southern_hemisphere);
        getParam("v2xStationID", v2xStationID);        
        msg.header.stationID.value = v2xStationID;
        getParam("PARAMS/nlp_planner/MCM_senser_debug_level", debug_level);
        if( ntr_reader_!=0 && ntr_reader_->hasData() && state_reader_!=0 && state_reader_->hasData() )
				{
          msg.maneuverCoordination.generationDeltaTime.value = getGenerationDeltaTime();
					state_reader_->getData(state_);
				  const double t = state_.getTime();
          ntr_reader_->getData(spr_tmp_);
          if(spr_tmp_.isActive(t))spr_ = spr_tmp_;
          auto trajectory_data = spr_.getTrajectory().getData();
          double X, Y,PSI, T;
          int NumPointsPlannedTrajectory;
          X= trajectory_data(1,0);
          Y= trajectory_data(2,0);
          double x, y;
          x= X;
          y = Y;          
          PSI= trajectory_data(3,0);
          T = trajectory_data(0,0);
          
 
          PSI = adore::mad::CoordinateConversion::twoPIrange(PSI);
          toCenterOfTheFrontSide(x,y,PSI);
      
          adore::mad::CoordinateConversion::UTMXYToLatLonDegree(x,y,utm_zone_,southern_hemisphere,lat,lon);
          msg.maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value = adore::mad::bound(- 90.,(lat),  90.);
          msg.maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value = adore::mad::bound(-180.,(lon),180.);
          msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value = PSI; 
          int factor = 1;
					static const int MAX_POINTS_IN_MSG = 30; //MCM defined max number
					if(trajectory_data.nc()>=MAX_POINTS_IN_MSG*2)
					{
						factor = std::floor((double)trajectory_data.nc()/(double)MAX_POINTS_IN_MSG);
						NumPointsPlannedTrajectory = MAX_POINTS_IN_MSG;
					}
					else
					{
						NumPointsPlannedTrajectory = std::min((int)MAX_POINTS_IN_MSG,(int)trajectory_data.nc());
					}
          pl_tj.elements.clear() ; 
          pl_tj.count= NumPointsPlannedTrajectory;
					for( int i=0;i<NumPointsPlannedTrajectory;i++)  //sparsing
					{
            tj_point.deltaTimeCs.value = (unsigned int)((trajectory_data(0,i*factor)-T)*10.0);
						tj_point.deltaXCm.value = (int)((trajectory_data(1,i*factor)-X)*100.0);
						tj_point.deltaYCm.value = (int)((trajectory_data(2,i*factor)-Y)*100.0);
            tj_point.absSpeedPresent = true;
						tj_point.absSpeed.value = (double)std::abs(trajectory_data(4,i*factor));
            tj_point.headingValuePresent = true;
            double heading = adore::mad::CoordinateConversion::twoPIrange(trajectory_data(3,i*factor));
            heading = adore::mad::CoordinateConversion::RadToDeg(heading);
            tj_point.headingValue.value = heading;//adore::mad::bound(tj_point.headingValue.MIN, heading, tj_point.headingValue.MAX);
            pl_tj.elements.push_back(tj_point);
						T = trajectory_data(0,i*factor);
						X = trajectory_data(1,i*factor);
						Y = trajectory_data(2,i*factor);       
					}    
          
          msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory = pl_tj;  ;    
          msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthValue.value = vehicle_a + vehicle_b + vehicle_c + vehicle_d;
           msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleWidth.value = pvehicle_->get_bodyWidth();
          
        }
        if(platooningstate_reader!=0 && platooningstate_reader->hasData())    
        {
          platooningstate_reader->getData(platoonInformation);
          if(platoonInformation.getId() == v2xStationID)
          {
            msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevelPresent = true;        
            msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value = int(platoonInformation.getToleratedDistanceAhead()*100);
            msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value= int(platoonInformation.getToleratedDistanceBehind()*100);
            msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value =  msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.SAE_LEVEL_3;// (int)platoonInformation.getTargetAutomationLevel();  
            msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value =     (int)platoonInformation.getLanePosition(); 

          }
        }
        print_debug(msg, platoonInformation);
        
      }
    virtual void run_func()
    {
      readSetPointRequest();
      setPointRequest_publisher.publish(msg); 
      setPointRequest_publisher_sim.publish(msg);
        
    }
    void print_debug(mcm_dmove_mcm_dmove::MCM msg, adore::fun::PlatooningInformation platoonInformation)
    {
      if(debug_level)
      {
        std::cout<<"\n"<< "messageID: "<<unsigned(msg.header.messageID.value) ;
        std::cout<<"\n"<< "stationID: "<<unsigned(msg.header.stationID.value) ;
        std::cout<<"\n"<< "count: "<< unsigned(msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count);
        std::cout<<"\n"<< "PlatoonID: "<<unsigned(platoonInformation.getId());
        std::cout<<"\n"<< "TleratedDistanceAheadCmps: "<<(double)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value/1000.;
        std::cout<<"\n"<< "ToleratedDistanceBehind: "<<(double)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value/1000.;
        std::cout<<"\n"<< "targetAutomationLevel: "<<(unsigned)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value ;
        std::cout<<"\n"<< "lanePosition: "<<msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value  ;
        std::cout<<"\n\n";

      }
    }
    void toCenterOfTheFrontSide(double &x, double &y, double psi )
    {
      x = x  +  std::cos(psi)*(vehicle_a+vehicle_b+vehicle_c) ;
      y = y  +  std::sin(psi)*(vehicle_a+vehicle_b+vehicle_c) ;
    }

};
} // namespace if_ROS
} // namespace adore




int main(int argc,char **argv)
{

    adore::if_ROS::setpointrequest_to_mcm sprtmcm;
    sprtmcm.init(argc, argv, 20., "setpointrequest_to_mcm_node");
    ROS_INFO("setpointrequest_to_mcm_node namespace is: %s", sprtmcm.getRosNodeHandle()->getNamespace().c_str());
    sprtmcm.run();
    return 0;

}
