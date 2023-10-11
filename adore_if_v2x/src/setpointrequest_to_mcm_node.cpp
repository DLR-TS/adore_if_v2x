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
#include <iostream>
//#include <adore_if_ros/baseapp.h>
//#include <adore_if_ros/funfactory.h>
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
//#include <adore_if_ros/paramsfactory.h>
#include <adore/fun/setpointrequest.h>
#include <coordinate_conversion/coordinate_conversion.h>
#include <mcm_dmove_mcm_dmove/MCM.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <adore_if_ros_msg/SetPointRequest.h>
#include <nav_msgs/Odometry.h>




/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
namespace if_ROS
{
class setpointrequest_to_mcm  
{
  private: 
      mcm_dmove_mcm_dmove::MCM msg;
      //adore::mad::AReader<adore::fun::SetPointRequest>* ntr_reader_;  
      //adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
      //adore::mad::AReader<adore::fun::PlatooningInformation>* platooningstate_reader;
      //adore::fun::PlatooningInformation platoonInformation;       
      //adore::params::APVehicle* ap_vehicle_;
      //adore::fun::SetPointRequest spr_tmp_;  
      //adore::fun::SetPointRequest spr_;
      //adore::fun::VehicleMotionState9d state_;
      ros::Subscriber SetPointRequestSubscriber;   
      ros::Publisher MCM_publisher; 
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
      double vehicle_width;
      double rate;
      ros::NodeHandle* nh_;
  
    public:
      setpointrequest_to_mcm()
      {
        ROS_INFO("Constructor");
        
      }
      void init(int argc, char **argv, double rate, std::string nodename)
      {
        std::cout<<"\ninit function";
        ROS_INFO("INIT");
        v2xStationID = 0;      
        last_t_ = -1.0;
        this->rate = rate;
        ros::init(argc, argv, nodename);
         nh_ = new ros::NodeHandle();
         SetPointRequestSubscriber= nh_->subscribe("FUN/SetPointRequest",1,&setpointrequest_to_mcm::receive_spr,this);
         MCM_publisher = nh_ ->advertise<mcm_dmove_mcm_dmove::MCM>("v2x/MCM",1);
        ///platooningstate_reader = fun_factory.getPlatooningStateReader();
        nh_->getParam("PARAMS/Vehicle/a", vehicle_a);
        nh_->getParam("PARAMS/Vehicle/b", vehicle_b);
        nh_->getParam("PARAMS/Vehicle/c", vehicle_c);
        nh_->getParam("PARAMS/Vehicle/bodyWidth", vehicle_width);  
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
      void receive_spr(adore_if_ros_msg::SetPointRequestConstPtr msg_spr)
      {
        auto sp = msg_spr.get()->setPoints;
        //std::cout<<"\n"<<sp[0];
        nh_->getParam("PARAMS/UTMZone", utm_zone_);
        nh_->getParam("PARAMS/SouthHemi",southern_hemisphere);
        nh_->getParam("v2xStationID", v2xStationID);        
        msg.header.stationID.value = v2xStationID;
        msg.maneuverCoordination.generationDeltaTime.value = getGenerationDeltaTime();
        double X, Y,PSI, T;
        int NumPointsPlannedTrajectory;
        X= sp[0].X;
        Y= sp[0].Y;
        double x, y;
        x= X;
        y = Y;          
        PSI= sp[0].PSI;
        T = sp[0].tStart; 
        PSI = adore::mad::CoordinateConversion::twoPIrange(PSI); 
        toCenterOfTheFrontSide(x,y,PSI);     
        adore::mad::CoordinateConversion::UTMXYToLatLonDegree(x,y,utm_zone_,southern_hemisphere,lat,lon);
        msg.maneuverCoordination.mcmParameters.basicContainer.referencePosition.latitude.value = adore::mad::bound(- 90.,(lat),  90.);
        msg.maneuverCoordination.mcmParameters.basicContainer.referencePosition.longitude.value = adore::mad::bound(-180.,(lon),180.);
        msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.heading.headingValue.value = PSI; 
        int factor = 1;
				static const int MAX_POINTS_IN_MSG = 30; //MCM defined max number  
        int spr_size = sp.size(); 
				if(spr_size>=MAX_POINTS_IN_MSG*2) {
					factor = std::floor((double)spr_size/(double)MAX_POINTS_IN_MSG);
					NumPointsPlannedTrajectory = MAX_POINTS_IN_MSG;
				}
				else{
					NumPointsPlannedTrajectory = std::min((int)MAX_POINTS_IN_MSG,(int)spr_size);
				}              
        std::cout<<"\n"<<utm_zone_<<"\t"<<X<<"\t"<<Y<<"\t"<<spr_size;
        pl_tj.elements.clear() ; 
        pl_tj.count= NumPointsPlannedTrajectory;
				for( int i=0;i<NumPointsPlannedTrajectory;i++)  //sparsing
				{
            tj_point.deltaTimeCs.value = (unsigned int)((sp[i*factor].tStart-T)*10.0);
						tj_point.deltaXCm.value = (int)((sp[i*factor].X-X)*100.0);
						tj_point.deltaYCm.value = (int)((sp[i*factor].Y-Y)*100.0);
            tj_point.absSpeedPresent = true;
						tj_point.absSpeed.value = (double)std::abs(sp[i*factor].vx);
            tj_point.headingValuePresent = true;
            double heading = adore::mad::CoordinateConversion::twoPIrange(sp[i*factor].PSI);
            heading = adore::mad::CoordinateConversion::RadToDeg(heading);
            tj_point.headingValue.value = heading;//adore::mad::bound(tj_point.headingValue.MIN, heading, tj_point.headingValue.MAX);
            pl_tj.elements.push_back(tj_point);
						T = sp[i*factor].tStart;
						X = sp[i*factor].X;
						Y = sp[i*factor].Y;       
				}   
        msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory = pl_tj;  ;    
        msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthValue.value = vehicle_a + vehicle_b + vehicle_c + vehicle_d;
        msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleWidth.value = vehicle_width;
        /*if(platooningstate_reader!=0 && platooningstate_reader->hasData())    
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
        }*/ 
        MCM_publisher.publish(msg);       
      }         

 
    /*  
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
    */
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
    //ROS_INFO("setpointrequest_to_mcm_node namespace is: %s", sprtmcm.nh_ ->getNamespace().c_str());
    sprtmcm.run();
    ros::Rate rate(20.);
    ros::spin();
    return 0;

}
