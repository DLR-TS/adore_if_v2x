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
#include <adore_if_ros_msg/CooperativePlanningSet.h>
#include <adore/fun/afactory.h>
#include <adore/params/afactory.h>
#include <adore/mad/coordinateconversion.h>
#include <mcm_dmove_mcm_dmove/MCM.h>




namespace adore
{
    namespace if_ROS
    {
        class mcm_to_platoon  : public Baseapp
        {
            private:
            adore::mad::AReader<adore::fun::VehicleMotionState9d>* state_reader_;
            adore::mad::AWriter<adore::env::CooperativeUserPrediction>* coopUserWriter;
            adore::env::CooperativeUserPrediction cup;
            adore::fun::VehicleMotionState9d state_;
            ros::Subscriber MCMSubscriber_sim;  
            ros::Subscriber MCMSubscriber_sim_1;   
            ros::Subscriber MCMSubscriber_;         
            int utm_zone_;
            int generationDeltaTime;
            int mem_generationDeltaTime; //memory
            int dt_betweenMessages;
            int v2xStationID;
            bool ignoreOldMsg;
            int debug_level;
            public:

        void init(int argc, char **argv, double rate, std::string nodename)
        {
            v2xStationID = 0;  
            Baseapp::init(argc, argv, rate, nodename);
            Baseapp::initSim();     
            std::function<void()> run_fcn = (std::bind(&mcm_to_platoon::run_func, this)); 
            adore::if_ROS::FUN_Factory fun_factory(getRosNodeHandle());
            adore::if_ROS::ENV_Factory env_factory(getRosNodeHandle());
            state_reader_ = fun_factory.getVehicleMotionStateReader();
            coopUserWriter = env_factory.getCooperativeUserWriter();
            Baseapp::addTimerCallback(run_fcn);
            dt_betweenMessages = 0;
        }
        void receive_mcm(mcm_dmove_mcm_dmove::MCM msg)
        {
        
            cup.clear();
            getParam("PARAMS/UTMZone", utm_zone_);
            getParam("v2xStationID", v2xStationID); 
            getParam("PARAMS/nlp_planner/MCM_senser_debug_level", debug_level);           
            if(v2xStationID == msg.header.stationID.value) 
            {
                std::cout<<"\n[Ignored] I hear myself"<<v2xStationID<<"\t"<<msg.header.stationID.value;
                return; //not process ego MCM
            }
            

            if((state_reader_)!=0 && state_reader_->hasData() )
            {
                state_reader_->getData(state_);
                double t = state_.getTime();   //any better way to read time?
                double T = t;
                //printf("\n%i",msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size());
                //printf("\n%i",msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count);
                auto msg_vm = msg.maneuverCoordination.mcmParameters;
                std::cout<<"\nI am listening to id "<<msg.header.stationID.value<<" right now ";
    
                generationDeltaTime = msg.maneuverCoordination.generationDeltaTime.value;
                dt_betweenMessages = generationDeltaTime - mem_generationDeltaTime;
                mem_generationDeltaTime = generationDeltaTime;

                cup.setId(msg.header.stationID.value);
     
                cup.setLanePosition(msg_vm.maneuverContainer.vehicleManeuver.lanePosition.value);
                //printf("\n%i",msg_vm.maneuverContainer.vehicleManeuver.targetAutomationLevel.value);
                cup.setTargetAutomationLevel(msg_vm.maneuverContainer.vehicleManeuver.targetAutomationLevel.value);
                cup.setToletatedDistanceAhead((double) msg_vm.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value/100 );
                cup.setToletatedDistanceBehind((double) msg_vm.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value/100);
                cup.setVehicleLength(msg_vm.maneuverContainer.vehicleManeuver.vehicleLength.vehicleLengthValue.value);

                cup.setVehicleWidth((double)msg_vm.maneuverContainer.vehicleManeuver.vehicleWidth.value);

                
                if(msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size()>0)
                {
                    double X, Y;
                    double lat_deg = (double)msg_vm.basicContainer.referencePosition.latitude.value;
                    double lon_deg = (double)msg_vm.basicContainer.referencePosition.longitude.value;
                    adore::mad::CoordinateConversion::LatLonToUTMXY(lat_deg,lon_deg,utm_zone_,X,Y);      
                     //  std::cout<<"\n"<<utm_zone_;
                   // std::cout<<"\n"<<X<<"\t"<<Y<<"\t"<<lat_deg<<"\t"<<lon_deg;//<<"\t"<<msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size()<<"\t"<< cooperativePlanning.id;
                    for(int i=0; i<msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements.size();i++)
                    {     
                          //HEADING IS MISSING       
                        double dx = ((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaXCm.value * 0.01);
                        double dy = ((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaYCm.value  * 0.01);
                        double heading = msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].headingValue.value;
                        heading = adore::mad::CoordinateConversion::DegToRad(heading);
                        X += dx;
                        Y += dy;
                        //std::cout<<"\n"<<X<<"\t"<<Y<<"\t"<<heading;
                        cup.currentTrajectory.x.push_back(X);
                        cup.currentTrajectory.y.push_back(Y);
                        cup.currentTrajectory.psi.push_back(heading);
                        cup.currentTrajectory.t0.push_back(t);
                        cup.currentTrajectory.v.push_back(((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].absSpeed.value));
                        //std::cout<<"\n"<<((double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].absSpeed.value  * 0.01);
                        t += (double)msg_vm.maneuverContainer.vehicleManeuver.plannedTrajectory.elements[i].deltaTimeCs.value  * 0.1;
                        cup.currentTrajectory.t1.push_back(t);
                    }
                coopUserWriter->write(cup); 
                print_debug(msg);
            }
            
        
        }
    }

    virtual void run_func()
    {
         MCMSubscriber_= getRosNodeHandle()->subscribe<mcm_dmove_mcm_dmove::MCM>("MCM_receiver/MCM_in",1,&mcm_to_platoon::receive_mcm,this);
        MCMSubscriber_= getRosNodeHandle()->subscribe<mcm_dmove_mcm_dmove::MCM>("mcm_receiver/MCM_in",1,&mcm_to_platoon::receive_mcm,this);
        MCMSubscriber_sim= getRosNodeHandle()->subscribe<mcm_dmove_mcm_dmove::MCM>("MCM_in",1,&mcm_to_platoon::receive_mcm,this);
        MCMSubscriber_sim_1= getRosNodeHandle()->subscribe<mcm_dmove_mcm_dmove::MCM>("v2x/incoming/MCM",1,&mcm_to_platoon::receive_mcm,this);
        //
    }
    void print_debug(mcm_dmove_mcm_dmove::MCM msg)
    {
      if(debug_level)
      {
        std::cout<<"\n"<< "messageID: "<<unsigned(msg.header.messageID.value) ;
        std::cout<<"\n"<< "stationID: "<<unsigned(msg.header.stationID.value) ;
        std::cout<<"\n"<< "count: "<< unsigned(msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.plannedTrajectory.count);
        std::cout<<"\n"<< "TleratedDistanceAheadCmps: "<<(double)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceAheadCmps.value/1000.;
        std::cout<<"\n"<< "ToleratedDistanceBehind: "<<(double)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.vehicleCapabilities.toleratedDistanceBehindCmps.value/1000.;
        std::cout<<"\n"<< "targetAutomationLevel: "<<(unsigned)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.targetAutomationLevel.value ;
        std::cout<<"\n"<< "lanePosition: "<<(unsigned)msg.maneuverCoordination.mcmParameters.maneuverContainer.vehicleManeuver.lanePosition.value  ;
        std::cout<<"\n\n";

      }
    }


        };
    } // namespace if_ROS
} // namespace adore

int main(int argc, char** argv) 
{    
    adore::if_ROS::mcm_to_platoon mcm2platoon;
    mcm2platoon.init(argc, argv, 20., "mcm_to_platoon_node");
    ROS_INFO("mcm_to_platoon_node namespace is: %s", mcm2platoon.getRosNodeHandle()->getNamespace().c_str());
    mcm2platoon.run();
    return 0;
}