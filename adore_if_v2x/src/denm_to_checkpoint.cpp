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
 *   Robert Markowski - initial API and implementation 
 *   Daniel Heß - convert output to checkpoint message
 ********************************************************************************/
#include <ros/ros.h>
#include <adore_if_ros/baseapp.h>
#include <adore/env/afactory.h>
#include <adore/mad/coordinateconversion.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>


/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
namespace if_ROS
{
class DENM2Checkpoint : public Baseapp
{
  private: 
  ros::Subscriber DENMSubscriber_;
  ros::Subscriber odomSubscriber_;
  adore::env::AFactory::TControlledConnectionWriter* checkpoint_writer_;
  double time_;
  
  public:
  void init(int argc, char **argv, double rate, std::string nodename)
  {        
    Baseapp::init(argc, argv, rate, nodename);
    Baseapp::initSim();
    std::function<void()> run_fcn = (std::bind(&DENM2Checkpoint::run_func, this)); 
    Baseapp::addTimerCallback(run_fcn);
    adore::if_ROS::ENV_Factory env_factory(getRosNodeHandle());
    checkpoint_writer_ = env_factory.getCheckPointWriter();
    time_ = 0.0;
  }
  void receive_denm(denm_v2_denm_pdu_descriptions::DENM msg)
  {
    // check if denm is relevant for this app
    if(msg.denm.alacartePresent > 0
      && msg.denm.alacarte.roadWorksPresent > 0
      && msg.denm.alacarte.roadWorks.closedLanesPresent > 0
      && msg.denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent > 0)
    {

        double x,y,z;
        adore::mad::CoordinateConversion::LatLonToUTMXY(
            msg.denm.management.eventPosition.latitude.value,
            msg.denm.management.eventPosition.longitude.value,
            32, x, y
        );
        z = msg.denm.management.eventPosition.altitude.altitudeValue.value;
        adore::env::ControlledConnection con(x,y,z,x,y,z);
        double end_time = time_+2.0;
        double open_end = end_time + 10000.0;
        adore::env::ConnectionStateEvent e0(adore::env::ConnectionState::STOP___AND___REMAIN,end_time,false,false,0.0,0.0);
        adore::env::ConnectionStateEvent e1(adore::env::ConnectionState::PROTECTED___MOVEMENT___ALLOWED,open_end,false,false,0.0,0.0);
        con.insertStateEvent(e0);
        con.insertStateEvent(e1);
        checkpoint_writer_->write(con);
    }
  }
  void receive_odom(nav_msgs::OdometryConstPtr msg)
  {
      time_ = msg->header.stamp.toSec();
  }
  virtual void run_func()
  {
    DENMSubscriber_= getRosNodeHandle()->subscribe("v2x/incoming/DENM",1,&DENM2Checkpoint::receive_denm,this);
    odomSubscriber_= getRosNodeHandle()->subscribe("localization",1,&DENM2Checkpoint::receive_odom,this);
  }
};
} // namespace if_ROS
} // namespace adore




int main(int argc,char **argv)
{

    adore::if_ROS::DENM2Checkpoint denm_checkpoint;
    denm_checkpoint.init(argc, argv, 1., "denm2checkpoint_node");
    denm_checkpoint.run();
    return 0;

}
