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
#include <coordinate_conversion/coordinate_conversion.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>
#include <adore_if_ros_msg/TCDConnectionStateTrace.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>


/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
namespace if_ROS
{
class DENM2Checkpoint 
{
  private: 
  ros::Subscriber DENMSubscriber_;
  ros::Subscriber odomSubscriber_;
  ros::Publisher publisher_;
  double time_;
  
  public:
  void init(int argc, char **argv, double rate, std::string nodename)
  {        
    ros::init(argc, argv, nodename);
    ros::NodeHandle nh;
    DENMSubscriber_= nh.subscribe("v2x/incoming/DENM",1,&DENM2Checkpoint::receive_denm,this);
    odomSubscriber_= nh.subscribe("localization",1,&DENM2Checkpoint::receive_odom,this);
    publisher_ = nh.advertise<adore_if_ros_msg::TCDConnectionStateTrace>("ENV/checkpoints",10);
    time_ = 0.0;
    ros::spin();
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
        adore_if_ros_msg::TCDConnectionStateTrace result;
        result.connection.first.x = x;
        result.connection.first.y = y;
        result.connection.first.z = z;
        result.connection.last = result.connection.first;
        double end_time = time_+2.0;
        double open_end = end_time + 10000.0;
        adore_if_ros_msg::TCDConnectionState s0,s1;
        s0.state = adore_if_ros_msg::TCDConnectionState::STOP___AND___REMAIN;
        s0.minEndTime = end_time;
        s0.maxEndTime_present = false;
        s0.likelyTime_present = false;
        s1 = s0;
        s1.minEndTime = open_end;
        s1.state = adore_if_ros_msg::TCDConnectionState::PROTECTED___MOVEMENT___ALLOWED;
        result.data.push_back(s0);
        result.data.push_back(s1);
        publisher_.publish(result);
    }
  }
  void receive_odom(nav_msgs::OdometryConstPtr msg)
  {
      time_ = msg->header.stamp.toSec();
  }
};
} // namespace if_ROS
} // namespace adore




int main(int argc,char **argv)
{

    adore::if_ROS::DENM2Checkpoint denm_checkpoint;
    denm_checkpoint.init(argc, argv, 1., "denm2checkpoint_node");
    return 0;

}
