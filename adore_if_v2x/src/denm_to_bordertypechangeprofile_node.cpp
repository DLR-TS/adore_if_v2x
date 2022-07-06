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
 ********************************************************************************/
#include <ros/ros.h>
#include <coordinate_conversion/coordinate_conversion.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>
#include <adore_if_ros_msg/BorderTypeChangeProfile.h>

/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
namespace if_ROS
{
class denm_to_bordertypechangeprofile 
{
  private: 
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  ros::NodeHandle* nh_;
  double rate_;
  
  public:
  void init(int argc, char **argv, double rate, std::string nodename)
  {        
    ros::init(argc, argv, nodename);
    nh_ = new ros::NodeHandle();
    rate_ = rate;
    subscriber_ = nh_->subscribe("v2x/incoming/DENM",1,&denm_to_bordertypechangeprofile::receive_denm,this);
    publisher_ = nh_->advertise<adore_if_ros_msg::BorderTypeChangeProfile>("ENV/BorderTypeChangeProfile",100);
    ros::spin();
  }
  void receive_denm(denm_v2_denm_pdu_descriptions::DENMConstPtr msg)
  {
    // check if denm is relevant for this app
    if(msg->denm.alacartePresent > 0
      && msg->denm.alacarte.roadWorksPresent > 0
      && msg->denm.alacarte.roadWorks.closedLanesPresent > 0
      && msg->denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent > 0)
    {
      // gather positions
      // x=lat, y=lon, z=alt coords
      std::vector<geometry_msgs::Point> spherical_coords;
      // base point: event position
      geometry_msgs::Point p;
      p.x = msg->denm.management.eventPosition.latitude.value;
      p.y = msg->denm.management.eventPosition.longitude.value;
      p.z = msg->denm.management.eventPosition.altitude.altitudeValue.value;
      spherical_coords.push_back(p);
      // delta coord: coord(n+1) = delta(n)+coord(n)
      if(msg->denm.situationPresent > 0 && msg->denm.situation.eventHistoryPresent > 0)
      {
        auto history = msg->denm.situation.eventHistory;
        for(int i = 0; i < history.elements.size(); i++)
        {          
          geometry_msgs::Point p;
          p.x = history.elements.at(i).eventPosition.deltaLatitude.value  + spherical_coords.at(i).x;
          p.y = history.elements.at(i).eventPosition.deltaLongitude.value + spherical_coords.at(i).y;
          p.z = history.elements.at(i).eventPosition.deltaAltitude.value  + spherical_coords.at(i).z;
          spherical_coords.push_back(p);
        }
      }
      // transform spherical to utm coordinates
      std::vector<geometry_msgs::Point> utm_coords;
      for(auto c = spherical_coords.begin(); c!=spherical_coords.end(); c++)
      {
        double x,y;
        adore::mad::CoordinateConversion::LatLonToUTMXY(c->x, c->y, 32, x, y);
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = 0;
        utm_coords.push_back(p);        
      }
      // determine closed lanes, order is from left to right
      auto bitstr = msg->denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values;
      std::vector<int> closedLanes;
      for(auto b = bitstr.begin(); b!= bitstr.end(); b++)
      {
        // if((*b) > 0) closedLanes.push_back(adore::env::BorderBased::BorderType::DRIVING_DELETED);
        // else closedLanes.push_back(adore::env::BorderBased::BorderType::DRIVING);
        if((*b) > 0) closedLanes.push_back(2);
        else closedLanes.push_back(1);
      }
      // when no history is given, duplicate the event position to have start and end
      if(utm_coords.size()==1) utm_coords.push_back(utm_coords.at(0));
      // generate and send msgs
      for(int i = 1; i<utm_coords.size(); i++)
      {
        adore_if_ros_msg::BorderTypeChangeProfile result;
        result.start = utm_coords.at(i-1);
        result.end = utm_coords.at(i);
        for(auto i:closedLanes)result.bordertypeprofile.push_back(i);
        publisher_.publish(result);
      }
    }

  }
};
} // namespace if_ROS
} // namespace adore




int main(int argc,char **argv)
{

    adore::if_ROS::denm_to_bordertypechangeprofile denm_btcp;
    denm_btcp.init(argc, argv, 1., "denm_to_bordertypechangeprofile_node");
    return 0;

}
