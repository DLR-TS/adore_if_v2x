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
class denm_to_bordertypechangeprofile : public Baseapp
{
  private: 
  ros::Subscriber DENMSubscriber_;
  adore::mad::AWriter<adore::env::BorderTypeChangeProfile>* btcp_writer_;
  
  public:
  void init(int argc, char **argv, double rate, std::string nodename)
  {        
    Baseapp::init(argc, argv, rate, nodename);
    Baseapp::initSim();
    std::function<void()> run_fcn = (std::bind(&denm_to_bordertypechangeprofile::run_func, this)); 
    Baseapp::addTimerCallback(run_fcn);
    adore::if_ROS::ENV_Factory env_factory(getRosNodeHandle());
    btcp_writer_ = env_factory.getBorderTypeChangeProfileWriter();
  }
  void receive_denm(denm_v2_denm_pdu_descriptions::DENM msg)
  {
    // check if denm is relevant for this app
    if(msg.denm.alacartePresent > 0
      && msg.denm.alacarte.roadWorksPresent > 0
      && msg.denm.alacarte.roadWorks.closedLanesPresent > 0
      && msg.denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent > 0)
    {
      // gather positions
      // x=lat, y=lon, z=alt coords
      std::vector<adore::env::BorderBased::Coordinate> spherical_coords;
      // base point: event position
      spherical_coords.push_back(adore::env::BorderBased::Coordinate(
          msg.denm.management.eventPosition.latitude.value,
          msg.denm.management.eventPosition.longitude.value,
          msg.denm.management.eventPosition.altitude.altitudeValue.value));
      // delta coord: coord(n+1) = delta(n)+coord(n)
      if(msg.denm.situationPresent > 0 && msg.denm.situation.eventHistoryPresent > 0)
      {
        auto history = msg.denm.situation.eventHistory;
        for(int i = 0; i < history.elements.size(); i++)
        {          
          spherical_coords.push_back(adore::env::BorderBased::Coordinate(
            history.elements.at(i).eventPosition.deltaLatitude.value  + spherical_coords.at(i).m_X,
            history.elements.at(i).eventPosition.deltaLongitude.value + spherical_coords.at(i).m_Y,
            history.elements.at(i).eventPosition.deltaAltitude.value  + spherical_coords.at(i).m_Z
          ));
        }
      }
      LOG_T("spherical size: %i", spherical_coords.size());
      // transform spherical to utm coordinates
      std::vector<adore::env::BorderBased::Coordinate> utm_coords;
      for(auto c = spherical_coords.begin(); c!=spherical_coords.end(); c++)
      {
        double x,y;
        adore::mad::CoordinateConversion::LatLonToUTMXY(c->m_X, c->m_Y, 32, x, y);
        LOG_T("x, y: %.1f, %.1f", x, y);
        utm_coords.push_back(adore::env::BorderBased::Coordinate(x,y,0));        
      }
      LOG_T("utm size: %i", utm_coords.size());
      // determine closed lanes, order is from left to right
      auto bitstr = msg.denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values;
      std::vector<adore::env::BorderBased::BorderType::TYPE> closedLanes;
      for(auto b = bitstr.begin(); b!= bitstr.end(); b++)
      {
        if((*b) > 0) closedLanes.push_back(adore::env::BorderBased::BorderType::DRIVING_DELETED);
        else closedLanes.push_back(adore::env::BorderBased::BorderType::DRIVING);
      }
      // when no history is given, duplicate the event position to have start and end
      if(utm_coords.size()==1) utm_coords.push_back(utm_coords.at(0));
      LOG_T("utm size 2: %i", utm_coords.size());
      // generate and send msgs
      for(int i = 1; i<utm_coords.size(); i++)
      {
        adore::env::BorderTypeChangeProfile btcp;
        btcp.start = utm_coords.at(i-1);
        btcp.end   = utm_coords.at(i);
        btcp.borderTypeProfile = closedLanes;
        btcp_writer_->write(btcp);
        LOG_T("Write btcp.");
      }
    }

  }
  virtual void run_func()
  {
    DENMSubscriber_= getRosNodeHandle()->subscribe<denm_v2_denm_pdu_descriptions::DENM>("v2x/incoming/DENM",1,&denm_to_bordertypechangeprofile::receive_denm,this);
  }
};
} // namespace if_ROS
} // namespace adore




int main(int argc,char **argv)
{

    adore::if_ROS::denm_to_bordertypechangeprofile denm_btcp;
    denm_btcp.init(argc, argv, 1., "denm_to_bordertypechangeprofile_node");
    ROS_INFO("denm_to_bordertypechangeprofile_node namespace is: %s", denm_btcp.getRosNodeHandle()->getNamespace().c_str());
    denm_btcp.run();
    return 0;

}
