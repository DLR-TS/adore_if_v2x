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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/
#include <ros/ros.h>
#include <coordinate_conversion/coordinate_conversion.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>

/*
This node reads the set points (planned trajectory) and write it to MCM and broadcast it
*/
namespace adore
{
    namespace if_ROS
    {
        class DENMGenerator
        {

            double denm_event_pos_x_;
            double denm_event_pos_y_;
            std::string denm_closed_lanes_;

            ros::Publisher denm_publisher_;

            public:

            void send_denm(int argc, char** argv, double rate, std::string nodename)
            {
                ros::init(argc, argv, nodename);
                ros::NodeHandle *n = new ros::NodeHandle();

                n->getParam("PARAMS/DENMG/DENM_EVENT_POSITION_X", denm_event_pos_x_);
                n->getParam("PARAMS/DENMG/DENM_EVENT_POSITION_Y", denm_event_pos_y_);
                n->getParam("PARAMS/DENMG/CLOSED_LANES", denm_closed_lanes_);

                denm_publisher_ =
                    n->advertise<denm_v2_denm_pdu_descriptions::DENM>("v2x/incoming/DENM", 500, true);

                ROS_INFO_STREAM_ONCE("------- v2x_trafficlights params -------");
   
                ROS_INFO_STREAM_ONCE("DENM published on: " << denm_publisher_.getTopic());
                ROS_INFO_STREAM_ONCE("DENM X " << denm_event_pos_x_);
                ROS_INFO_STREAM_ONCE("DENM Y " << denm_event_pos_y_);
                ROS_INFO_STREAM_ONCE("----------------------------------------");
                
                //create denm here
                ros::Rate ros_rate(rate);
        
                denm_v2_denm_pdu_descriptions::DENM denm;

                denm.header.stationID.value = 1;
                denm.header.messageID.value =its_container_v2_its_container::ItsPduHeader_messageID::DENM;
                denm.header.protocolVersion.value = 2;
                
                denm.denm.locationPresent = 1;
                denm.denm.management.actionID.originatingStationID.value = 1030401;
                denm.denm.management.actionID.sequenceNumber.value = 199;
                denm.denm.management.detectionTime.value = 570200647;
                denm.denm.management.referenceTime.value = 570200647;

                double lat, lon;
                adore::mad::CoordinateConversion::UTMXYToLatLonDegree(denm_event_pos_x_, denm_event_pos_y_, 32, false, lat,lon);

                denm.denm.management.eventPosition.longitude.value = lon;
                denm.denm.management.eventPosition.latitude.value = lat;
                denm.denm.management.relevanceTrafficDirectionPresent = 1;
                denm.denm.management.relevanceTrafficDirection.value = its_container_v2_its_container::RelevanceTrafficDirection::UPSTREAM_TRAFFIC;
                denm.denm.management.validityDuration.value = 300;
                denm.denm.management.stationType.value = its_container_v2_its_container::StationType::SPECIAL_VEHICLES;

                //denm.denm.location.traces.elements.push_back();
                denm.denm.location.eventPositionHeadingPresent = 0;

                denm.denm.situationPresent = 1;
                denm.denm.situation.eventType.causeCode.value = its_container_v2_its_container::CauseCodeType::ACCIDENT;
                denm.denm.situation.eventType.subCauseCode.value = 0;
     
                denm.denm.situation.informationQuality.value = 6;
                
                denm.denm.alacartePresent = 1;
                denm.denm.alacarte.roadWorksPresent = 1;
                denm.denm.alacarte.roadWorks.lightBarSirenInUsePresent = 1;
                denm.denm.alacarte.roadWorks.lightBarSirenInUse.values.push_back(1);
                denm.denm.alacarte.roadWorks.lightBarSirenInUse.values.push_back(0);

                denm.denm.alacarte.roadWorks.closedLanesPresent = 1;
                denm.denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatusPresent = 1;
                denm.denm.alacarte.roadWorks.closedLanes.outerhardShoulderStatus.value = its_container_v2_its_container::HardShoulderStatus::CLOSED;

                denm.denm.alacarte.roadWorks.closedLanes.drivingLaneStatusPresent = 1;

                for (int i = 0; i < denm_closed_lanes_.length(); i++)
                {
                    denm.denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values.push_back((uint8_t) (denm_closed_lanes_[i]-48));
                }
                // denm.denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values.push_back(1);
                // denm.denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values.push_back(1);
                // denm.denm.alacarte.roadWorks.closedLanes.drivingLaneStatus.values.push_back(1);
                //denm.denm.location.traces.elements.push_back();

                while (ros::ok())
                {
                    denm_publisher_.publish(denm);

                    ros_rate.sleep();
                }

            }
        };
    }  // namespace if_ROS
}  // namespace adore

        int main(int argc, char** argv)
        {
            adore::if_ROS::DENMGenerator denmg;
            denmg.send_denm(argc, argv, 5.0, "v2x_trafficlights_node");
            return 0;
        }