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

#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <coordinate_conversion/coordinate_conversion.h>

#include <adore_if_ros_msg/TCDConnectionStateTrace.h>
#include <dsrc_v2_mapem_pdu_descriptions/MAPEM.h>
#include <dsrc_v2_spatem_pdu_descriptions/SPATEM.h>
#include <unordered_map>

namespace adore
{
    namespace if_ROS
    {
        class V2XTrafficLights 
        {
          private:

            // intersection id to intersection data
            // one intersection may have multiple layers
            typedef std::unordered_multimap<u_int32_t, dsrc_v2_dsrc::IntersectionGeometry> MAPEMContainer;

            std::unordered_map<int, std::vector<int>> received_mapem_layers_;

            MAPEMContainer intersectionID_to_map_;

            // vehice position and time data
            double X_;
            double Y_;
            double Z_;
            double t_;


            ros::Subscriber odom_subscriber_;
            ros::Subscriber mapem_subscriber_;
            ros::Subscriber spatem_subscriber_;

            ros::Publisher connection_state_publisher;
            bool _use_system_time;
            int _utm_zone_;
            float _logging_frequency;

          public:
  
            V2XTrafficLights() : _utm_zone_{ 32 }, _use_system_time{ false }, _logging_frequency{ 1.0f }
            {
            }

            void odom_receive(nav_msgs::OdometryConstPtr msg)
            {
                X_ = msg->pose.pose.position.x;
                Y_ = msg->pose.pose.position.y;
                Z_ = msg->pose.pose.position.z;
                t_ = msg->header.stamp.toSec();
            }
            void init(int argc, char** argv, double rate, std::string nodename)
            {

                ros::init(argc, argv, nodename);
                ros::NodeHandle *n = new ros::NodeHandle();

                n->getParam("PARAMS/V2X_TL/UTMZone", _utm_zone_);
                n->getParam("PARAMS/V2X_TL/UseSystemTime", _use_system_time);

                odom_subscriber_ = n->subscribe("odom",10,&V2XTrafficLights::odom_receive,this);

                mapem_subscriber_ = n->subscribe<dsrc_v2_mapem_pdu_descriptions::MAPEM>(
                    "v2x/incoming/MAPEM", 100, &V2XTrafficLights::receive_mapem, this);

                spatem_subscriber_ = n->subscribe<dsrc_v2_spatem_pdu_descriptions::SPATEM>(
                    "v2x/incoming/SPATEM", 100, &V2XTrafficLights::receive_spatem, this);

                connection_state_publisher =
                    n->advertise<adore_if_ros_msg::TCDConnectionStateTrace>("ENV/tcd", 500, true);

                ROS_INFO_STREAM_ONCE("------- v2x_trafficlights params -------");
                ROS_INFO_STREAM_ONCE("MAPEM subscribed to: " << mapem_subscriber_.getTopic());
                ROS_INFO_STREAM_ONCE("SPATEM subscribed to: " << spatem_subscriber_.getTopic());
                ROS_INFO_STREAM_ONCE("Connection published on: " << connection_state_publisher.getTopic());
                ROS_INFO_STREAM_ONCE("_logging_frequency: " << _logging_frequency);
                ROS_INFO_STREAM_ONCE("_use_system_time: " << _use_system_time);
                ROS_INFO_STREAM_ONCE("_utm_zone_: " << _utm_zone_);
                ROS_INFO_STREAM_ONCE("----------------------------------------");
                ros::spin();
            }

            double getTime()
            {
                double time = 0;

                if (!_use_system_time)
                {
                    time = t_;
                }
                else
                {
                    using namespace std::chrono;
                    uint64_t ms = system_clock::now().time_since_epoch().count();
                    time = ((double)ms) / 1000000000;
                }

                return time;   
            }

            bool mapemLayerAlreadyProcessed(int station_id, int layer_id,
                                            std::unordered_map<int, std::vector<int>>& map_to_analyse)
            {
                try
                {
                    // station not found
                    if (map_to_analyse.find(station_id) == map_to_analyse.end())
                    {
                        std::vector<int> vec;
                        vec.push_back(layer_id);
                        map_to_analyse[station_id] = vec;
                        return false;
                    }
                    // station_id already exists, check whether layerid exists
                    else
                    {
                        auto vec = map_to_analyse[station_id];

                        if (std::count(vec.begin(), vec.end(), layer_id))
                            return true;
                        else
                            vec.push_back(layer_id);

                        return false;
                    }

                    return false;
                }
                catch (const std::out_of_range& ex)
                {
                    std::vector<int> messages;
                    messages.push_back(layer_id);
                    map_to_analyse[station_id] = messages;
                    return false;
                }
            }

            void receive_mapem(dsrc_v2_mapem_pdu_descriptions::MAPEM msg)
            {
                ROS_INFO_STREAM("mapem received: "  << "Station: " << msg.header.stationID.value << ", "
                                                    << "Layer: " << msg.map.layerID.value << ", "
                                                    << "Intersection-Count: " << (int)msg.map.intersections.count);

                if (mapemLayerAlreadyProcessed(msg.header.stationID.value, msg.map.layerID.value,
                                               received_mapem_layers_))
                    return;

                if (msg.map.intersectionsPresent)
                {
                    for (auto&& i : msg.map.intersections.elements)
                        intersectionID_to_map_.emplace(i.id.id.value, i);
                }
            }

            void receive_spatem(dsrc_v2_spatem_pdu_descriptions::SPATEM msg)
            {
                ROS_INFO_STREAM("spatem received: "  << "Station: " << msg.header.stationID.value << ", "
                                                     << "Intersection-Count: " << ", "
                                                     << (int)msg.spat.intersections.count);

                auto traces = generateConnectionStates(msg);
                
                for (auto&& connection_trace : traces)
                {
                    connection_state_publisher.publish(connection_trace);
                }
            }

            virtual std::vector<adore_if_ros_msg::TCDConnectionStateTrace> generateConnectionStates(dsrc_v2_spatem_pdu_descriptions::SPATEM msg)
            {
                std::vector<adore_if_ros_msg::TCDConnectionStateTrace> traces;

                // spat may contain information for mutliple intersections
                for(auto intersection : msg.spat.intersections.elements)
                {
                    int intersectionID = intersection.id.id.value;
                    
                    // spat may contain multiple signal group states
                    for(auto state_elements : intersection.states.elements)
                    {
                        // one signalgroup may apply to multiple connections on intersection
                        auto signaled_connection = getConnectionsForSignalgroup(intersectionID,state_elements.signalGroup.value);

                        // multiple 
                        auto state_of_signal_group = getConnectionStatesFromSPAT(state_elements.signalGroup.value,intersection);

                        for(auto connection : signaled_connection)
                        {
                            adore_if_ros_msg::TCDConnectionStateTrace trace;
                            trace.connection = connection;
                            trace.data = state_of_signal_group;
                            traces.push_back(trace);
                        }
                    }

                }
                return traces;
            }

            virtual std::vector<adore_if_ros_msg::Connection> getConnectionsForSignalgroup(int intersectionID, u_int8_t signal_group)
            {
                std::vector<adore_if_ros_msg::Connection> signaled_connections;
                
                auto iterators = intersectionID_to_map_.equal_range(intersectionID);

                for (auto i =iterators.first; i!=iterators.second; ++i)
                {
                    u_int32_t intersection_id = (*i).first;

                    for (auto&& lane : (*i).second.laneSet.elements)
                    {
                        if (!lane.connectsToPresent)
                            continue;

                        for (auto&& connection : lane.connectsTo.elements)
                        {
                            if (!connection.signalGroupPresent || connection.signalGroup.value != signal_group)
                                continue;


                            double trace_first_x;
                            double trace_first_y;
                            double trace_last_x;
                            double trace_last_y;

                            // we found a connection with a signal group -> build a connectionState
                            u_int8_t signal_group = connection.signalGroup.value;

                            adore_if_ros_msg::Connection adore_connection;

                            // calculate absolute coords for all offnet nodes of ingressing lane
                            calculateAbsoluteWGS84CoordsFromOffsetNodeList((*i).second.refPoint.lat.value,
                                                                           (*i).second.refPoint.long_.value,
                                                                           lane.nodeList);

                            auto ingressing_node = lane.nodeList.nodes.elements[0];

                            adore::mad::CoordinateConversion::LatLonToUTMXY(ingressing_node.delta.node_LatLon.lat.value,
                                                                            ingressing_node.delta.node_LatLon.lon.value,
                                                                            _utm_zone_, trace_first_x, trace_first_y);

                            adore_connection.first.x = trace_first_x;
                            adore_connection.first.y = trace_first_y;
                            adore_connection.first.z = 0;

                            // get the lane id that is connected with the current lane
                            u_int8_t egressing_lane_id = connection.connectingLane.lane.value;
                            dsrc_v2_dsrc::GenericLane egressing_lane =
                                getLaneFromMAPEM(intersection_id, egressing_lane_id, intersectionID_to_map_);

                            if (egressing_lane.nodeList.nodes.count < 1)
                                continue;

                            // generates wgs84 coordinate from offset values and saves it back in node
                            auto egressing_node = egressing_lane.nodeList.nodes.elements[0];
                            getWGSCoordinateFromOffset((*i).second.refPoint.lat.value,
                                                       (*i).second.refPoint.long_.value, egressing_node);

                            adore::mad::CoordinateConversion::LatLonToUTMXY(egressing_node.delta.node_LatLon.lat.value,
                                                                            egressing_node.delta.node_LatLon.lon.value,
                                                                            _utm_zone_, trace_last_x, trace_last_y);

                            adore_connection.last.x = trace_last_x;
                            adore_connection.last.y = trace_last_y;
                            adore_connection.last.z = 0;

                            signaled_connections.push_back(adore_connection);
                        }
                    }
                }
                return signaled_connections;
            }

            virtual std::vector<adore_if_ros_msg::TCDConnectionState>
            getConnectionStatesFromSPAT(u_int8_t signal_group, dsrc_v2_dsrc::IntersectionState& spat)
            {
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "------- Connection State --------");
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "Intersection " << spat.id.id.value);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "SignalGroup: " << signal_group);


                std::vector<adore_if_ros_msg::TCDConnectionState> ret;
                double DEFAULT_VALIDITY = 3.0;

                double vehicle_time = getTime();
                double vehicle_second_of_year = getSecondOfYearFromUTC(vehicle_time);
                double spat_second_of_year = getSecondOfYearFromMoy(spat.moy.value, spat.timeStamp.value);

                for (auto&& i : spat.states.elements)
                {
                    if (signal_group == i.signalGroup.value)
                    {
                        for (auto&& state : i.state_time_speed.elements)
                        {
                            adore_if_ros_msg::TCDConnectionState conn_state;
                            conn_state.state = state.eventState.value;

                            conn_state.likelyTime = vehicle_time +3.0;
                            conn_state.maxEndTime = vehicle_time +3.0;
                            conn_state.minEndTime = vehicle_time +3.0;
            
                            conn_state.likelyTime_present = true;
                            conn_state.maxEndTime_present = true;
                            conn_state.maxEndTime_present = true;

                            ret.push_back(conn_state);
                            break;
                        }
                    }
                }

                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "StatesCount: " << ret.size());
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "-----------------------------" << std::endl);
                return ret;
            }

            virtual dsrc_v2_dsrc::GenericLane getLaneFromMAPEM(u_int32_t intersection_id, u_int8_t lane_id,
                                                               MAPEMContainer& mapem_mmap)
            {
                dsrc_v2_dsrc::GenericLane lane;

                auto range = mapem_mmap.equal_range(intersection_id);

                for (auto it = range.first; it != range.second; ++it)
                {
                    for (auto&& i : (*it).second.laneSet.elements)
                    {
                        if (i.laneID.value == lane_id)
                        {
                            lane = i;
                            return lane;
                        }
                    }
                }
                return lane;
            }

            /**
             * --------------------------------------------------------------------------------
             * helper methods for geo- and time manipulation
             * --------------------------------------------------------------------------------
             **/

            virtual void getOffsetCoordinateFromNode(dsrc_v2_dsrc::NodeXY& node, double& lat, double& lon)
            {
                if (node.delta.node_XY1.x.value != 0)
                {
                    lat = node.delta.node_XY1.y.value;
                    lon = node.delta.node_XY1.x.value;
                    return;
                }
                if (node.delta.node_XY2.x.value != 0)
                {
                    lat = node.delta.node_XY2.y.value;
                    lon = node.delta.node_XY2.x.value;
                    return;
                }
                if (node.delta.node_XY3.x.value != 0)
                {
                    lat = node.delta.node_XY3.y.value;
                    lon = node.delta.node_XY3.x.value;
                    return;
                }
                if (node.delta.node_XY4.x.value != 0)
                {
                    lat = node.delta.node_XY4.y.value;
                    lon = node.delta.node_XY4.x.value;
                    return;
                }
                if (node.delta.node_XY5.x.value != 0)
                {
                    lat = node.delta.node_XY5.y.value;
                    lon = node.delta.node_XY5.x.value;
                    return;
                }
                if (node.delta.node_XY6.x.value != 0)
                {
                    lat = node.delta.node_XY6.y.value;
                    lon = node.delta.node_XY6.x.value;
                    return;
                }
            }

            virtual void getWGSCoordinateFromOffset(double lat_ref, double lon_ref, dsrc_v2_dsrc::NodeXY& delta_node)
            {
                // here all data is in m
                double delta_lat = 0;
                double delta_lon = 0;
                double meridian_r = 111752.85;

                getOffsetCoordinateFromNode(delta_node, delta_lat, delta_lon);

                // lat is now in degree
                delta_lat = delta_lat / meridian_r;

                // lon depends on latidue, see https://sciencing.com/equators-latitude-6314100.html
                double circle_radius = std::cos(lat_ref * M_PI / 180) * 6371000.785;
                circle_radius = 2 * M_PI * circle_radius;

                // lon is now degree
                delta_lon = delta_lon / (circle_radius / 360);

                // save final wgs84 coord in node_LatLon
                delta_node.delta.node_LatLon.lat.value = lat_ref + delta_lat;
                delta_node.delta.node_LatLon.lon.value = lon_ref + delta_lon;
            }

            virtual void calculateAbsoluteWGS84CoordsFromOffsetNodeList(double lat_ref, double lon_ref,
                                                                        dsrc_v2_dsrc::NodeListXY& nodeList)
            {
                double current_ref_lat = lat_ref;
                double current_ref_lon = lon_ref;

                for (auto&& i : nodeList.nodes.elements)
                {
                    getWGSCoordinateFromOffset(current_ref_lat, current_ref_lon, i);
                    current_ref_lat = i.delta.node_LatLon.lat.value;
                    current_ref_lon = i.delta.node_LatLon.lon.value;
                }
            }

            virtual int getLeapYearSeconds(double vehicle_time)
            {
                int years_since_epoch = (int)(vehicle_time / 31557600.0);  // 365.25 Julian year
                int year = 1970 + years_since_epoch;

                // 2021 % 4 = 1
                int leap = year % 4;
                return leap * 6 * 60 * 60;
            }

            virtual double getSecondOfYearFromUTC(double time)
            {
                return std::fmod(time - getLeapYearSeconds(time), 31557600.0f);
            }

            virtual double getSecondOfYearFromMoy(int32_t moy, u_int16_t dsecond)
            {
                return ((double)moy * 60.0 + (double)dsecond / 1000.0);
            }

            virtual double secondsToChange(double vehicle_second_of_year, double spat_second_of_year,
                                           double validility_time)
            {
                if (std::abs(vehicle_second_of_year - spat_second_of_year) > 3600)
                {
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency,"SPaT-2-Veh time diff is > 1h. Aborting.");
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --vehicle-soy: " << vehicle_second_of_year);
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --spat-soy: " << spat_second_of_year);
                    return -1;
                }
                // timestamp of spat is in the future
                else if (spat_second_of_year > vehicle_second_of_year)
                {
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency,"SPaT-Time is in the future. Aborting.");
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --vehicle-soy: " << vehicle_second_of_year);
                    // ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --spat-soy: " << spat_second_of_year);
                    return -1;
                }

                double spat_hour_and_minute = std::fmod((spat_second_of_year / 3600), 24);
                int spat_hour = (int)(spat_hour_and_minute);
                int spat_minute = (int)((spat_hour_and_minute - spat_hour) * 60);
                int spat_second = (int)(((int)spat_second_of_year) % 60);

                double vehicle_hour_and_minute = std::fmod((vehicle_second_of_year / 3600), 24);
                int vehicle_hour = (int)(vehicle_hour_and_minute);
                int vehicle_minute = (int)((vehicle_hour_and_minute - vehicle_hour) * 60);
                int vehicle_second = (int)(((int)vehicle_second_of_year) % 60);

                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,
                                          "SPAT-Hour  : " << spat_hour << "| Veh-Hour  : " << vehicle_hour);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,
                                          "SPAT-Minute: " << spat_minute << "| Veh-Minute: " << vehicle_minute);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,
                                          "SPAT-Second: " << spat_second << "| Veh-Second: " << vehicle_second);

                double spat_validitlity_in_this_hour = validility_time / 10;
                double spat_second_in_this_hour = std::fmod(spat_second_of_year, 3600.0);
                double seconds_until_change = -1;

                // example: if spat_validitlity_in_this_hour is 45 and spat_second_in_this_hour is 1250, 45s is meant to
                // be added to the next hour
                if (spat_validitlity_in_this_hour < spat_second_in_this_hour && vehicle_hour == spat_hour)
                {
                    ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "SPAT-validility overflow. Adding seconds to next "
                                                                  "full hour.");
                    double sec_to_next_hour = 3600 - vehicle_minute * 60 + vehicle_second;
                    seconds_until_change = sec_to_next_hour + spat_validitlity_in_this_hour;
                }
                // algorithm taken from
                // https://transportationops.org/sites/transops/files/Updated%20Signalized%20Intersection%20CCI%20-%2004242019%20ver%201.9.4.pdf
                else
                    seconds_until_change = (spat_validitlity_in_this_hour) - (vehicle_minute * 60 + vehicle_second);

                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "Rec. Validility: " << validility_time);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency, "Seconds till change: " << seconds_until_change);

                return seconds_until_change;
            }
        };
    }  // namespace if_ROS
}  // namespace adore

int main(int argc, char** argv)
{
    adore::if_ROS::V2XTrafficLights tlin;
    tlin.init(argc, argv, 2.0, "v2x_trafficlights_node");
    return 0;
}