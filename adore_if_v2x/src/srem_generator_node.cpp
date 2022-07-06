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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/
#include <ros/ros.h>
#include <dsrc_v2_srem_pdu_descriptions/SREM.h>
#include <dsrc_v2_dsrc/SignalRequestPackage.h>
#include <nav_msgs/Odometry.h>
#include <coordinate_conversion/coordinate_conversion.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <thread>

enum DEBUGMODE {OFF,MODE1,MODE2};

/*
This node reads a file with SREM-zones (polygon + ID). If ego-vehicle is in one of these SREM-zones the zone-specific SREM is broadcasted.
*/
namespace adore
{
    namespace if_ROS
    {
        class SREMGenerator 
        {
            private:
            ros::Publisher srem_publisher_;
            ros::Subscriber state_subscriber_;
            ros::NodeHandle* nh_;
            
            dsrc_v2_srem_pdu_descriptions::SREM srem_;

            std::vector<std::vector<double> > srem_zones_;


            int request_counter_;

            DEBUGMODE debug_mode_;
            DEBUGMODE last_debug_mode_;

            double rate_;
            double x_,y_,z_,t_;
            bool position_initialized_;

            public:
            void receive_position(nav_msgs::OdometryConstPtr msg)
            {
                x_ = msg->pose.pose.position.x;
                y_ = msg->pose.pose.position.y;
                z_ = msg->pose.pose.position.z;
                t_ = msg->header.stamp.toSec();
                position_initialized_ = true;
            }
            void init(int argc, char **argv, double rate, std::string nodename)
            {        
                position_initialized_ = false;
                ros::init(argc, argv, nodename);
                nh_ = new ros::NodeHandle();
                state_subscriber_ = nh_->subscribe("localization",SREMGenerator::receive_position);
                srem_publisher_ =  nh_->advertise<dsrc_v2_srem_pdu_descriptions::SREM>("v2x/outgoing/SREM",1);
                rate_ = rate;
                request_counter_ = 1;

                //read file with SREM-zones
                std::string srem_zones_file = "";
                 nh_->getParam("PARAMS/sremZonesFile", srem_zones_file);

                std::string line;
                std::ifstream file(srem_zones_file);
                int pos;

                if(!file.is_open())
                {
                    std::cout<<"failed to open file: "<<srem_zones_file<<std::endl;
                }
                else
                {
                    while(std::getline(file,line))
                    {

                        if(line.find("#")!=std::string::npos)continue;

                        if(line.find(",")==std::string::npos)
                        {
                            std::cout<<"error in file: "<<srem_zones_file<<std::endl;
                            break;
                        }
                        else
                        {
                            std::vector<double> srem_zone;
                            double value;
                            
                            while(line.find(',')!=std::string::npos) 
                            {
                                std::stringstream ss1;
                                pos = line.find(',');
                                ss1<<line.substr(0,pos);
                                ss1>>value;
                                srem_zone.push_back(value);
                                line = line.substr(pos+1);   
                            }

                            std::stringstream ss2;
                            ss2<<line;
                            ss2>>value;
                            srem_zone.push_back(value);

                            srem_zones_.push_back(srem_zone);
                        }
                    }
                }

                //debugging
                debug_mode_ = OFF;
                last_debug_mode_ = OFF;
            }

            bool isInArea(int& connection_id, int& intersection_id)
            {
                if(position_initialized_ )
                {
                    for(int i=0;i<srem_zones_.size();i++)
                    {
                        //save polygon of current SREM-zone without IDs
                        int j = 3; //startindex of polygon data 
                        int counter = 0;
                        std::vector<double> datapoint;
                        std::vector<std::vector<double> > srem_zone_polygon;

                        if((srem_zones_[i].size()-j) % 2 == 0)
                        {
                            while(j < srem_zones_[i].size())
                            {
                                while(counter < 2)
                                {
                                    datapoint.push_back(srem_zones_[i][j]);
                                    j++;
                                    counter++;
                                }
                                
                                srem_zone_polygon.push_back(datapoint);

                                counter = 0;
                                datapoint.clear();
                            }
                        }
                        else
                        {
                            std::cout<<"error in file: polygon data might be corrupted, the count of numbers is not divisible by 2"<<std::endl;
                            return false;
                        }


                        double xi = x_;
                        double yi = y_;
                        int cross_counter = 0;

                        for(int j=0;j<srem_zone_polygon.size()-1;j++)
                        {
                            double x0,x1,y0,y1;
                            if(srem_zone_polygon[j][1]==srem_zone_polygon[j+1][1])continue;//line is parallel to x-axis (ray is also assumed parallel to x-axis)
                            if(srem_zone_polygon[j][1]<srem_zone_polygon[j+1][1])
                            {
                                x0 = srem_zone_polygon[j][0];     y0 = srem_zone_polygon[j][1];
                                x1 = srem_zone_polygon[j+1][0];   y1 = srem_zone_polygon[j+1][1];
                            }
                            else
                            {
                                x1 = srem_zone_polygon[j][0];     y1 = srem_zone_polygon[j][1];
                                x0 = srem_zone_polygon[j+1][0];   y0 = srem_zone_polygon[j+1][1];
                            }
                            if(yi<y0||y1<yi)continue;//line's y-range does not contain point (ray will not intersect line)
                            
                            double dx = x1-x0;
                            double dy = y1-y0;
                            double x_cross = (yi-y0)/dy*dx+x0-xi;//x distance from pi where ray parallel to x-axis is crossed
                        
                            if(x_cross>=0.0)cross_counter++;//ray starting at pi, parallel to x-axis is crossing line right of pi
                        }

                        bool inside = (cross_counter%2)==1;//is in, if cross_counter is odd

                        if(inside)
                        {
                            connection_id = srem_zones_[i][2];
                            intersection_id = srem_zones_[i][1];
                            std::cout<<"zone_id="<<srem_zones_[i][0]<<", intersection id="<<srem_zones_[i][1]<<", connection_id="<<srem_zones_[i][2]<<std::endl;
                            return inside;
                        }
                    }
                    return false;
                }
                else
                {
                    return false;
                }
            }

            void send_srem()
            {
                double second_of_year = getSecondOfYearFromUTC();
                int dSecond = getDSecond();

                double lat = 0.0;
                double lon = 0.0;
                int connection_id = -1;
                int intersection_id = -1;

                if(position_initialized_ )
                {

                    adore::mad::CoordinateConversion::UTMXYToLatLonDegree(x_, y_, 32, false, lat,lon);
                }

                if(isInArea(connection_id,intersection_id))
                {

                    srem_.header.protocolVersion.value = 2;
                    srem_.header.messageID.value = 9;
                    srem_.header.stationID.value = 111;

                    srem_.srm.timeStampPresent = true;
                    srem_.srm.timeStamp.value = (int)(second_of_year/60);
                    srem_.srm.second.value = dSecond;

                    srem_.srm.requestor.id.choice = 1;
                    srem_.srm.requestor.id.stationID.value = 111;
                    srem_.srm.requestor.positionPresent = true;
                    srem_.srm.requestor.position.position.lat.value = lat;
                    srem_.srm.requestor.position.position.long_.value = lon;

                    if(connection_id!=-1) //sending requestor + requests package
                    {
                        srem_.srm.requestsPresent = true;

                        dsrc_v2_dsrc::SignalRequestPackage signal_request_package;
                        signal_request_package.request.id.id.value = intersection_id;
                        signal_request_package.request.requestID.value = 1;
                        signal_request_package.request.requestType.value = 1;
                        signal_request_package.request.inBoundLane.choice = 2;
                        signal_request_package.request.inBoundLane.connection.value = connection_id;
                        srem_.srm.requests.count = 1;
                        if(request_counter_==1)srem_.srm.requests.elements.push_back(signal_request_package);
                        request_counter_++;
                    }
                    else //reset requests package
                    {
                        srem_.srm.requestsPresent = false;
                        srem_.srm.requests.count = 0;
                        srem_.srm.requests.elements.clear();
                        request_counter_ = 1;
                    }

                    srem_publisher_.publish(srem_);
                }

                //debugging
                if(debug_mode_==MODE1||debug_mode_==MODE2)
                {
                    //reset requests package
                    if(last_debug_mode_!=debug_mode_)
                    {
                        srem_.srm.requestsPresent = false;
                        srem_.srm.requests.count = 0;
                        srem_.srm.requests.elements.clear();
                        request_counter_ = 1;
                    }

                    srem_.header.protocolVersion.value = 2;
                    srem_.header.messageID.value = 9;
                    srem_.header.stationID.value = 111;

                    srem_.srm.timeStampPresent = true;
                    srem_.srm.timeStamp.value = (int)(second_of_year/60);
                    srem_.srm.second.value = dSecond;

                    srem_.srm.requestor.id.choice = 1;
                    srem_.srm.requestor.id.stationID.value = 111;
                    srem_.srm.requestor.positionPresent = true;
                    srem_.srm.requestor.position.position.lat.value = 52.298009858960725;
                    srem_.srm.requestor.position.position.long_.value = 10.542583354042584;

                    if(debug_mode_==MODE2) //sending requestor + requests package
                    {
                        srem_.srm.requestsPresent = true;

                        dsrc_v2_dsrc::SignalRequestPackage signal_request_package;
                        signal_request_package.request.id.id.value = 555;
                        signal_request_package.request.requestID.value = 1;
                        signal_request_package.request.requestType.value = 1;
                        signal_request_package.request.inBoundLane.choice = 2;
                        signal_request_package.request.inBoundLane.connection.value = 12;
                        srem_.srm.requests.count = 1;
                        if(request_counter_==1)srem_.srm.requests.elements.push_back(signal_request_package);
                        request_counter_++;
                    }

                    srem_publisher_.publish(srem_);
                    last_debug_mode_ = debug_mode_;
                }
            }

             void run()
            {
                ros::Rate r(rate_); 
                while(ros::ok())send_srem();
            }


            double getTime()
            {
                double time = 0;

                using namespace std::chrono;
                uint64_t ms = system_clock::now().time_since_epoch().count();
                time = ((double)ms) / 1000000000;

                return time;   
            }

            double getSecondOfYearFromUTC()
            {
                time_t now;
                struct tm newyear;
                double seconds;

                time(&now);  // get current time; same as: now = time(NULL)

                newyear = *gmtime(&now);

                newyear.tm_hour = 0; newyear.tm_min = 0; newyear.tm_sec = 0;
                newyear.tm_mon = 0;  newyear.tm_mday = 1;

                seconds = difftime(now,mktime(&newyear));

                return seconds;
            }

            int getDSecond()
            {
                double time = getSecondOfYearFromUTC() + (getTime() - (int)getTime());
                int dSecond = (int)((time - ((int)(time/60))*60)*1000);

                return dSecond;
            }

            //debugging
            void setDebugMode(DEBUGMODE input)
            {
                debug_mode_ = input;
            }
            DEBUGMODE getDebugMode()
            {
                return debug_mode_;
            }
        };
    }  // namespace if_ROS
}  // namespace adore

adore::if_ROS::SREMGenerator sremg;
bool terminated = false;

void kbinput()
{
    while(!terminated)
    {
        int c = std::cin.get();
        if( c == 49) 
        {
            if(sremg.getDebugMode()!=MODE1)
            {
                sremg.setDebugMode(MODE1);
                std::cout<<"DebugMode1 ON"<<std::endl;
            }
            else
            {
                sremg.setDebugMode(OFF);
                std::cout<<"DebugMode1 OFF"<<std::endl;
            }
        }
        if( c == 50 )
        {
            if(sremg.getDebugMode()!=MODE2)
            {
                sremg.setDebugMode(MODE2);
                std::cout<<"DebugMode2 ON"<<std::endl;
            }
            else
            {
                sremg.setDebugMode(OFF);
                std::cout<<"DebugMode2 OFF"<<std::endl;
            }
        }
    }
}

int main(int argc, char** argv)
{
    std::cout<<"srem_generator_node:"<<std::endl;
    std::cout<<"press 1 + Enter to (de-)activate DebugMode1: sending permanently requestor package dummy"<<std::endl;
    std::cout<<"press 2 + Enter to (de-)activate DebugMode2: sending permanently requestor + requests package dummy"<<std::endl;
    std::thread kbinput_thread(kbinput);
    sremg.init(argc, argv, 10, "srem_generator_node");
    sremg.run();
    terminated = true;
    ros::shutdown();
    return 0;
}
