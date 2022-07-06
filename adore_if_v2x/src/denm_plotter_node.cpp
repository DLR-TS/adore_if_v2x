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
*   Stephan Lapoehn
********************************************************************************/

#include<ros/ros.h>
#include <denm_v2_denm_pdu_descriptions/DENM.h>//please remove reference to v2x package
#include <plotlablib/figurestubfactory.h>
#include <adore/mad/coordinateconversion.h>
#include <set>

namespace adore
{
  namespace if_ROS
  {  
    class PlotDENMNode
    {
      private:

      ros::Subscriber denm_subscriber_;
      DLR_TS::PlotLab::AFigureStub* figure_;
      std::set<std::string> currently_ploted_DENMs_;

      public:

      PlotDENMNode(){}
      
      void init(int argc, char **argv, std::string nodename)
      {
        ros::init(argc, argv, nodename);

        ros::NodeHandle n_;
        denm_subscriber_ = n_.subscribe<denm_v2_denm_pdu_descriptions::DENM>("v2x/incoming/DENM", 500, &PlotDENMNode::receive_DENM, this);

        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        figure_ = fig_factory.createFigureStub(2);
        figure_->show();

      }

      void receive_DENM(denm_v2_denm_pdu_descriptions::DENM denm)
      {
          double x,y;
          adore::mad::CoordinateConversion::LatLonToUTMXY(denm.denm.management.eventPosition.latitude.value,denm.denm.management.eventPosition.longitude.value, 32, x, y);

          std::stringstream ss;

          ss << denm.header.stationID << "+" << denm.denm.situation.eventType.causeCode << "+" << denm.denm.situation.eventType.subCauseCode;

          std::string hash = ss.str();

          if(currently_ploted_DENMs_.find(hash) != currently_ploted_DENMs_.end()) return;
          else currently_ploted_DENMs_.insert(hash);


          figure_->plotTexture(hash,"../images/warning.png",x,y,1,PI,5,5);
          
      }

    };
  }
}

int main(int argc,char **argv)
{
    adore::if_ROS::PlotDENMNode fbn;    
    fbn.init(argc, argv, "plot_DENM_node");
    ros::spin();
    return 0;
}
