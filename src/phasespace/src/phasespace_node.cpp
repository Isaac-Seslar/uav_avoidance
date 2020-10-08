#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dirent.h"
#include <fstream>
#include <string>
#include <map>
#include "owl.hpp"
#include "json.hpp" // https://github.com/nlohmann/json/

using json = nlohmann::json;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "phasespace");
  ros::NodeHandle nh;

  // Node Parameters
  ros::NodeHandle nhp("~");
  std::string server_ip, rigid_body_folder, profile, tf_frame;
  nhp.param<std::string>("server_ip", server_ip, "192.168.1.230");
  nhp.param<std::string>("rigid_body_folder", rigid_body_folder, ros::package::getPath("phasespace")+std::string("/config"));
  nhp.param<std::string>("profile",profile,"profile-1");
  nhp.param<std::string>("tf_frame",tf_frame,"map");

  // PhaseSpace objects
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Rigids rigids;

  // Initialize
  ROS_INFO("Connecting to PhaseSpace server...");
  if(owl.open(server_ip) <= 0 || owl.initialize("profile="+profile) <= 0)
  {
    ROS_ERROR_STREAM(owl.lastError());
    ros::shutdown();
    return 0;
  }
  ROS_INFO("Connection Established!");

  // Read JSON file defining rigid bodies. Using parser: https://github.com/nlohmann/json/
  std::vector<json> j;
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(rigid_body_folder.c_str())) != NULL)
  {
    while((ent = readdir(dir)) != NULL)
    {
      std::string filename = rigid_body_folder + "/" + ent->d_name;
      if (filename.substr(filename.rfind('.') + 1) == "json")
      {
        std::ifstream fs(filename);
        if (fs.good())
        {
          json ji;
          fs >> ji;
          j.push_back(ji);
        }
      }
    }
  }
  else
  {
    ROS_ERROR_STREAM("Directory '" << rigid_body_folder << "' does not exist");
  }
  ROS_INFO("Found JSON configuration files for %lu rigid bodies.", j.size());

  // Generate trackers for OWL rigids
  std::vector<OWL::TrackerInfo> tracker_info;
  for (int i = 0; i < j.size(); i++)
  {
    tracker_info.push_back(OWL::TrackerInfo(j.at(i)["trackers"].at(0)["id"], j.at(i)["trackers"].at(0)["type"], j.at(i)["trackers"].at(0)["name"], j.at(i)["trackers"].at(0)["options"]));
  }
  owl.createTrackers(tracker_info.data(), tracker_info.data()+tracker_info.size());

  // Add markers to trackers
  for (int i = 0; i < j.size(); i++)
  {
    std::vector<OWL::MarkerInfo> marker_info;
    for (int k = 0; k < j.at(i)["trackers"].at(0)["markers"].size(); k++)
    {
      json marker = j.at(i)["trackers"].at(0)["markers"].at(k);
      marker_info.push_back(OWL::MarkerInfo(marker["id"], j.at(i)["trackers"].at(0)["id"], marker["name"], marker["options"]));
    }
    owl.assignMarkers(marker_info.data(), marker_info.data()+marker_info.size());
  }
  
  // Publishers
  tf::TransformBroadcaster tfbr;
  ros::Publisher vizPub = nh.advertise<visualization_msgs::MarkerArray>("markers",10);
  std::map<int,ros::Publisher> posePubs;
  std::map<int,std::string> id2name;
  std::map<int,Eigen::Affine3d> id2linkTf;
  for (int i = 0; i < j.size(); i++)
  {
    posePubs[j.at(i)["trackers"].at(0)["id"].get<int>()] = nh.advertise<geometry_msgs::PoseStamped>(j.at(i)["trackers"].at(0)["name"].get<std::string>()+std::string("/pose"),10);
    id2name[j.at(i)["trackers"].at(0)["id"].get<int>()] = j.at(i)["trackers"].at(0)["name"].get<std::string>();
    Eigen::Affine3d tf(Eigen::Affine3d::Identity());
    if (j.at(i)["trackers"].at(0).count("link_transform"))
    {
      const auto& ltf = j.at(i)["trackers"].at(0)["link_transform"];
      Eigen::Translation3d trans(ltf["translation"]["x"].get<double>(), ltf["translation"]["y"].get<double>(), ltf["translation"]["z"].get<double>());
      Eigen::Quaterniond quat(ltf["quaternion"]["w"].get<double>(), ltf["quaternion"]["x"].get<double>(), ltf["quaternion"]["y"].get<double>(), ltf["quaternion"]["z"].get<double>());
      tf = trans * quat;
    }
    id2linkTf[j.at(i)["trackers"].at(0)["id"].get<int>()] = tf;
  }
  
  // start streaming
  owl.streaming(true);
  
  // Main loop
  while(ros::ok() && owl.isOpen() && owl.property<int>("initialized"))
  {
    // Check if new data available
    const OWL::Event *event = owl.nextEvent(1000);
    if(!event) continue;

    switch(event->type_id())
    {
      // Output error
      case OWL::Type::ERROR:
      {
        ROS_WARN_STREAM(event->name() << ": " << event->str() << std::endl);
        break;
      }
      case OWL::Type::FRAME:
      {
        ros::Time timeNow = ros::Time::now();
        //std::cout << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << std::endl;
        for(const OWL::Event *e = event->begin(); e != event->end(); e++)
        {
          switch(e->type_id())
          {
            case OWL::Type::MARKER:
            {
              if(e->get(markers) > 0)
              {
                // Construct marker visualization msg
                visualization_msgs::MarkerArray vizMarkerArray;
                visualization_msgs::Marker vizMarker;
                vizMarker.header.frame_id = tf_frame; 
                vizMarker.header.stamp = timeNow;
                vizMarker.action = visualization_msgs::Marker::ADD;
                vizMarker.type = visualization_msgs::Marker::POINTS;
                vizMarker.scale.x = 0.02;
                vizMarker.scale.y = 0.02;
                vizMarker.pose.orientation.w = 1.0;
                vizMarker.color.r = 1.0f;
                vizMarker.color.a = 1.0f;
                std::vector<std_msgs::ColorRGBA> colors(1,std_msgs::ColorRGBA());
                colors[0].r = 1.0f;
                colors[0].a = 1.0f;
                vizMarker.colors = colors;
                //std::cout << " " << e->type_name() << " " << e->name() << "=" << markers.size() << ":" << std::endl;
                for(OWL::Markers::const_iterator m = markers.begin(); m != markers.end(); m++)
                {
                  if(true) //(m->cond > 0)
                  {
                    //cout << "   " << m->id << ") pos=" << m->x << "," << m->y << "," << m->z << endl;
                    vizMarker.id = m->id;
                    std::vector<geometry_msgs::Point> p(1,geometry_msgs::Point());
                    p[0].x = 0.001*m->x;
                    p[0].y = -0.001*m->z; // PS swaps Y/Z
                    p[0].z = 0.001*m->y;
                    vizMarker.points = p;
                    vizMarkerArray.markers.push_back(vizMarker);
                  }
                }
                vizPub.publish(vizMarkerArray);
              }
              break;
            }
            
            case OWL::Type::RIGID:
            {
              if(e->get(rigids) > 0)
              {
                //std::cout << " " << e->type_name() << " " << e->name() << "=" << rigids.size() << ":" << std::endl;
                for(OWL::Rigids::const_iterator r = rigids.begin(); r != rigids.end(); r++)
                {
                  if(posePubs.count(r->id)>0 && !((r->pose[0] == 0) && (r->pose[1] == 0) && (r->pose[2] == 0) && (r->pose[3] == 1) && (r->pose[4] == 0) && (r->pose[5] == 0) && (r->pose[6] == 0))) //(r->cond > 0)
                  {
                    //std::cout << "   " << r->id << ") pose=" << 0.001*r->pose[0] << "," << 0.001*r->pose[1] << "," << 0.001*r->pose[2]
                    //<< " " << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
                    //<< std::endl;
                    
                    // Link transform
                    // PS swaps Y/Z
                    Eigen::Affine3d rbPose = Eigen::Translation3d(0.001*r->pose[0],-0.001*r->pose[2],0.001*r->pose[1]) * Eigen::Quaterniond(r->pose[3],r->pose[4],-1*r->pose[6],r->pose[5]);
                    Eigen::Affine3d linkPose = rbPose * id2linkTf.at(r->id);
                    Eigen::Vector3d linkT(linkPose.translation());
                    Eigen::Quaterniond linkQ(linkPose.rotation());

                    // Publish TF
                    tf::Transform poseTF;
                    poseTF.setOrigin(tf::Vector3(linkT(0), linkT(1), linkT(2))); // PS swaps Y/Z
                    poseTF.setRotation(tf::Quaternion(linkQ.x(), linkQ.y(), linkQ.z(), linkQ.w()));
                    tfbr.sendTransform(tf::StampedTransform(poseTF,timeNow,tf_frame,id2name.at(r->id)));

                    // Publish pose
                    geometry_msgs::PoseStamped poseMsg;
                    poseMsg.header.stamp = timeNow;
                    poseMsg.header.frame_id = tf_frame; // ROS standards specify this is parent frame name //id2name.at(r->id);;
                    poseMsg.pose.position.x = linkT(0);
                    poseMsg.pose.position.y = linkT(1); // PS swaps Y/Z
                    poseMsg.pose.position.z = linkT(2);
                    poseMsg.pose.orientation.w = linkQ.w();
                    poseMsg.pose.orientation.x = linkQ.x();
                    poseMsg.pose.orientation.y = linkQ.y(); // PS swaps Y/Z
                    poseMsg.pose.orientation.z = linkQ.z();
                    posePubs.at(r->id).publish(poseMsg);
                  }
                }
              }
            }
            break;
          } // switch
        }
        break;
      }
    }
    
    ros::spinOnce();
  }

  owl.done();
  owl.close();
  
  ros::waitForShutdown();
  return 0;
}
