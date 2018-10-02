
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "normal_surface_calc/targetPoints.h"

#include <iostream>
#include <cstdlib>
#include <fstream>
#include <exception>
//#include <conio.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <geometry_msgs/Point32.h>
#include "path_checker.h"
#include "string_convertor.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector< vector <Point2d> > vPtSignature;
ros::Publisher pubCommand;
bool exe_task = false;
string bbox_objects = "";
string grasp_obj = "";
//store the previous target points
std::vector<Point3d> position;
std::vector<Point3d> normals;
int data_size=0;
string splitTag="@@";

double differThreshold=0.05;

string sendback_cmd="";

int sendingFreq = 1000;

vector<float> obj_locations;  // < x1,y1,z1, h1, w1..., ,x2,y2,... >
vector<string> obj_names; // <cls1, cls2, cls3, ...>


string constructSinglePointStr(cv::Point3d pt)
{
  return string_convertor::d2s(pt.x)+","+string_convertor::d2s(pt.y)+","+string_convertor::d2s(pt.z);
}

string constructPubStr(vector< vector <Point3d> > ps, vector< vector <Point3d> > ns)
{
  string rtStr="";
  size_t strokesNum=ps.size();
  for(int i=0;i<strokesNum;i++)
  {
     size_t pointsNum=ps[i].size();
     if(pointsNum>0)
     {
       string thisLineStr="";
       for(int j=0;j<pointsNum-1;j++)
         thisLineStr += constructSinglePointStr(ps[i][j])+","+constructSinglePointStr(ns[i][j])+",";
       thisLineStr+=constructSinglePointStr(ps[i][pointsNum-1])+","+constructSinglePointStr(ns[i][pointsNum-1]);
       if(i==strokesNum-1)
         rtStr+=thisLineStr;
       else
         rtStr+=thisLineStr+";";
     }
  }
  return rtStr;
}

double sumPositionPoints()
{
  double rtd=0;
  size_t pointNumber=position.size();
  for(int i=0;i<pointNumber;i++)
    rtd +=position[i].x+position[i].y+position[i].z;
  return rtd;
}

bool checkNAN(const normal_surface_calc::targetPoints::ConstPtr& msg)
{
  bool rtb=false;
  for (int i = 0; i < msg->path_robot.size(); i++) {
    if(isnan(msg->path_robot[i].x)||isnan(msg->path_robot[i].y)||isnan(msg->path_robot[i].z))
        rtb=true;
    if(isnan(msg->normals_robot[i].x)||isnan(msg->normals_robot[i].y)||isnan(msg->normals_robot[i].z))
        rtb=true;
  }
  return rtb;
}

//will only publish the task command to robot side if exe_task tag is true
void path_dataCallback(const normal_surface_calc::targetPoints::ConstPtr& msg)
{
  try
  {
    if (vPtSignature.size()==0)
    {
      return;
    }
    //cout<<"path_dataCallback"<<endl;
    //cout<<"path_robot.size"<<msg->path_robot.size()<<endl;
    double currentPositionSum=sumPositionPoints();
    position.resize(msg->path_robot.size());
    normals.resize(msg->path_robot.size());
    sendback_cmd="drawing:1:";
    // if(bbox_objects == "") //if no object detection  message received, do not send commands to robot side.
    //   return ;
    // else

      //sendback_cmd += grasp_obj + splitTag; // add the name of target from the interface for grasping
      //sendback_cmd += bbox_objects + splitTag;//combine object detection results

    path_checker pcheck(msg, vPtSignature);
    position=pcheck.getPathPositions();
    normals=pcheck.getNormalVects();
    vector< vector <Point3d> > ps=pcheck.getStrokesPathPositions();
    vector< vector <Point3d> > ns=pcheck.getStrokesNormalVects();
    data_size=position.size();
    if(data_size>0)
    {
      sendback_cmd +=constructPubStr(ps, ns);
      //std::cout<<"targetPoints Topic received! no NAN."<<endl;
    }

    //sendback_cmd +=endTag;
    double newPositionSum=sumPositionPoints();
    if(abs(currentPositionSum-newPositionSum)>differThreshold)
    {
      std::cout<<"new path drawing task detected!"<<endl;
      if(!exe_task)
        std::cout<<"holding until execution command published!"<<endl;
   }
 }
 catch(exception &e) {
   std::cout << "Catch an exception: " << e.what() << std::endl;
 }
}


void target_bbox_callback(const std_msgs::String::ConstPtr& msg)
{
    bbox_objects = msg->data;
    //cout << "received target bbox" << endl;
    vector<string> tokens = string_convertor::split(bbox_objects, ':');
    obj_locations.clear();
    obj_names.clear();

    if (tokens[0] == "target_bbox") {
      // parse the string
      int num_box = atoi(tokens[1].c_str()); // number of boxes
      string locs = tokens[2]; // name and info of all boxes, sponge, ....
      tokens.clear();
      tokens = string_convertor::split(locs, ',');
      for (int i = 0; i < num_box; i++) {
          obj_names.push_back(tokens[6*i]);
          obj_locations.push_back(atof(tokens[6*i+1].c_str()));
          obj_locations.push_back(atof(tokens[6*i+2].c_str()));
          obj_locations.push_back(atof(tokens[6*i+3].c_str()));
          obj_locations.push_back(atof(tokens[6*i+4].c_str()));
          obj_locations.push_back(atof(tokens[6*i+5].c_str()));
      }
    }
}


void grasp_obj_callback(const std_msgs::String::ConstPtr& msg)
{
    string data = string(msg->data);
    // either grasp:objname, or execute
    vector<string> tokens = string_convertor::split(data, ':');
    cout << "received sth in the grasp obj cb func:" << data << endl;
    if(tokens[0] == "execute") // execute will be published by /command_init from interface when clicking execute
    {
      // execute:1:path
       cout << "received execution task" << endl;
       exe_task = true;
       std_msgs::String msg_pub;
       msg_pub.data = sendback_cmd;
       pubCommand.publish(msg_pub);//send the command to robot side by simply publishing the topic /rr/commands
       std::cout << "new path drawing task published! " << sendback_cmd << endl;
       exe_task = false;
       sendback_cmd="";
       // drawing:1:strokes under /rr/commands
    }
    else if (tokens[0] == "grasp") {
      cout << "received grasping task" << endl;
      grasp_obj = tokens[1];
      vector<float> obj_loc;
      // find the index for the obj
      int obj_idx = 0; // index of the selected object
      bool no_match = true; // whether the obj is in the current list
      for (int i = 0 ; i < obj_names.size(); i++){
          if (grasp_obj == obj_names[i]) {
            obj_idx = i;
            no_match = false;
            break;
          }
      }
      if (!no_match) {
          // store the obj locations, x,y,z,h,w
          for (int i = 0 ; i < 5; i++){
            obj_loc.push_back(obj_locations[5*obj_idx + i]);
          }

          cout << "the object locations are: ";
          for (int i = 0 ; i < 5; i++){
            cout << obj_loc[i] <<  ",";
          }
          cout << endl;

          //publish the grasp command plus the obj name and location
          //grasp:1:sponge@@x,y,z,h,w
          stringstream msg_str;
          string separator = "";
          msg_str << "grasp:1:" << grasp_obj << splitTag;
          for (int i = 0 ; i < 5; i++){
            msg_str << separator << obj_loc[i];
            separator = ",";
          }
          std_msgs::String msg;
          msg.data = msg_str.str();
          pubCommand.publish(msg);//send the command to robot side by simply publishing the topic /rr/commands
          std::cout << "new grasping task published!:" << msg_str.str() << endl;
          // clear the vector after publishing
          //obj_locations.clear();
      }
      else
          cout << "The selected object is not there anymore" << endl;
    }
    else
      exe_task = false;
}


void signature_data_callback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data=="reset")
    {
       vPtSignature.clear();
       return ;
    }
    vector<string> strokeStrs=string_convertor::split(msg->data, ';');//get different strokes.
    size_t strokesNum=strokeStrs.size();
    if(strokesNum>0)
    {
        vPtSignature.clear();
        vector<vector<Point2d> >().swap(vPtSignature);
        for(int i=0;i<strokesNum;i++)
        {
          string thisStroke=strokeStrs[i];
          vector<double> points=string_convertor::fromString2Array(thisStroke);
          size_t pointNum=points.size()/2;
          vector<Point2d> strokePoints;
          for(int j=0;j<pointNum;j++)
              strokePoints.push_back(Point2d(points[2*j],points[2*j+1]));
          vPtSignature.push_back(strokePoints);
        }
        //published=false;
    }
}




int main(int argc, char **argv )
{
    //initialize the ROS system and become a node.
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    pubCommand = nh.advertise<std_msgs::String>("/rr/commands", 1, true);
    ros::Subscriber sub = nh.subscribe("/targetPoints", 1, path_dataCallback);
    ros::Subscriber sub1 = nh.subscribe("/chris/strokes", 1000, signature_data_callback);
    ros::Subscriber sub2 = nh.subscribe("/target_bbox", 1000, target_bbox_callback);
    //ros::Subscriber sub3 = nh.subscribe("/execute", 1000, exe_task_callback);
    ros::Subscriber sub3 = nh.subscribe("/command_init", 1000, grasp_obj_callback);

    ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::spin();
    ros::shutdown();
    return 0;
}
