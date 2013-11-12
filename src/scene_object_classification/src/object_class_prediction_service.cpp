#include "Object.hpp"
#include "SceneInformation.hpp"
#include "ApiConvertKTHDB.hpp"
#include "ObjectFeatures.hpp"
#include "DatabaseInformation.hpp"
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include "TestScene.hpp"
#include "Training.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "ros/ros.h"
#include "strands_qsr_msgs/GetGroupEstimate.h"
#include "strands_qsr_msgs/BBox.h"
#include "strands_qsr_msgs/GroupEstimate.h"

#define DEBUG 0

/* 
Request and response are passed by reference:
 bool add(beginner_tutorials::AddTwoInts::Request  &req,
   beginner_tutorials::AddTwoInts::Response &res)
*/

using namespace std; 



bool handle_group_estimate(strands_qsr_msgs::GetGroupEstimate::Request  & req,
         strands_qsr_msgs::GetGroupEstimate::Response & res) {


  // string dataFolderName = (argv[ 1 ]);
  // cout << "The input folder name is: " << dataFolderName << endl; 
  string dataFolderName = "data_more_objects";
  string inputFolder =  "/home/marina/Project_Scene_Classification/project-scene-classification/src/scene_object_classification/data/data_more_objects" ;

  string fileSimulationDatabase = "/home/marina/Project_Scene_Classification/project-scene-classification/src/scene_object_classification/data/data_simulation/simulation/bham_office_desk_500_modifiedroot.json";
  cout << inputFolder << endl;

  //*******************************************************************
  // Training using the real-world KTH dataset of 42 desktop scenes
  //*******************************************************************
  Training computeTraining;
  // computeTraining.compute(inputFolder);    // training on the real world KTH database
  computeTraining.compute(fileSimulationDatabase, true);     // training on the simulated database of 500 scenes
  DatabaseInformation trainedDatabase = computeTraining.getDatabaseInformation(); 
  
  //*******************************************************************
  // Creating data like the request of ROS service
  // Input test scene: same data as the request fields of the ROS service. 
  //*******************************************************************
  // type
  const char *vinit[] = {"Monitor", "Keyboard", "Mouse",  "Mug", "Lapotp", "Lamp", "Pen/pencil"};
  vector<string> type(vinit, (vinit+ sizeof(vinit)/sizeof(vinit[0]))); 

  // object_id
  vector<string> object_id;
  object_id.push_back("obj1");
  object_id.push_back("obj2");

  // bbox
  vector<vector<pcl::PointXYZ> > bbox;
  vector<pcl::PointXYZ> boundingBoxObj1;
  vector<pcl::PointXYZ> boundingBoxObj2;
  pcl::PointXYZ v0;
  v0.x = 0.16537714004516602;
  v0.y = 0.7233791351318359;
  v0.z = 0.004999995231628418;
  pcl::PointXYZ v1;
  v1.x = 0.16537714004516602;
  v1.y = 0.723379135131835;
  v1.z = 0.45243966579437256; 
  pcl::PointXYZ v2;
  v2.x = 0.1617276668548584;
  v2.y = 1.2348713874816895;
  v2.z = 0.45243966579437256;
  pcl::PointXYZ v3;
  v3.x = 0.1617276668548584;
  v3.y =  1.2348713874816895;
  v3.z = 0.004999995231628418;
  pcl::PointXYZ v4;
  v4.x = 0.43334078788757324;
  v4.y = 0.7252907752990723;
  v4.z = 0.004999995231628418;
  pcl::PointXYZ v5;
  v5.x = 0.43334078788757324;
  v5.y = 0.7252907752990723;
  v5.z = 0.45243966579437256;
  pcl::PointXYZ v6;
  v6.x = 0.4296913146972656;
  v6.y = 1.2367830276489258;
  v6.z = 0.45243966579437256;
  pcl::PointXYZ v7;
  v7.x = 0.4296913146972656;
  v7.y = 1.2367830276489258;
  v7.z = 0.004999995231628418; 
  boundingBoxObj1.push_back(v0);
  boundingBoxObj1.push_back(v1);
  boundingBoxObj1.push_back(v2);
  boundingBoxObj1.push_back(v3);
  boundingBoxObj1.push_back(v4);
  boundingBoxObj1.push_back(v5);
  boundingBoxObj1.push_back(v6);
  boundingBoxObj1.push_back(v7);
  bbox.push_back(boundingBoxObj1);
 
  v0.x = 0.534;
  v0.y = 0.730;
  v0.z = 0.004;
  v1.x = 0.534;
  v1.y = 0.730;
  v1.z = 0.052; 
  v2.x = 0.497;
  v2.y = 1.287;
  v2.z = 0.052;
  v3.x = 0.497;
  v3.y = 1.287;
  v3.z = 0.004;
  v4.x = 0.780;
  v4.y = 0.746;
  v4.z = 0.004;
  v5.x = 0.780;
  v5.y = 0.746;
  v5.z = 0.052;
  v6.x = 0.743;
  v6.y = 1.303;
  v6.z = 0.052;
  v7.x = 0.743;
  v7.y = 1.303;
  v7.z = 0.004; 
  boundingBoxObj2.push_back(v0);
  boundingBoxObj2.push_back(v1);
  boundingBoxObj2.push_back(v2);
  boundingBoxObj2.push_back(v3);
  boundingBoxObj2.push_back(v4);
  boundingBoxObj2.push_back(v5);
  boundingBoxObj2.push_back(v6);
  boundingBoxObj2.push_back(v7);
  bbox.push_back(boundingBoxObj2);

  //*********************************************************
  // Test
  // ********************************************************
  TestScene unknownScene(trainedDatabase);
  unknownScene.loadAnnotationServiceFormat(type, object_id, bbox);
  unknownScene.extractFeatures();
 
 // Compute probabilities for all modeled object categories + classify objects. 
  vector<vector<double> > weights = unknownScene.predictObjectClasses();

  // ********************************************************
  // Creating the ROS service response with the computed fields
  // ********************************************************
 
  vector<strands_qsr_msgs::GroupEstimate> estimate;

  // for each object in the test scene
  for ( int i = 0; i < object_id.size(); i++ ) {

    strands_qsr_msgs::GroupEstimate currentObjectEstimate;
    currentObjectEstimate.object_id = object_id.at(i);

    ROS_INFO("Creating response for object_id: %s ", currentObjectEstimate.object_id.c_str());

    // for each of the modeled object cateogories the test object is tested against
    for ( int j = 0; j < type.size(); j++ ) {
      double currentWeight = weights[i][j];
      string currentType = type[j];

      currentObjectEstimate.weight.push_back(currentWeight); 
      currentObjectEstimate.type.push_back(currentType);

      ROS_INFO("The weight is: %f for object type: %s", currentWeight, currentType.c_str());
    }

    estimate.push_back(currentObjectEstimate);
  }  

  res.estimate = estimate; 

  return true;
}


int main(int argc, char **argv) {

  // the last parameter is a string containing the name of the ROS node
  ros::init(argc, argv, "relational_estimator_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("object_class_prediction", handle_group_estimate);
  ROS_INFO("Ready to estimate the test scene");
  ros::spin();
  return 0;
}

