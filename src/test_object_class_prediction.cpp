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

// to do: include header files for the ROS services and messages
#include "ros/ros.h"
// #include "strands_qsr_msgs/GetGroupEstimate.h"
// #include "strands_qsr_msgs/BBox.h"
// #include "strands_qsr_msgs/GroupEstimate.h"

#define DEBUG 0

/* 
Request and response are passed by reference:
 bool add(beginner_tutorials::AddTwoInts::Request  &req,
   beginner_tutorials::AddTwoInts::Response &res)
*/

using namespace std; 

int main(int argc, char *argv[]) {

  string dataFolderName = (argv[ 1 ]);
  cout << "The input folder name is: " << dataFolderName << endl; 
  string inputFolder =  "./" +  dataFolderName ;

  //*******************************************************************
  // Training using the real-world KTH dataset of 42 desktop scenes
  //*******************************************************************
  Training computeTraining(inputFolder);
  computeTraining.compute();
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
  v0.x = 0.165;
  v0.y = 1.292;
  v0.z = 0.004;
  pcl::PointXYZ v1;
  v1.x = 0.165;
  v1.y = 1.292;
  v1.z = 0.452; 
  pcl::PointXYZ v2;
  v2.x = 0.141;
  v2.y = 1.803;
  v2.z = 0.452;
  pcl::PointXYZ v3;
  v3.x = 0.141;
  v3.y = 1.803;
  v3.z = 0.004;
  pcl::PointXYZ v4;
  v4.x = 0.433;
  v4.y = 1.304;
  v4.z = 0.004;
  pcl::PointXYZ v5;
  v5.x = 0.433;
  v5.y = 1.304;
  v5.z = 0.452;
  pcl::PointXYZ v6;
  v6.x = 0.408;
  v6.y = 1.815;
  v6.z = 0.452;
  pcl::PointXYZ v7;
  v7.x = 0.408;
  v7.y = 1.815;
  v7.z = 0.004; 
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

/*
# unique object identifiers
string object_id
# object types
string[] type
# type weights
float32[] weight
// *************************************************************
def handle_group_estimate(req):
    res = GetGroupEstimateResponse()
    res.estimate = list()
    for obj in req.object_id:    
        est = GroupEstimate()
        est.object_id = obj
        est.type  = list()
        est.weight  = list()

        for t in req.type:
            est.type.append(t)
            est.weight.append(random.uniform(0,1))

        res.estimate.append(est)

    return res
// *************************************************************
*/
 /*
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

  */

  return 0;
}



