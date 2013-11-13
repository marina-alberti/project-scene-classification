#include "ApiConvertSimulationDB.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <iostream>

#define DEBUG 0
#define TESTFLAG 1

using namespace boost::property_tree;


static const std::string TAG_SCENARIO = "scenario";
static const std::string TAG_NAME = "name";
static const std::string TAG_OBJECT = "object";
static const std::string TAG_NOBJECTS = "numberOfObjects";
static const std::string TAG_ALLOBJECTS = "allObjects";
static const std::string TAG_POSE = "pose";
static const std::string TAG_DIMENSIONS = "dimensions";
static const std::string TAG_LENGTH = "length";
static const std::string TAG_WIDTH = "width";
static const std::string TAG_HEIGHT = "height";
static const std::string TAG_COLOR = "color";
static const std::string TAG_INDICES = "indices";

ApiConvertSimulationDB::ApiConvertSimulationDB(string inputFileName) {
  fileNameJSON = inputFileName;
}


/* 
This function parses the whole JSON file and iteratively parses the scenes in the file.
*/
void ApiConvertSimulationDB::parseFileJSON(vector<SceneInformation> & sceneInformationList) {
  
  int countScene = 0;
  float  _deskLength = 2;
  float  _deskWidth = 1.2;

  boost::property_tree::ptree doc;
  boost::property_tree::read_json(fileNameJSON, doc);

  // for each scene in the JSON file 
  BOOST_FOREACH(boost::property_tree::ptree::value_type& sceneRoot, doc.get_child("root")) {

    SceneInformation currentScene;
    currentScene.setDeskLength(_deskLength);
    currentScene.setDeskWidth(_deskWidth);
    currentScene.setDeskCentroid();
    if (DEBUG) {
      cout << "Parsing a new scene:" << endl;
    }
    parseSceneJSON(sceneRoot, currentScene);
    sceneInformationList.push_back(currentScene);
    countScene++;
  }

}


void ApiConvertSimulationDB::parseSceneJSON(boost::property_tree::ptree::value_type & sceneRoot, SceneInformation & currentScene){

  // the scene will contain a list of objects;
  vector<Object> objectVector;

  BOOST_FOREACH (boost::property_tree::ptree::value_type& sceneField, sceneRoot.second) {

    // "type" field: contains the object category of each object and the "name" of each object
    if (strcmp(sceneField.first.c_str(), "type") == 0) {

      // for each of the objects in the scene
      BOOST_FOREACH (boost::property_tree::ptree::value_type& objectList, sceneField.second) {
        if (DEBUG) {
          cout << "A new object in the test scene: " << objectList.first << endl;
          cout << "And the object category type is: " << objectList.second.get_value<std::string>() << endl;
        }
        Object newObject;

        newObject.setInstanceName(objectList.first);
        newObject.setObjectName(objectList.second.get_value<std::string>());
        newObject.setCategoryName(objectList.second.get_value<std::string>());

        objectVector.push_back(newObject);
        cout << newObject.getObjectName() << endl;
      }
    }

   // here setting the bounding box of each object
   if (strcmp(sceneField.first.c_str(), "bbox") == 0) {

      int countObject = 0;

      // for each of the objects in the scene (same order?)
      BOOST_FOREACH (boost::property_tree::ptree::value_type& objectList, sceneField.second) {

        vector<pcl::PointXYZ> boundingBoxVertices;

        // for each 3D point = vertix of the bounding box
        BOOST_FOREACH (boost::property_tree::ptree::value_type& bboxPoint, objectList.second) {

          pcl::PointXYZ boundingBoxVertix;

          // loop over the x y z coordinates of the current vertix
          int countCoord = 0;
          BOOST_FOREACH (boost::property_tree::ptree::value_type& bboxPointCoordinate, bboxPoint.second) {  
            if (countCoord == 0) {
              boundingBoxVertix.x = bboxPointCoordinate.second.get_value<double>();
            }
            else if (countCoord == 1) {
              boundingBoxVertix.y = bboxPointCoordinate.second.get_value<double>();
            }
            else if (countCoord == 2) {
              boundingBoxVertix.z = bboxPointCoordinate.second.get_value<double>();
            }
            countCoord++;
          }
          boundingBoxVertices.push_back(boundingBoxVertix);
          
        }
        (objectVector[countObject]).setBoundingBox(boundingBoxVertices);
        currentScene.addObject(objectVector[countObject]);
        countObject++;
      }
    }

  }
}







