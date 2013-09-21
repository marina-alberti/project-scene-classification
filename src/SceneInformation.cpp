
#include "SceneInformation.hpp"
#include "Object.hpp"
#include <fstream>
#include <iostream>
#include <ios>
#include <sstream>
#include <boost/property_tree/xml_parser.hpp>

#define DEBUG 0

using namespace boost::property_tree;

SceneInformation::SceneInformation () {
  sceneType = "";
  numberOfObjects = 0;
  deskLength = 0;
  deskWidth = 0;
  deskCentroid.x = 0;
  deskCentroid.y = 0;
  deskCentroid.z = 0;
}

void SceneInformation::setDeskLength(float inputLength) {
  deskLength = inputLength;
}

float SceneInformation::getDeskLength() {
  return deskLength;
}

void SceneInformation::setDeskWidth(float inputWidth) {
  deskWidth = inputWidth;
}

float SceneInformation::getDeskWidth() {
  return deskWidth;
}

void SceneInformation::setType(string inputString) {
  sceneType = inputString;
}

string SceneInformation::getType() {
  return sceneType;
}

void SceneInformation::setDeskCentroid() { 
  deskCentroid.x = deskLength / 2;
  deskCentroid.y = deskWidth / 2;
  deskCentroid.z = 0;
}

pcl::PointXYZ SceneInformation::getDeskCentroid() {
  return deskCentroid;
}

void SceneInformation::addObject(Object& currentObject) {
  objectList.push_back(currentObject);
}

vector<Object> SceneInformation::getObjectList(){
  return objectList;
} 

void SceneInformation::showSceneInformation() {
  cout << "This scene is of type: " <<  getType() << endl
       << "Internal data structure (IDS): " << endl;
  for (int i = 0; i < objectList.size(); i++) {
    Object currentObject = objectList[i];
    if (DEBUG) {  cout << endl << currentObject.getObjectName() << endl;  }
    currentObject.getBoundingBox();
    currentObject.getCentroid();
  }
}

/*
create a new vector<Object> . 
Iterates over the internal objectList.
When finds the objectName == one of the 3 
considered objects, then push_back that object 
in specific positino in the new vector.
Then replaces the old vector with the new one
*/
// to do : check what happens if the 3 objects are not present. size = 3.
// I shuold skip the current scene (?)
void SceneInformation::orderObjectList() {
  vector<Object> newObjectList(3);
  bool checkMonitor = 0;
  bool checkKeyboard = 0;
  bool checkMouse = 0;

  for(vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it)
  {
    string currentName = (*it).getObjectName();
    cout << currentName << endl;
    const char * currentNameChar = currentName.c_str();
    if (strcmp(currentNameChar, "Monitor") == 0 || strcmp(currentNameChar, "monitor") == 0 || strcmp(currentNameChar, "Screen") == 0 ) {
       if (DEBUG) { cout << "The monitor is found." << endl; }
       newObjectList[0] = *it;
       checkMonitor = 1;
    }
    if (strcmp(currentNameChar, "Keyboard") == 0 || strcmp(currentNameChar, "keyboard") == 0) {
       if (DEBUG) { cout << "The Keyboard is found." << endl; }
       newObjectList[1] = *it;
       checkKeyboard = 1;
    }
    if (strcmp(currentNameChar, "Mouse") == 0 || strcmp(currentNameChar, "mouse") == 0) {
       if (DEBUG) { cout << "The Mouse is found." << endl; }
       newObjectList[2] = *it;
       checkMouse = 1;
    }
    
  }
  if (checkMonitor == 0 || checkKeyboard == 0 || checkMouse == 0) {

    cout << endl << "ERROR: the objects have not been found." << endl;
    exit(1);
  }
  
  objectList = newObjectList;
  cout << objectList.size() << endl;
}

void SceneInformation::addFeatureSingleObject(FeatureInformation currentFeature) {
  featureListSingleObject.push_back(currentFeature);
}

void SceneInformation::addFeaturePairObject(FeatureInformation currentFeature) {
  featureListPairObject.push_back(currentFeature);
}



