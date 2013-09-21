#include "Object.hpp"
  
#define DEBUG 0

Object::Object() {
  objectName = "";
}

void Object::setObjectName(string inputName) {
  objectName = inputName;
} 

void Object::setBoundingBox(vector<pcl::PointXYZ> inputBoundingBox)   {
  for (int i = 0; i < inputBoundingBox.size(); i++) {
  boundingBox.push_back(inputBoundingBox[i]);
  }
  Object::setCentroid();
}


void Object::setCentroid() { 
  float x_mean = (boundingBox.points[0].x + boundingBox.points[1].x + boundingBox.points[2].x + boundingBox.points[3].x + boundingBox.points[4].x + boundingBox.points[5].x + boundingBox.points[6].x + boundingBox.points[7].x) / 8; 
  float y_mean = (boundingBox.points[0].y + boundingBox.points[1].y + boundingBox.points[2].y + boundingBox.points[3].y + boundingBox.points[4].y + boundingBox.points[5].y + boundingBox.points[6].y + boundingBox.points[7].y) / 8;
  float z_mean = (boundingBox.points[0].z + boundingBox.points[1].z + boundingBox.points[2].z + boundingBox.points[3].z + boundingBox.points[4].z + boundingBox.points[5].z + boundingBox.points[6].z + boundingBox.points[7].z) / 8;
  centroid.x = x_mean;
  centroid.y = y_mean;
  centroid.z = z_mean;
}

string Object::getObjectName() {
  return objectName;
}

pcl::PointCloud<pcl::PointXYZ> Object::getBoundingBox(){
  if (DEBUG) {
    for (int i = 0; i < boundingBox.size(); i++ ) {
      cout << "Vertix "<< i << endl;
      cout << "x: " << boundingBox.points[i].x << endl ;
      cout << "y: " << boundingBox.points[i].y << endl ;
      cout << "z: " << boundingBox.points[i].z << endl ;
    }
  }
  return boundingBox;
}

pcl::PointXYZ Object::getCentroid() {
 
  if (DEBUG) {
    cout << "Centroid: "<< endl;
    cout << "x: " << centroid.x << endl ;
    cout << "y: " << centroid.y << endl ;
    cout << "z: " << centroid.z << endl ;
  }
  return centroid;
}



