#ifndef OBJECT_H
#define OBJECT_H


#include <string>
#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

// The object geometry amd information 
class Object{

private:
  string objectName;
  pcl::PointCloud<pcl::PointXYZ> boundingBox;
  pcl::PointXYZ centroid;

/*
  This function computes the centroid of the bouding box given the 8 vertices 
  of the bounging box and sets the private member: "centroid".
  Computes the 3D centroid coordinates: x, y, z, by computing the mean of the corresponding 
  coordinate values of all the 8 vertices of the bounding box cuboid.
*/
  void setCentroid();
  int actualObjectID;
  int predictedObjectID;


public:
  Object();
  void setObjectName(string);
  int getActualObjectID() { return actualObjectID; }
  int getPredictedObjectID() { return predictedObjectID; }
  void setPredictedObjectID(int i) { predictedObjectID = i; } 

/*
  This function sets the "boundingBox" data member given in input a 
  vector of PCL points corresponding to the 8 vertices.
  The 8 vertices are pushed back in the point cloud that represents
  the bounding box.
*/
  void setBoundingBox(vector<pcl::PointXYZ>);   
  string getObjectName();
  pcl::PointCloud<pcl::PointXYZ> getBoundingBox();  
  pcl::PointXYZ getCentroid();
  void setActualObjectID(int i) { actualObjectID = i; } 

};


#endif

