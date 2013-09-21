#include "ObjectFeatures.hpp"

#define DEBUG 0
#define PI 3.14159265

ObjectFeatures::ObjectFeatures( Object & inputObject, pcl::PointXYZ inputDeskCentroid ) {
  internalObject = inputObject;
  deskCentroid = inputDeskCentroid;
/*
  angle2dCentroid = 0;
  angle2d = 0;
  angle3d = 0;
  volumeSize = 0;
  sizeProjectedX = 0;
  sizeProjectedY = 0;
  sizeProjectedZ = 0;
*/
 
}

void ObjectFeatures::computePose() {
  pcl::PointXYZ objectPose = internalObject.getCentroid();
  pose.addValue(objectPose.x);
  pose.addValue(objectPose.y);
  pose.addValue(objectPose.z);
}


void ObjectFeatures::computeAngle3d () {
  // Eigen::Vector4f VectorObjectPose4f(pose.x, pose.y, 0, 0);
  pcl::PointXYZ objectCentroid = internalObject.getCentroid();
  Eigen::Vector4f v0;
  v0[0] = objectCentroid.x;
  v0[1] = objectCentroid.y;
  v0[2] = objectCentroid.z;
  v0[3] = 1; // ?
  Eigen::Vector4f v1(1, 0, 0, 1);  // check ?
  float _angle = pcl::getAngle3D (v0, v1);
  angle3d.addValue( _angle );

  if (DEBUG) {
    cout << "The angle 3D is : " << _angle 
          << " = in degrees " << (_angle * 180 / 3.14) << endl;
    cout << "  when object centroid x = " 
         << (internalObject.getCentroid()).x << endl;   
    cout << "  and object centroid y = " 
         << (internalObject.getCentroid()).y << endl;  
    cout << "  and object centroid z = " 
         << (internalObject.getCentroid()).z << endl;  
  }
  
}


void ObjectFeatures::computeAngle2dCentroid() {

/*
  float dx_startModel = startModel_x-x_sensor;
  float dx_vertix = x_vertix-x_sensor;
  float dy_startModel = startModel_y-y_sensor;
  float dy_vertix = y_vertix-y_sensor;
  float l_startModel = sqrt( dx_startModel*dx_startModel + dy_startModel*dy_startModel );
  float l_vertix = sqrt( dx_vertix*dx_vertix + dy_vertix*dy_vertix );
  float slope_startModel = (startModel_y-y_sensor)/(startModel_x-x_sensor);
  float slope_vertix = (y_vertix-y_sensor)/(x_vertix-x_sensor);
  // compute the angles with respect to x axis
  theta_startModel = atan(slope_startModel) ;   
  float theta_vertix = atan(slope_vertix) ;  
  float angle = abs(theta_vertix - theta_startModel);  
*/
 
  float dx_object_desk = (internalObject.getCentroid()).x - deskCentroid.x;
  float dy_object_desk = (internalObject.getCentroid()).y - deskCentroid.y;
 //float l_object_desk = sqrt( dx_object_desk * dx_object_desk + dy_object_desk * dy_object_desk );
  float slope_object_desk = ( dy_object_desk ) / ( dx_object_desk );
  float angle_object_desk = atan2 (dy_object_desk, dx_object_desk);

  if ( angle_object_desk < 0 ) {
    angle_object_desk = angle_object_desk + 2 * PI;  
  }
    
 // float angle2 = (slope_startModel - slope_vertix) / (1.0 + (slope_startModel * slope_vertix)); 

  if (DEBUG) {
    cout << "The angle between desk centroid and object centroid is : " << angle_object_desk 
          << " = in degrees " << (angle_object_desk * 180 / 3.14) << endl;
    cout << " when desk centroid x = " << deskCentroid.x << " and object centroid x = " 
         << (internalObject.getCentroid()).x << endl;   
    cout << " when desk centroid y = " << deskCentroid.y << " and object centroid y = " 
         << (internalObject.getCentroid()).y << endl;   
  }

  angle2dCentroid.addValue(angle_object_desk);
}



void ObjectFeatures::computeAngle2d() {

  float dx_object_desk = (internalObject.getCentroid()).x - 0.0;
  float dy_object_desk = (internalObject.getCentroid()).y - 0.0;
 //float l_object_desk = sqrt( dx_object_desk * dx_object_desk + dy_object_desk * dy_object_desk );
  float slope_object_desk = ( dy_object_desk ) / ( dx_object_desk );
  float angle_object_desk = atan2 (dy_object_desk, dx_object_desk);
  if ( angle_object_desk < 0 ) {
    angle_object_desk = angle_object_desk + 2 * PI;  
  }

  if (DEBUG) {
    cout << "The angle between desk CORNER and object centroid is : " << angle_object_desk 
          << " = in degrees " << (angle_object_desk * 180 / 3.14) << endl;
    cout << " when object centroid x = " 
         << (internalObject.getCentroid()).x << endl;   
    cout << " when object centroid y = " 
         << (internalObject.getCentroid()).y << endl;   
  }
  angle2d.addValue(angle_object_desk);
}


void ObjectFeatures::computeVolumeSize() {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = internalObject.getBoundingBox();
   
  float dim1 = pcl::euclideanDistance(internalBoundingBox.points[1], internalBoundingBox.points[0]); 
  float dim2 = pcl::euclideanDistance(internalBoundingBox.points[3], internalBoundingBox.points[0]); 
  float dim3 = pcl::euclideanDistance(internalBoundingBox.points[4], internalBoundingBox.points[0]); 

  float _volumeSize = (dim1 * dim2 * dim3);
  if (DEBUG) { 
    cout << "The volume size of current object is: " << _volumeSize << endl;
  }

  volumeSize.addValue(_volumeSize);
}


/*
 This function computes the size of bounding box project into the X axis
*/
void  ObjectFeatures::computeSizeProjectedX() {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = internalObject.getBoundingBox();
  vector<float> verticesProjectedX;
  for (int i = 0; i < 8; i++) {
    verticesProjectedX.push_back(internalBoundingBox.points[i].x) ;

  }
  std::sort (verticesProjectedX.begin(), verticesProjectedX.end());
  float minX = verticesProjectedX[0];
  float maxX = verticesProjectedX[7];
  float _size = abs ( maxX - minX );
  sizeProjectedX.addValue( _size );

  if (DEBUG) { 
    cout << "The volume size of current object projected onto X is: " << _size << endl;
    cout << "Difference of " << maxX << " and " << minX << endl; 
  }
}


void  ObjectFeatures::computeSizeProjectedY() {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = internalObject.getBoundingBox();
  vector<float> verticesProjectedY;
  for (int i = 0; i < 8; i++) {
    verticesProjectedY.push_back(internalBoundingBox.points[i].y) ;

  }
  std::sort (verticesProjectedY.begin(), verticesProjectedY.end());
  float minY = verticesProjectedY[0];
  float maxY = verticesProjectedY[7];
  float _size = abs ( maxY - minY );
  sizeProjectedY.addValue( _size);

  if (DEBUG) { 
    cout << "The volume size of current object projected onto y is: " << _size << endl;
  }
}


void  ObjectFeatures::computeSizeProjectedZ() {
  pcl::PointCloud<pcl::PointXYZ> internalBoundingBox = internalObject.getBoundingBox();
  vector<float> verticesProjectedZ;
  for (int i = 0; i < 8; i++) {
    verticesProjectedZ.push_back(internalBoundingBox.points[i].z) ;

  }
  std::sort (verticesProjectedZ.begin(), verticesProjectedZ.end());
  float minZ = verticesProjectedZ[0];
  float maxZ = verticesProjectedZ[7];
  float _size = abs ( maxZ - minZ );
  sizeProjectedZ.addValue(_size );

  if (DEBUG) { 
    cout << "The volume size of current object projected onto z is: " << _size << endl;
  }
}



void ObjectFeatures::extractFeatures() {

  computePose();
  allFeatures.push_back(pose);
 
  computeAngle2dCentroid();
  allFeatures.push_back(angle2dCentroid);
  computeAngle2d();
  allFeatures.push_back(angle2d);
  /*
  computeAngle3d();
  allFeatures.push_back(angle3d);
  */
  computeVolumeSize();
  allFeatures.push_back(volumeSize);
  computeSizeProjectedX();
  allFeatures.push_back(sizeProjectedX);
  computeSizeProjectedY();
  allFeatures.push_back(sizeProjectedY);
  computeSizeProjectedZ();
  allFeatures.push_back(sizeProjectedZ);

}




