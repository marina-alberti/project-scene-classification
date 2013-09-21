#include "ObjectPairFeatures.hpp"

#define DEBUG 0
#define PI 3.14159265


ObjectPairFeatures::ObjectPairFeatures( Object & inputObject1, Object & inputObject2 ) {
  referenceObject = inputObject1;
  targetObject = inputObject2;
/*
  EuclideanDistance = 0;
  EuclideanDistance2d = 0;
  orientation2d = 0;
  orientation3d = 0;
  minimumDistanceBoudaries = 0;
 */
}


void ObjectPairFeatures::computeEuclideanDistance() {

  pcl::PointXYZ centroidReferenceObject = referenceObject.getCentroid();
  pcl::PointXYZ centroidTargetObject = targetObject.getCentroid();
  float _EuclideanDistance = pcl::euclideanDistance( centroidReferenceObject, centroidTargetObject); 
  EuclideanDistance.addValue(_EuclideanDistance);

  if (DEBUG) {
    cout << "The Euclidean distance 3d between the 2 objects is : " << _EuclideanDistance 
         << endl;
    cout << "   when REFobject centroid x = " 
         << (referenceObject.getCentroid()).x << " and  y = " 
         << (referenceObject.getCentroid()).y << endl;   
      cout << " and TARGETobject centroid x = " 
         << (targetObject.getCentroid()).x << " and y = " 
         << (targetObject.getCentroid()).y << endl; 
  }

}


void ObjectPairFeatures::computeEuclideanDistance2d() {

  pcl::PointXYZ centroidReferenceObject = referenceObject.getCentroid();
  centroidReferenceObject.z = 0;
  pcl::PointXYZ centroidTargetObject = targetObject.getCentroid();
  centroidTargetObject.z = 0;
  float _EuclideanDistance2d = pcl::euclideanDistance( centroidReferenceObject, centroidTargetObject); 
  EuclideanDistance2d.addValue( _EuclideanDistance2d );

  if (DEBUG) {
    cout << "The Euclidean distance 2d between the 2 objects is : " << _EuclideanDistance2d 
         << endl;
    cout << "   when REFobject centroid x = " 
         << (referenceObject.getCentroid()).x << " and  y = " 
         << (referenceObject.getCentroid()).y << endl;   
      cout << " and TARGETobject centroid x = " 
         << (targetObject.getCentroid()).x << " and y = " 
         << (targetObject.getCentroid()).y << endl; 
  }
}


void ObjectPairFeatures::computeOrientation2d() {

  float dx = (targetObject.getCentroid()).x - (referenceObject.getCentroid()).x;
  float dy = (targetObject.getCentroid()).y - (referenceObject.getCentroid()).y;
  float slope = ( dy ) / ( dx );
  float _angle = atan2 (dy, dx);
  // float dx_horizontal = 

  if ( _angle < 0 ) {
    _angle = _angle + 2 * PI;  
  }
  orientation2d.addValue(_angle);

  if (DEBUG) {
    
    cout << "The angle between the 2 objects is : " << _angle 
          << " = in degrees " << ( _angle  * 180 / 3.14) << endl;
    cout << "   when REFobject centroid x = " 
         << (referenceObject.getCentroid()).x << " and  y = " 
         << (referenceObject.getCentroid()).y << endl;   
      cout << " and TARGETobject centroid x = " 
         << (targetObject.getCentroid()).x << " and y = " 
         << (targetObject.getCentroid()).y << endl; 
  }

}

void ObjectPairFeatures::computeSizeDifference() {

  pcl::PointCloud<pcl::PointXYZ> targetBoundingBox = targetObject.getBoundingBox();
  float dim1 = pcl::euclideanDistance(targetBoundingBox.points[1], targetBoundingBox.points[0]); 
  float dim2 = pcl::euclideanDistance(targetBoundingBox.points[3], targetBoundingBox.points[0]); 
  float dim3 = pcl::euclideanDistance(targetBoundingBox.points[4], targetBoundingBox.points[0]); 
  float sizeTarget = (dim1 * dim2 * dim3);

  pcl::PointCloud<pcl::PointXYZ> referenceBoundingBox = referenceObject.getBoundingBox();
  float dim1r = pcl::euclideanDistance(referenceBoundingBox.points[1], referenceBoundingBox.points[0]); 
  float dim2r = pcl::euclideanDistance(referenceBoundingBox.points[3], referenceBoundingBox.points[0]); 
  float dim3r = pcl::euclideanDistance(referenceBoundingBox.points[4], referenceBoundingBox.points[0]); 
  float sizeReference = (dim1r * dim2r * dim3r);

  float diff = sizeTarget / sizeReference;   
  sizeDifference.addValue(diff);

  if (DEBUG)  {
    cout << "The difference is SIZE between the 2 objects is : " << diff << endl;
  }
}

void ObjectPairFeatures::computeVerticalHeightDifference() {
  float dz = (targetObject.getCentroid()).z - (referenceObject.getCentroid()).z;
  verticalHeightDifference.addValue(dz);

   if (DEBUG)  {
    cout << "The difference is Vertical Height between the 2 objects is : " << dz << endl;
  }
}

void ObjectPairFeatures::extractFeatures() {

  computeEuclideanDistance();
  allFeatures.push_back(EuclideanDistance);
  computeEuclideanDistance2d();
  allFeatures.push_back(EuclideanDistance2d);
  computeOrientation2d();
  allFeatures.push_back(orientation2d);
  computeSizeDifference();
  allFeatures.push_back(sizeDifference);
  computeVerticalHeightDifference();
  allFeatures.push_back(verticalHeightDifference);
  
}


