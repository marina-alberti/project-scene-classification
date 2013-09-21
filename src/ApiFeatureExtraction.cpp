#include "ApiFeatureExtraction.hpp"

#define DEBUG 0

ApiFeatureExtraction::ApiFeatureExtraction() {
}


void ApiFeatureExtraction::extractFeaturesSingleObjects( SceneInformation & inputScene) {

  vector<Object> objectList = inputScene.getObjectList();
  pcl::PointXYZ deskCentroid = inputScene.getDeskCentroid();
  /*
  float deskLength = inputScene.getDeskLength();
  float deskWidth = inputScene.getDeskWidth();
  */

  for(vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
    
   Object currentObject = *it;
   if (DEBUG) {
      cout << endl << "Adding the features of a new object: "  << currentObject.getObjectName() << endl; 
    }
     
    ObjectFeatures singleObjectFeatureExtraction(currentObject, deskCentroid);  
    singleObjectFeatureExtraction.extractFeatures();  
    vector<FeatureInformation> singleObjectFeatures = singleObjectFeatureExtraction.getAllFeatures();
    if (DEBUG) { 
      cout << "The size of the features of the current object is : " 
      << singleObjectFeatures.size() << endl;
    }
    
    for(vector<FeatureInformation>::iterator it2 = singleObjectFeatures.begin(); it2 != singleObjectFeatures.end(); ++it2) {
      inputScene.addFeatureSingleObject(*it2);
      if (DEBUG) {
        // cout << "The added feature is : " << (*it2) << endl; 
      }
    }
    vector <FeatureInformation> allFeats = inputScene.getFeatureListSingleObject();
    if (DEBUG) { cout << "Total N Of features : " << allFeats.size() << endl;   } // ok 
  }
}


/*
For each object (i) in the scene:
 - Iterates over all the other objects and for each other object: 
     -  takes this object (j)
     - computes feature for object pair (i, j)
     - adds the features to the vector<features> of the scene "inputScene".
*/
void ApiFeatureExtraction::extractFeaturesPairObjects(SceneInformation & inputScene) {

  vector<Object> objectList = inputScene.getObjectList();
  for(vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
    Object referenceObject = *it;

    if (DEBUG) {
      cout << endl << "Adding the features of a new REFERENCE object. " << endl; 
    }
 

    for(vector<Object>::iterator it2 = objectList.begin(); it2 != objectList.end(); ++it2) {
      if (it2 != it) {
        Object targetObject = *it2;

        if (DEBUG) {
        cout << endl << "Adding the features of a new TARGET object. Pair is : " << referenceObject.getObjectName() << ",  " << targetObject.getObjectName() << endl; 
        }

        ObjectPairFeatures pairObjectFeatureExtraction(referenceObject, targetObject);
        pairObjectFeatureExtraction.extractFeatures();
        vector<FeatureInformation> _features = pairObjectFeatureExtraction.getAllFeatures();

        if (DEBUG) { 
          cout << "The size of the features of the current object pair is : " 
               << _features.size() << endl;
        }

        for(vector<FeatureInformation>::iterator it3 = _features.begin(); it3 != _features.end(); ++it3) {
          inputScene.addFeaturePairObject((*it3));
          if (DEBUG) {
           // cout << "The added feature is : " << (*it3) << endl; 
          }
        }
      }
    }
 
  }
}






