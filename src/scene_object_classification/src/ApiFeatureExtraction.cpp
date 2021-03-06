#include "ApiFeatureExtraction.hpp"

#define DEBUG 0
#define TESTFLAG 1

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
   if (TESTFLAG) {
      cout << endl << "Adding the features of a new object: "  << currentObject.getObjectName() << endl; 
    }
     
    ObjectFeatures singleObjectFeatureExtraction(currentObject, deskCentroid);  
    singleObjectFeatureExtraction.extractFeatures();  
    vector<FeatureInformation> singleObjectFeatures = singleObjectFeatureExtraction.getAllFeatures();
    if (DEBUG) { 
      cout << "The size of the features of the current object is : " 
      << singleObjectFeatures.size() << endl;
    }
    
    int objectID =  (*it).getActualObjectID();
    inputScene.addAllFeatSingleObject(singleObjectFeatures, objectID) ; // new
    
    //vector <FeatureInformation> allFeats = inputScene.getFeatureListSingleObject();
    //if (DEBUG) { cout << "Total N Of features : " << allFeats.size() << endl;   } // ok 
  }
}

// to do: add check the actual object ID.


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
        int objectID1 = (*it).getActualObjectID();
        int objectID2 = (*it2).getActualObjectID();
        inputScene.addAllFeatPairObject( _features, objectID1, objectID2);  // new

      }
    }
 
  }
}






