#include "Object.hpp"
#include "SceneInformation.hpp"
#include "ApiConvertKTHDB.hpp"
#include "ObjectFeatures.hpp"
#include "DatabaseInformation.hpp"
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std; 

int main(int argc, char *argv[]) {

  if (argc < 2) {
    cout << "Insert the data folder with the XML annotation files." << endl;
    return 0;
  }
    
  string dataFolderName = (argv[ 1 ]);
  cout << "The XML file name is: " << dataFolderName << endl; 
  string inputFolder =  "./" +  dataFolderName ;
  DatabaseInformation storeDatabase( inputFolder );

  storeDatabase.loadAnnotationsInIDS();
  cout << "Before calling apifeatureextraction from main" << endl;
  storeDatabase.callApiFeatureExtraction();
  
  cout << "The total number of Scenes is: " << storeDatabase.getNumberOfScenes() << endl;
  storeDatabase.setFeatureMatrix();
  cout << "The feature matrix has been filled." << endl;
  storeDatabase.printFeatureMatrix();
  storeDatabase.computeGMM_SingleObject_SingleFeat(3);
  storeDatabase.computeGMM_SingleObject_AllFeat(1);
  // storeDatabase.computeGMM_PairObject_SingleFeat(3);
  return 0;
}

/* //  the previous main function
int main(int argc, char *argv[]) {


  if (argc < 2) {
    cout << "Insert the XML file name." << endl;
    return 0;
  }
  string filenameXML = (argv[ 1 ]);
  cout << "The XML file name is: " << filenameXML << endl;  
  ApiConvertKTHDB convertKTHannotation(filenameXML);
  SceneInformation exampleScene;
  convertKTHannotation.parseFileXML(exampleScene);
 // exampleScene.showSceneInformation();
  

  exampleScene.orderObjectList();

 
  vector<Object> listObjectsScene = exampleScene.getObjectList();

  ObjectFeatures computationFeaturesFirstObject(listObjectsScene[0]);

  computationFeaturesFirstObject.computeVolumeSize();
  computationFeaturesFirstObject.computeSizeProjectedX();
  computationFeaturesFirstObject.computeSizeProjectedY();
  computationFeaturesFirstObject.computeSizeProjectedZ();


  return 0;

} */


