#include "LOOCV.hpp"

#define DEBUG 0

LOOCV::LOOCV(string inputfolder)  {
  dirname = inputfolder;
  numberOfFiles = 0;
  testFilesList = "";

}

void LOOCV::compute() {
  storeFiles();
 /* Following computatino should be for each cross validation fold */
  for (int i = 1; i < 2 ; i++) {      //; i < allFileNames.size(); i++ ) {    // allFileNames.size()
    // i = INDEX_TEST;
    createTrainingSet(i);
    createTestSet(i);
    doTraining();
    doTest();

    if (DEBUG) {
      cout << std::endl << " Inside LOOCV compute. End test. " << endl;
    }
    
  }
  for (int j = 0; j < probSceneListLoocv.size(); j++ ) {
    cout << "Scene "  << j << " :   " << probSceneListLoocv[j] << endl;
  }
}

// getAnnotationFileNames change the function name
void LOOCV::storeFiles() {
  DIR *dp;
  struct dirent *ep;
  struct stat filestat;
  const char * dirnameC = dirname.c_str();
  dp = opendir(dirnameC);
  if (dp != NULL) {
    // for each file inside the data directory
    while ((ep = readdir(dp)) != NULL) {
      if (DEBUG) {cout << "START loop cicle: " << ep->d_name << " is the name. "<< endl; }
      if (S_ISDIR(filestat.st_mode)) {continue; } 
      if ((strcmp(ep->d_name, ".") == 0) || (strcmp(ep->d_name, "..") == 0)) {continue; }
      string filepath = dirname + "/" + ep->d_name;
      string filenameXML = filepath;
      if (DEBUG) { cout << "The XML file name is: " << filenameXML << endl;   }
      allFileNames.push_back(filenameXML);
      numberOfFiles++;
    }
    closedir (dp);
  }
  else {
    perror ("Could not open the directory!");
  }
  if (DEBUG) {
    cout << "The total nunber of files is : " << numberOfFiles <<  endl;
    cout << "The stored files are : " << allFileNames.size() << endl;
  }
}


void LOOCV::createTrainingSet(int index) {
  trainingFilesList.clear();
  for ( int  i = 0; i < allFileNames.size(); i++ ) {
    if (i != index) {
      trainingFilesList.push_back(allFileNames.at(i));
    }
  }
  if (DEBUG) {
    cout << "The number of training files is : " << trainingFilesList.size() << endl;
  }
}

// to do: eliminate for loop - write an inline function
void LOOCV::createTestSet(int index) {

  testFilesList = allFileNames.at(index);

}


void LOOCV::doTraining() {
  DatabaseInformation storeDatabase(trainingFilesList);
  storeDatabase.loadAnnotationsInIDS ();

  /* Feature Extraction  */
  cout << "Before calling apifeatureextraction from LOOCV" << endl;
  storeDatabase.callApiFeatureExtraction();
  cout << "The total number of Scenes is: " << storeDatabase.getNumberOfScenes() << endl;
  storeDatabase.setFeatureMatrix();
  cout << "The feature matrix has been filled." << endl;

  /* Learning module */
  storeDatabase.computeGMM_SingleObject_AllFeat(N_CLUSTERS_OBJECT);
  storeDatabase.computeGMM_PairObject_SingleFeat(N_CLUSTERS_PAIR);   

  learnedModelSingleObject = storeDatabase.getLearnedModelSingleObject();
  learnedModelPairObject = storeDatabase.getLearnedModelPairObject();

  cout <<  "Inside LOOCV Do training :the size of learnedModelPairObject is : " << learnedModelPairObject.size() << endl;
  meanNormalization = storeDatabase.getmeanNormalization();
  stdNormalization = storeDatabase.getstdNormalization();

  // storeDatabase.printFeatureMatrix();

  cout << endl ;
}


void LOOCV::doTest() {
  cout << "TestFile = "  << testFilesList << endl;
  TestScene unknownScene(testFilesList, learnedModelSingleObject, meanNormalization, stdNormalization, learnedModelPairObject);

  if (DEBUG) {
    cout << endl<< "Inside Test. Before Loading annotations. " << endl;
  }

  unknownScene.loadAnnotation();

  if (DEBUG) {
    cout << endl<< "Inside Test. Before extracting features. " << endl;
  }
  unknownScene.extractFeatures();

  if (DEBUG) {
    cout << endl<< "Inside Test. Before predicting object classes. " << endl;
  }
  /* Compute probabilities for 3 objects */
  unknownScene.predictObjectClasses();

  if (DEBUG) {
    cout << std::endl << " Inside LOOCV doTest. End test. " << endl;
  }

  unknownScene.extractFeaturesPairObjects();

  double prob = unknownScene.computeProbObjectPairs();

  probSceneListLoocv.push_back(prob);

  

}









