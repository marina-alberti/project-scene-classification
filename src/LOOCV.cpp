#include "LOOCV.hpp"

#define DEBUG 0
#define TESTFLAG 1

LOOCV::LOOCV(string inputfolder)  {
  dirname = inputfolder;
  numberOfFiles = 0;
  testFilesList = "";
  cMatrixSet = 0;

}

void LOOCV::compute() {
  storeFiles();

 /* Following computation should be for each cross validation fold */
  for (int i = 0; i < 3; i++) {      //; i < allFileNames.size(); i++ ) {

    cout << endl << endl << "inside LOOCV looping: LOOP = " << i << endl << endl;
    indexLoop = i;
    // i = INDEX_TEST;
    createTrainingSet(i);
    createTestSet(i);
    doTraining();

    doTest();  // TO DO put back

    if (DEBUG) {
      cout << std::endl << " Inside LOOCV compute. End test. " << endl;
    }
    
  }
  for (int j = 0; j < probSceneListLoocv.size(); j++ ) {
    cout << "Scene "  << j << " :  Simlarity score :  " << probSceneListLoocv[j] << endl;
  }
}

// getAnnotationFileNames change the function name
/* Stores all the file names in the folder into the data member "allFileNames" */
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
  //testFilesList = "./mock_Missing_mouse/710-27-06-13-morning.xml";
  // testFilesList = "./mock_Duplicate/719-25-06-13-morning_mouse.xml";
}


void LOOCV::doTraining() {

  DatabaseInformation storeDatabase(trainingFilesList);

  // Convert objects annotations from KTH manual annotation tool format ->
//     -> Internal Data Structure (IDS) 
  storeDatabase.loadAnnotationsInIDS();

  // Feature Extraction  
  cout << "Inside LOOCV Training: Before calling apifeatureextraction from LOOCV" << endl;
  storeDatabase.callApiFeatureExtraction();
  cout << "Inside LOOCV Training: The total number of Scenes is: " << storeDatabase.getNumberOfScenes() << endl;
  storeDatabase.setFeatureMatrix();
  cout << "Inside LOOCV Training: The feature matrix has been filled." << endl;


  // Learning module 
  // (i) Learning Object category models 
  storeDatabase.computeGMM_SingleObject_AllFeat(N_CLUSTERS_OBJECT);


  // (ii) Learning spatial relations between different object categories 
  // // storeDatabase.computeGMM_PairObject_SingleFeat(N_CLUSTERS_PAIR);   
  storeDatabase.computeGMM_PairObject_AllFeat(N_CLUSTERS_PAIR); 

  // Store the learned models into the data members of LOOCV class 
  learnedModelSingleObject = storeDatabase.getLearnedModelSingleObject();
  learnedModelPairObject = storeDatabase.getLearnedModelPairObject();

  if (TESTFLAG) {
    cout <<  "Inside LOOCV Do training: the size of learnedModelPairObject is : " 
         << learnedModelPairObject.size() << endl;
    cout <<  "Inside LOOCV Do training: the size of learnedModelSingleObject is : " 
         << learnedModelSingleObject.size() << endl;
  }
  meanNormalization = storeDatabase.getmeanNormalization();
  stdNormalization = storeDatabase.getstdNormalization();

  // storeDatabase.printFeatureMatrix();

  cout << endl ; 

}


/* Test of unkown scene. */
void LOOCV::doTest() {
  cout << "TestFile = "  << testFilesList << endl;

  TestScene unknownScene(testFilesList, learnedModelSingleObject, meanNormalization, stdNormalization, learnedModelPairObject);

  if (DEBUG) {
    cout << endl<< "Inside LOOCV Test. Before Loading annotations. " << endl;
  }

  // choose 1 to randomly remove 1 object from the test scene, 0 for normal behaviour.
  unknownScene.loadAnnotation(0);

  if (DEBUG) {
    cout << endl<< "Inside LOOCV Test. Before extracting features. " << endl;
  }
  unknownScene.extractFeatures();

  if (DEBUG) {
    cout << endl<< "Inside LOOCV Test. Before predicting object classes. " << endl;
  }
  // Compute probabilities for 3 objects + classify objects. 
  unknownScene.predictObjectClasses();

  // add here the LOOCV for object class prediction - performance computation.
  unknownScene.evaluateObjectClassificationPerformance();
  cv::Mat cMatrix = unknownScene.getConfusionMatrix();

  if ( cMatrixSet == 0 ) {
    cMatrixObjectClassification = cMatrix.clone();
    cMatrixSet = 1;
  }
  else {
    cMatrixObjectClassification = cMatrixObjectClassification + cMatrix;
  }
 
  if (TESTFLAG) {
    cout << "The total confusion matrix is: " << endl << cMatrixObjectClassification << endl;
  }

  unknownScene.extractFeaturesPairObjects_HandleMissing();

  double prob = unknownScene.computeProbObjectPairs_AllFeats();

  probSceneListLoocv.push_back(prob);
  
 

}










