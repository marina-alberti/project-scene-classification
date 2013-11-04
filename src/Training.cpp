#include "Training.hpp"

#define DEBUG 0
#define TESTFLAG 1

Training::Training(string inputfolder) {
  dirname = inputfolder;
  numberOfFiles = 0;
}

void Training::compute() {
  storeFiles();
  createTrainingSet();
  doTraining();
}

// getAnnotationFileNames change the function name
/* Stores all the file names in the folder into the data member "allFileNames" */
void Training::storeFiles() {
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
      if (TESTFLAG) { cout << "The XML file name is: " << filenameXML << endl;   }
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


void Training::createTrainingSet() {
  trainingFilesList.clear();
  for ( int  i = 0; i < allFileNames.size(); i++ ) {
      trainingFilesList.push_back(allFileNames.at(i));
  }
  if (DEBUG) {
    cout << "The number of training files is : " << trainingFilesList.size() << endl;
  }
}


void Training::doTraining() {

  DatabaseInformation tmpDatabase;  // to do : change!
  storeDatabase = tmpDatabase;

  //******************************************************************************
  // Conversion to Internal data structure
  // Convert objects annotations from KTH manual annotation tool format -> (IDS) 
  storeDatabase.loadAnnotationsInIDS(trainingFilesList);

  //******************************************************************************
  // Feature Extraction  

  cout << "Inside Training Training: Before calling apifeatureextraction from Training" << endl;
  storeDatabase.callApiFeatureExtraction();
  storeDatabase.setFeatureMatrix();
  cout << "Inside Training Training: The feature matrix has been filled." << endl;

  //******************************************************************************
  // Learning module 

  // (i) Learning Object category models 
  storeDatabase.computeGMM_SingleObject_AllFeat(N_CLUSTERS_OBJECT);

  // (ii) Learning spatial relations between different object categories 
  storeDatabase.computeGMM_PairObject_AllFeat(N_CLUSTERS_PAIR); 

  //******************************************************************************
  
  // Store the learned parameters into the data members of Training class 

  learnedModelSingleObject = storeDatabase.getLearnedModelSingleObject();
  learnedModelPairObject = storeDatabase.getLearnedModelPairObject();

  // Stores also the values of mean and std for the normalization of the feature matrices.
  meanNormalization = storeDatabase.getmeanNormalization();
  stdNormalization = storeDatabase.getstdNormalization();
  storeDatabase.computeObjectFrequencies();

  // stores the frequency count for the different object categories and co-occurrence of object categories
  countObjectFrequencies = storeDatabase.getObjectFrequencies();
  countObjectFrequencies1 = storeDatabase.getObjectFrequencies1();
  countObjectPairFrequencies = storeDatabase.getObjectPairFrequencies();
  
  meanNormalizationPair = storeDatabase.getmeanNormalizationPair();
  stdNormalizationPair = storeDatabase.getstdNormalizationPair();
  maxFeatPair =  storeDatabase.getmaxFeatPair();
  minFeatPair =  storeDatabase.getminFeatPair();
  thresholds = storeDatabase.getThresholds();
  
  //******************************************************************************

  if (TESTFLAG) {
    cout <<  "Inside Training::doTraining: the size of learnedModelPairObject is : " 
         << learnedModelPairObject.size() << endl;
    cout <<  "Inside Training::doTraining: the size of learnedModelSingleObject is : " 
         << learnedModelSingleObject.size() << endl;
  }

}

DatabaseInformation Training::getDatabaseInformation() {
  return storeDatabase;
}



/* Test of unkown scene. */
/*
void LOOCV::doTest() {
  cout << "TestFile = "  << testFilesList << endl;

  TestScene unknownScene(testFilesList, learnedModelSingleObject, meanNormalization, stdNormalization, learnedModelPairObject, countObjectFrequencies, countObjectFrequencies1, meanNormalizationPair, stdNormalizationPair, countObjectPairFrequencies, maxFeatPair, minFeatPair, thresholds);

  if (DEBUG) {
    cout << endl<< "Inside LOOCV Test. Before Loading annotations. " << endl;
  }

  // choose 1 to randomly remove 1 object from the test scene, 0 for normal behaviour.
  unknownScene.loadAnnotation(BOOLREMOVE);

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

  // double prob = unknownScene.computeProbObjectPairs_AllFeats();   // old version

  double prob = unknownScene.computeSimilarityScore();
  cout << "In LOOCV do Test after calling computeProbObjectPairs_AllFeats()" << endl;
  probSceneListLoocv.push_back(prob);
  
  cout << "In LOOCV do Test after push back " << endl;
 

}

*/

// to do: eliminate for loop - write an inline function
/*
void LOOCV::createTestSet(int index) {
  testFilesList = allFileNames.at(index);

  //testFilesList = "./mock_Duplicate/710-25-06-13-afternoonNotebook.xml";
  //testFilesList = "./mock_Duplicate/719-27-06-13-morningPhone.xml";
}
*/








