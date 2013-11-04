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
  for (int i = 0; i < allFileNames.size(); i++) {      	// allFileNames.size()
      cout << endl << endl << "inside LOOCV looping: LOOP = " << i << endl << endl;
      indexLoop = i;
      // i = INDEX_TEST;
      createTrainingSet(i);
      createTestSet(i);
      doTraining();
      cout << endl << endl << "inside LOOCV looping: LOOP = " << i << endl << endl;
      doTest(); 

      if (TESTFLAG) {
        cout << std::endl << " Inside LOOCV compute. End test. " << endl;
      }
    }
        evaluatePerformance(cMatrixObjectClassification);
  for (int j = 0; j < probSceneListLoocv.size(); j++ ) {
    cout  << (probSceneListLoocv[j]) << endl;
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

  //testFilesList = "./mock_Duplicate/710-25-06-13-afternoonNotebook.xml";
  //testFilesList = "./mock_Duplicate/719-27-06-13-morningPhone.xml";
}


void LOOCV::doTraining() {

  DatabaseInformation storeDatabase;

  // Convert objects annotations from KTH manual annotation tool format ->
//     -> Internal Data Structure (IDS) 
  storeDatabase.loadAnnotationsInIDS(trainingFilesList);

  // Feature Extraction  
  cout << "Inside LOOCV Training: Before calling apifeatureextraction from LOOCV" << endl;
  storeDatabase.callApiFeatureExtraction();
  cout << "Inside LOOCV Training: The total number of Scenes is: " << storeDatabase.getNumberOfScenes() << endl;
  storeDatabase.setFeatureMatrix();
  cout << "Inside LOOCV Training: The feature matrix has been filled." << endl;


  // Learning module 
  // (i) Learning Object category models 
  storeDatabase.computeGMM_SingleObject_AllFeat(N_CLUSTERS_OBJECT);

  // storeDatabase.computeGMMSingleObject_Onemodel();

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

  cout << endl ; 

}


/* Test of unkown scene. */
void LOOCV::doTest() {
  cout << "TestFile = "  << testFilesList << endl;

  TestScene unknownScene(storeDatabase);

  if (DEBUG) {
    cout << endl<< "Inside LOOCV Test. Before Loading annotations. " << endl;
  }

  // choose 1 to randomly remove 1 object from the test scene, 0 for normal behaviour.
  unknownScene.loadAnnotation(testFilesList, BOOLREMOVE);

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










