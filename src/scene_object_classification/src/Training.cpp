#include "Training.hpp"

#define DEBUG 0
#define TESTFLAG 1

Training::Training() {
  
  numberOfFiles = 0;
}

void Training::createTrainingSet(string inputfolder, bool simulationDatabase) {
  
  dirname = inputfolder;

  if (simulationDatabase == false) {
    storeFiles();
    createTrainingSet_Real();
  }
  else {
    createTrainingSet_Simulation();
  }
  //doTraining();
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


void Training::createTrainingSet_Real() {
  trainingFilesList.clear();
  for ( int  i = 0; i < allFileNames.size(); i++ ) {
      trainingFilesList.push_back(allFileNames.at(i));
  }
  if (DEBUG) {
    cout << "The number of training files is : " << trainingFilesList.size() << endl;
  }

  DatabaseInformation tmpDatabase;  // to do : change!
  storeDatabase = tmpDatabase;

  // Convert objects annotations from KTH manual annotation tool format -> (IDS) 
  storeDatabase.loadAnnotationsInIDS(trainingFilesList);

}


void Training::createTrainingSet_Simulation() {

  DatabaseInformation tmpDatabase;  // to do : change!
  storeDatabase = tmpDatabase;
  storeDatabase.loadAnnotationsInIDS_Simulation(dirname);  

}

void Training::doTraining() {

  //******************************************************************************
  // Feature Extraction  

  storeDatabase.callApiFeatureExtraction();
  storeDatabase.setFeatureMatrix();
  
  //******************************************************************************
  // Learning module 

  // (i) Learning Object category models 
  storeDatabase.computeGMM_SingleObject_AllFeat(N_CLUSTERS_OBJECT);
  
  // (ii) Learning spatial relations between different object categories 
  storeDatabase.computeGMM_PairObject_AllFeat(N_CLUSTERS_PAIR); 
  if (TESTFLAG) {
    cout << "Inside Training class after computing GMM pair of Objects" << endl;
  }
  //******************************************************************************
  
  // Store the learned parameters into the data members of Training class 

  learnedModelSingleObject = storeDatabase.getLearnedModelSingleObject();
  learnedModelPairObject = storeDatabase.getLearnedModelPairObject();

  if (TESTFLAG) {
    cout << "Inside Training class after storing the learned GMM models" << endl;
  }

  // Stores also the values of mean and std for the normalization of the feature matrices.
  // meanNormalization = storeDatabase.getmeanNormalization();
  // stdNormalization = storeDatabase.getstdNormalization();
  storeDatabase.computeObjectFrequencies();

  if (TESTFLAG) {
    cout << "Inside Training class after getting the object co occurrence frequecies" << endl;
  }

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


