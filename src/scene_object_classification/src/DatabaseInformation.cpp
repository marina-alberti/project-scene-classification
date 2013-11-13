#include "DatabaseInformation.hpp"

#define DEBUG 0
#define TESTFLAG 1


DatabaseInformation::DatabaseInformation() {
  numberOfScenes = 0;
  // filesList = trainingFilesList;
  // dirname = inputDir;
}


//****************************************************************************
void DatabaseInformation::loadAnnotationsInIDS(vector<string> trainingFilesList) {

  filesList = trainingFilesList;
  for (int i = 0; i < filesList.size(); i++ ) {

    string filenameXML = filesList.at(i);
    if (TESTFLAG)  { cout << "The XML file name is: " << filenameXML << endl;    }
  
    ApiConvertKTHDB convertKTHannotation(filenameXML);
    SceneInformation currentScene;
    convertKTHannotation.parseFileXML(currentScene);
    sceneList.push_back(currentScene);
    numberOfScenes = numberOfScenes + 1; 
    if (DEBUG) {cout << "Added a new scene to the sceneList. " << endl; } 

  }
}

//****************************************************************************
/* 
   Loads into the "sceneList" data member of this "DatabaseInformation" class
   all the scenes present in the given file.
   TO DO: add first and last scenes to load, and add possibility 
   to load all scenes but 1 for leave-one-out cross-validation
*/

void DatabaseInformation::loadAnnotationsInIDS_Simulation(string fileAnnotations) {
 
  if (TESTFLAG)  { cout << "The JSON file name is: " << fileAnnotations << endl;    }
  
  ApiConvertSimulationDB convertAnnotation(fileAnnotations);
  convertAnnotation.parseFileJSON(sceneList);   // pass by reference the scene list 

  numberOfScenes = sceneList.size(); 

  if (TESTFLAG)  { cout << "The number of scenes in the database is : " << numberOfScenes << endl; }
 
}


//****************************************************************************
/*
This function - for each scene in "sceneList" - calls to apiFeatureExtraction
and stores the features in the vectors of features of the scene
*/
void DatabaseInformation::callApiFeatureExtraction() {
  
  for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {

    if (DEBUG) {cout << endl << endl << "Feature Extraction STARTS: "<< endl 
	<< "Selecting a new scene for feature extration (inside DatabaseInformation::callApiFeatureExtraction)" << endl; }
    
    SceneInformation currentScene = (*it);  
    ApiFeatureExtraction extractFeaturesApi; 
    extractFeaturesApi.extractFeaturesSingleObjects(*it);  
    extractFeaturesApi.extractFeaturesPairObjects(*it); 
  }
}
//****************************************************************************

/*
  This function collects all the features stor  void computeGMMSingleObject_Onemodel();ed into the field "featureList" of 
  each scene "SceneInformation" into the 2d matrix "featureMatrix" data member of this class.
  For each "SceneInformation" object in sceneList:
  goes through vector<FeatureInformation> featureListSingleObject
  stores it in a row of featureMatrix
*/
/*
void DatabaseInformation::setFeatureMatrix() {

  for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
    SceneInformation currentScene = (*it);

    // The SingleObjectFeatures computation 
    vector<FeatureInformation> currentSceneFeaturesSingleObject =     currentScene.getFeatureListSingleObject();
    if (DEBUG) { 
      cout << "In DBINFO set FMatrix : The size of the vector of Feat SINGLE_OBJ of current scene is : " <<  		  currentSceneFeaturesSingleObject.size() << endl;
    }
    featureMatrixSingleObject.push_back(currentSceneFeaturesSingleObject);

    // The pair of Objects Features Computation 
    vector <FeatureInformation> currentSceneFeaturesPairObject =     currentScene.getFeatureListPairObject();
    if (DEBUG) { 
      cout << "In DBINFO set FMatrix : The size of the vector of Feat PAIR_OBJ of current scene is : " <<  currentSceneFeaturesPairObject.size() << endl;
    }
    featureMatrixPairObject.push_back(currentSceneFeaturesPairObject);
  }
}
*/


//****************************************************************************
/*  New version of the function : creates feature matrices for all objects 
     and pairs objects, in predefined category list */ 

void DatabaseInformation::setFeatureMatrix() {

  if (TESTFLAG) {
    cout << "In setFeatureMatrix : start." << endl;
  }
  // Single Object FM
  // initialize the feature matrix (I already know the Number of predifined object classes)
  FMSingleObject.reserve(NOBJECTCLASSES);

  if (TESTFLAG) {
    cout << "In setFeatureMatrix : reserved space in the matrix, == NOBJECTCLASSES." << endl;
  }

  // iterate over set of object class IDs, predefined 
  for (int i = 0; i < NOBJECTCLASSES ; i++) {
    if (TESTFLAG) {
      cout << "In setFeatureMatrix : current object class ID: i =  " << i << endl;
    }
    int countScene = 0;
    int nScene = sceneList.size();
    vector<vector<FeatureInformation> > currentFeaturesScenes;
    //(FMSingleObject[i]).resize(nScene);
    // iterate over all scenes in the database
    for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
      //SceneInformation currentScene = (*it);

      if (DEBUG) { 
        cout << "In setFeatureMatrix : scene ID:  " << countScene << endl;
      }

      // iterate over all sets of features (from different objects) present in the current scene of the database
      vector<AllFeatSingleObject> allFeatureCurrentObject = (*it).getAllFeatSingleObject();
      if (TESTFLAG) {
      //  cout << "In setFeatureMatrix : size of allFeatureCurrentObject is : " << allFeatureCurrentObject.size() << endl;
      }

      for(vector<AllFeatSingleObject>::iterator it2 = allFeatureCurrentObject.begin(); it2 != allFeatureCurrentObject.end(); ++it2) {


        // if the current set of features is actually from the currently considered object class
        
        int currentID  = (*it2).getObjectID();
        
        if ( currentID == i) {

          vector<FeatureInformation> currentFeatList = (*it2).getAllFeatures(); 
          if (TESTFLAG) {
            // cout << "In setFeatureMatrix : size of currentFeatList:  " << currentFeatList.size() << "object class ID : " << i <<endl;
          }
          currentFeaturesScenes.push_back(currentFeatList);  
          
        }
      
      }
      countScene++;
    }
    FMSingleObject.push_back(currentFeaturesScenes);
  }

  if (TESTFLAG) {
    cout << "In setFeatureMatrix : end of single object features." << endl;
    cout << "The size of FMSingleObject is :  " << FMSingleObject.size() << endl;
    cout << "The size of FMSingleObject 1 = Monitor is :  " << (FMSingleObject[0]).size() << endl;
    cout << "The size of FMSingleObject 2 = keyboard is :  " << (FMSingleObject[1]).size() << endl;
    cout << "The size of FMSingleObject 3 = mouse is :  " << (FMSingleObject[2]).size() << endl ;
    cout << "The size of FMSingleObject 4 = mug is :  " << (FMSingleObject[3]).size() << endl;
    cout << "The size of FMSingleObject 5 = lamp is :  " << (FMSingleObject[4]).size() << endl;
    cout << "The size of FMSingleObject 6 = laptop is :  " << (FMSingleObject[5]).size() << endl;
    cout << "The size of FMSingleObject 7 = pen is :  " << (FMSingleObject[6]).size() << endl;

  }
   
  // *************************************************************************************************
  // Pair Object FM 

  // initialize the feature matrix (I already know the Number of predifined object classes)
  FMPairObject.reserve(NOBJECTCLASSES);
  // iterate over set of object class IDs, predefined 
  for (int i = 0; i < NOBJECTCLASSES ; i++) {
    vector<vector<vector<FeatureInformation > > > vectorObj1;
    for (int j = 0; j < NOBJECTCLASSES ; j++) {

      //if (i != j) {  // to check

        if (TESTFLAG) {
          cout << "In setFeatureMatrix : current object classes IDs : i =  " << i << " and j = " << j << endl;
        }

        vector<vector<FeatureInformation> > currentFeaturesScenes;
        // iterate over all scenes in the database
        int countScene = 0;
        for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
          //SceneInformation currentScene = (*it);
        if (DEBUG) {
          cout << "In setFeatureMatrix : scene ID:  " << countScene << endl;
        }
          // iterate over all sets of features (from different object pairs) present in the current scene of the database
          vector<AllFeatPairObject> allFeatureCurrentObjectPair =  (*it).getAllFeatPairObject();
          for(vector<AllFeatPairObject>::iterator it2 = allFeatureCurrentObjectPair.begin(); it2 != allFeatureCurrentObjectPair.end(); ++it2) {
            // if the current set of features is actually from the currently considered object pair classes
            if ( ((*it2).getObjectID1() == i ) && ( (*it2).getObjectID2() == j ) ) {
              vector<FeatureInformation> currentFeatList = (*it2).getAllFeatures();
              currentFeaturesScenes.push_back(currentFeatList); 
            }
          }
          countScene++;
        }
        vectorObj1.push_back(currentFeaturesScenes);
      //}
    }
    FMPairObject.push_back(vectorObj1);
  }


 
}


void DatabaseInformation::printFeatureMatrix() {
// for each scene    
  int Ocount = 0;
// Printing the matrix with features from the Single Object 
  for(vector<vector<vector<FeatureInformation> > >::iterator it = FMSingleObject.begin(); it != FMSingleObject.end(); ++it) {
    Ocount ++;
    cout << endl << "Object : " << Ocount << endl << endl; // <<  "Scene" << Scenec << endl; 
    // for each object in predefined list
    vector<vector<FeatureInformation> > featureVectorO = (*it); 
    
    for ( vector<vector<FeatureInformation> >::iterator it2 = featureVectorO.begin(); it2 != featureVectorO.end(); ++it2) {
      cout << endl;
      vector<FeatureInformation> currentSceneFeature = (*it2);

      for ( vector<FeatureInformation>::iterator it3 = currentSceneFeature.begin(); it3 != currentSceneFeature.end(); ++it3) {
      FeatureInformation  currentFeature = *it3;
      vector<float> currentFeatureValues = currentFeature.getAllValues();
      for ( vector <float>::iterator it3 = currentFeatureValues.begin(); it3 != currentFeatureValues.end(); ++it3)  {
          cout << (*it3) << "   " ;
        }
      }

    }
  }
  cout << endl << endl << endl;
  /*
  // // Printing the matrix with features from the Object Pair 
  Scenec = 0;

  for(vector<vector<FeatureInformation> >::iterator it = featureMatrixPairObject.begin(); it != featureMatrixPairObject.end(); ++it) {
    Scenec ++;
    cout << endl ; // <<  "Scene" << Scenec << endl; 
    // for each feature of the current scene
    vector<FeatureInformation> featureVectorScene = (*it); 
    
    for ( vector<FeatureInformation>::iterator it2 = featureVectorScene.begin(); it2 != featureVectorScene.end(); ++it2) {
      
      FeatureInformation currentFeature = (*it2);
      vector<float> currentFeatureValues = currentFeature.getAllValues();
      for ( vector <float>::iterator it3 = currentFeatureValues.begin(); it3 != currentFeatureValues.end(); ++it3) {
        cout << (*it3) << "   " ;
      }
    }
  }
*/ 
}



/*
  For each feature iterate over all the scenes
  vector <featureInformation> : vectorOfFeat for the current scene
  extract feat_i
  add it to Mat obj (nScenes x Dim_Feat) as double. // samples.at<double>(countScene, 1) = random_number;
  Train EM model and get means, variances, weights (1x3)
  these should be stored / returned.
  I will have NFeat EM models
*/
/*0, 0, 0, 0
void DatabaseInformation::computeGMM_SingleObject_SingleFeat(int nclusters) {

  // compute the number of features 
  vector<FeatureInformation> exv1 = featureMatrixSingleObject.at(1);
  int nFeats = exv1.size();
  if (DEBUG) {
    cout << endl << endl << "Starting Compute GMM for Single Objects / Svoid DatabaseInformation::computeGMM_SingleObject_SingleFeat(int nclusters)ingle Feats " << endl << endl;
  }

  // for each feature (Single Feat of Single Object)
  for ( int i = 0; i < nFeats; i++ ) { 

    vector<FeatureInformation> exv = featureMatrixSingleObject.at(1);
    FeatureInformation exf = exv.at(i);

    // compute the feature size
    int fsize = exf.getFeatureSize();
    if (DEBUG) {
      cout << endl << endl << " Feat Number : " << i << endl ;
      cout << "Dimensionality of current Feat: " << fsize << endl;
    }
    cv::Mat FeatMat = cv::Mat::zeros ( numberOfScenes, fsize,  CV_64F ); // double, (nScenes x FeatSize)
    // for each scene
    int countScene = 0;
    for(vector<vector<FeatureInformation> >::iterator it = featureMatrixSingleObject.begin(); it != featureMatrixSingleObject.end(); ++it) {
      vector<FeatureInformation> featureVectorCurrentScene = (*it);
      FeatureInformation currentFeature = featureVectorCurrentScene.at(i);
      vector<float> currentFeatureValues = currentFeature.getAllValues();  
      // depending on dimentionality of that feature: I add all values in current row
      for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
        FeatMat.at<double>(countScene, j) = (double) (currentFeatureValues.at(j));
      }
    countScene++;
    } 
    cv::EM em_model(nclusters);
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( FeatMat ); 
    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }
    cv::Mat _means = em_model.get<cv::Mat>("means");
    cv::Mat _weights = em_model.get<cv::Mat> ("weights");
    vector<cv::Mat> _covs = em_model.get<vector<cv::Mat> > ("covs");
   
    if (DEBUG) { 
      std::cout << "The size of the means is:  " << _means.size()  << "  and weights : "
	 << _weights.size()  << std::endl ;
    }
  }
}

*/

/* For each object - 
	for each scene -
		take all feats from the current scene relative to the current object	
		put all the values in 1 single vector (independently of feature dimensionality)	
		this vector is the Nscenes x Dim mat to train the EM model for the current object
  
  Return for each Object the mean variances and weight of the mixture component
  I will need more components WRT the Single Feature modelling of the SingleObject
  I need to normalize the features.
 */
/*
void DatabaseInformation::computeGMM_SingleObject_AllFeat(int nclusters) {

  // compute the number of features 
  int numberOfObjects = 3;  // to do : compute it from the vector_Of_SceneInformation
  vector<FeatureInformation> exv1 = featureMatrixSingleObject.at(0);
  int nFeatsTotal = exv1.size();
  int nFeats = nFeatsTotal / numberOfObjects;
  int fsize = 9;        // to do: compute it
  
  if (DEBUG) {
    cout << endl << endl << "Starting Compute GMM for Single Objects / All Feats " << endl << endl;
  }

  // for each object
  for ( int i = 0 ; i < numberOfObjects; i++ ) {
    int countScene = 0;
    int index_feat_min = i * nFeats;  
    int index_feat_max = i * nFeats + nFeats;
    cv::Mat FeatMat = cv::Mat::zeros ( numberOfScenes, fsize,  CV_64F ); 
    if (DEBUG)  {cout << "Current object :  " << i << endl; }

    // for each scene
    for(vector<vector<FeatureInformation> >::iterator it = featureMatrixSingleObject.begin(); it != featureMatrixSingleObject.end(); ++it) {
      int countFeat = 0;  
      if (DEBUG)  {
        cout << "  Current scene :  " << countScene << endl; 
      }

      // for each feature of the current scene and belonging to current object
      for (int index = index_feat_min; index < index_feat_max; index++ ) {
        FeatureInformation currentFeature = (*it).at(index) ;  
        vector<float> currentFeatureValues = currentFeature.getAllValues();  
        // depending on dimentionality of that feature: I add all values in current row
        for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
          FeatMat.at<double>(countScene, countFeat) = (double) (currentFeatureValues.at(j));
          countFeat++;
        }
      }
      countScene++; 
    }
    // // normalize the feature matrix : 
    cv::Mat normalizedFeatMat = FeatMat.clone();
    vector<double> currentObjectMean;
    vector<double> currentObjectStd;
    for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
      cv::Mat currentCol = FeatMat.col(icolumn);
      double s = 0;
      double s_std = 0;
      for (int c = 0; c < currentCol.rows ; c++ ) {
        s += currentCol.at<double>(c);
      }
      double _mean = s / currentCol.rows; 
      for (int c = 0; c < currentCol.rows ; c++ ) {
        s_std += pow( ( currentCol.at<double>(c) - _mean) , 2);
      }   
      double _std =sqrt(s_std / currentCol.rows);   
      if (_std == 0) { _std == 1; }
      currentObjectMean.push_back(_mean);
      currentObjectStd.push_back(_std);
      for (int c = 0 ; c < currentCol.rows ; c++ ) {
        // normalizedFeatMat.at<double>(c, icolumn) = (currentCol.at<double>(c) - _mean) / (_std );
      }
    }
    meanNormalization.push_back(currentObjectMean);
    stdNormalization.push_back(currentObjectStd);
    // // end normalization feature matrix.

    // test: select lower dimensionality of feature matrix   
    cv::Mat featsTrain = normalizedFeatMat.colRange(0, 9);     
    if (DEBUG) {
      cout << endl << endl << "Object : " << i << endl << 
         "The feature matrix dim is " << normalizedFeatMat.size() << endl;
      cout << endl << endl << "The feature matrix N ROWS is " << normalizedFeatMat.rows << endl;
      cout << endl <<  "The features are " << endl <<  normalizedFeatMat << endl;
      // cv::Mat featsTrain = normalizedFeatMat.clone();  // col(5);
      // cout << "The feature dimensionality for training is now : " << featsTrain.size() << endl;
      // cout << "The features are " <<  featsTrain << endl;
    }
    // End test lower feature dimensionality 

    //  Training EM model for the current object.  
    cv::EM em_model(nclusters);
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( featsTrain );    // normalizedFeatMat to change
    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }
    learnedModelSingleObject.push_back(em_model);

    if (DEBUG) { 
      cv::Mat _means = em_model.get<cv::Mat>("means");
      cv::Mat _weights = em_model.get<cv::Mat> ("weights");
      vector<cv::Mat> _covs = em_model.get<vector<cv::Mat> > ("covs");

      std::cout << "The size of the means is:  " << _means.size()  <<
       "  and weights : " << _weights.size()  << std::endl << 
       std::endl;
     
    }
  }  
}
*/



//****************************************************************************
void DatabaseInformation::computeGMM_SingleObject_AllFeat(int nclusters) {
  int fsize = 9;        // to do: compute it
  int countFeat;
  if (DEBUG) {
    cout << endl << endl << "Starting Compute GMM for Single Objects / All Feats " << endl << endl;
  }

  // for each considered object class ("i" is also the object class ID as stored in Object-> actualObjectID)
  for ( int i = 0 ; i < FMSingleObject.size(); i++ ) {

    if (TESTFLAG)  {cout << "Current object :  " << i << endl; }
    
    // inizialize the feature matrix "FeatMat" for current object class, as a cv::Mat object
    cv::Mat FeatMat = cv::Mat::zeros ( FMSingleObject.at(i).size(), fsize,  CV_64F ); 

    int countScene = 0;
 
    // for each scene in the training database
    for(vector<vector<FeatureInformation> >::iterator it = (FMSingleObject.at(i)).begin(); it != (FMSingleObject.at(i)).end(); ++it) {
      
      // (*it) is a vector of FI
      countFeat = 0;
      // for each feature of the current scene - and belonging to current object class
      for (vector<FeatureInformation>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
        FeatureInformation currentFeature = *it2;  
        vector<float> currentFeatureValues = currentFeature.getAllValues();  
        
        // depending on dimentionality of that feature: I add all values in current row
        for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
          FeatMat.at<double>(countScene, countFeat) = (double) (currentFeatureValues.at(j));
          countFeat++;
        }
      }
      countScene++; 
    }

    // obtain "FeatMat" : <numberOfScenes x numberOfFeatures> (meaning 1-D features)

    // **********************************************************************
    // // // NORMALIZATION
    // //    Normalize the feature matrix "FeatMat": 
    
    vector<double> meansVectorCurrentObject = computeMean(FeatMat);
    vector<double> stdVectorCurrentObject = computeStd(FeatMat, meansVectorCurrentObject);

    //cv::Mat normalizedFeatMat = doNormalization(FeatMat, meansVectorCurrentObject, stdVectorCurrentObject);

    meanNormalization.push_back(meansVectorCurrentObject);
    stdNormalization.push_back(stdVectorCurrentObject);

    cv::Mat normalizedFeatMat = FeatMat.clone();

    /* // old version
    cv::Mat normalizedFeatMat = FeatMat.clone();
    vector<double> currentObjectMean;
    vector<double> currentObjectStd;
    // for each column i.e. each 1-D feature
    for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
      cv::Mat currentCol = FeatMat.col(icolumn);
      double s = 0;
      double s_std = 0;
      for (int c = 0; c < currentCol.rows ; c++ ) {
        s += currentCol.at<double>(c);
      }
      // compute mean
      double _mean = s / currentCol.rows; 
      // compute std
      for (int c = 0; c < currentCol.rows ; c++ ) {
        s_std += pow( ( currentCol.at<double>(c) - _mean) , 2);
      }   
      double _std =sqrt(s_std / currentCol.rows);   
      if (_std == 0) { _std = 1; }
      // store the mean and std values for the current object class and the current feature
      currentObjectMean.push_back(_mean);
      currentObjectStd.push_back(_std);
      // normalize the current column (i.e. feature) of the feature matrix
      for (int c = 0 ; c < currentCol.rows ; c++ ) {
        normalizedFeatMat.at<double>(c, icolumn) = (currentCol.at<double>(c) - _mean) / (_std );
      }
    }
    // store all the mean and std values for all the different features, for the considered object class.
    meanNormalization.push_back(currentObjectMean);
    stdNormalization.push_back(currentObjectStd);
    */

    // *************************************************************************
    // // end NORMALIZATION feature matrix.
    // **********************************************************************

    // Test for feature relevance experiments ->
    // test: select lower dimensionality of feature matrix   
  
    cv::Mat featsTrain = normalizedFeatMat.colRange(0, 9);    
    /*
    cv::Mat featsTrain = cv::Mat(normalizedFeatMat.rows, 4, CV_64F);   
    normalizedFeatMat.col(0).copyTo(featsTrain.col(0));
    normalizedFeatMat.col(1).copyTo(featsTrain.col(1));
    normalizedFeatMat.col(3).copyTo(featsTrain.col(2));
    normalizedFeatMat.col(4).copyTo(featsTrain.col(3));
    */

    if (DEBUG) {
      cout << endl << endl << "Object : " << i << endl << 
         "The feature matrix dim is " << FeatMat.size() << endl;
      cout << endl << endl << "The feature matrix N ROWS is " << FeatMat.rows << endl;
      cout << endl <<  "The features are " << endl <<  FeatMat << endl;
    }
    // END test lower feature dimensionality 
    // **********************************************************************

    //  Training EM model for the current object.  
    cv::EM em_model(nclusters);
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( featsTrain );    // normalizedFeatMat to change
    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }
    learnedModelSingleObject.push_back(em_model);


    // **************************************************************************
    // testing on the same training database:

    cv::Mat _means = em_model.get<cv::Mat>("means");
    cv::Mat _weights = em_model.get<cv::Mat> ("weights");
    vector<cv::Mat> _covs = em_model.get<vector<cv::Mat> > ("covs");

    double minProb = 1000;
    for (int zz = 0; zz < featsTrain.rows; zz++) {
      cv::Mat featsTrainScene = featsTrain.row(zz);
      double prob = computeGMMProbability(featsTrainScene, _means, _covs, _weights);
      prob = log(prob);
      if (prob < minProb) {
        minProb = prob;
      }
      // cout << "Model  " << i << " prob " << prob << endl;
    }
    cout << "Model  " << i << "  minprob  " << minProb << endl << endl;

    thresholds.push_back(minProb);

    // **************************************************************************

    if (DEBUG) {
      cout << "Inside DBInfo compute GMM SO: size of model is now: " << learnedModelSingleObject.size() << endl;
    }


    if (DEBUG) { 
     
       std::cout << "The size of the means is:  " << _means.size()  <<
       "  and weights : " << _weights.size()  << std::endl << 
       std::endl;
       cout << "The mean matrix of current GMM model is : "  << _means << endl;
       cout << "The weights are : " << _weights << endl;
      
    }
  }  
}



//****************************************************************************
void DatabaseInformation::computeGMMSingleObject_Onemodel() {

  // i have to give the labels to the classes

  int fsize = FMSingleObject.size();        // to do: compute it
  int countFeat;
  if (TESTFLAG) {
    cout << endl << endl << "Starting Compute 1 GMM for Single Objects / All Feats " << endl << FMSingleObject.size() << endl;
  }

  // FeatMat will contain the data for all the objects, and will have size = Nsamples x featsize
  //  where Nsamples is the sum of the numbers of samples for each object category

  cv::Mat FeatMat; //= cv::Mat::zeros ( FMSingleObject.at(i).size(), fsize,  CV_64F ); 
  cv::Mat labels;

  // for each considered object class ("i" is also the object class ID as stored in Object-> actualObjectID)
  for ( int i = 0 ; i < FMSingleObject.size(); i++ ) {

    cv::Mat FeatMatSingleObject = cv::Mat::zeros ( FMSingleObject.at(i).size(), fsize,  CV_64F ); 
    cv::Mat labelsSingleObject; // = cv::Mat::zeros( FMSingleObject.at(i).size(), 1, CV_32SC1 );
    if (TESTFLAG)  {cout << "Current object :  " << i << endl; }
    
    int countScene = 0;
 
    // for each scene in the training database
    for(vector<vector<FeatureInformation> >::iterator it = (FMSingleObject.at(i)).begin(); it != (FMSingleObject.at(i)).end(); ++it) {
      
      countFeat = 0;

      // for each feature of the current scene - and belonging to current object class
      for (vector<FeatureInformation>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
        FeatureInformation currentFeature = *it2;  
        vector<float> currentFeatureValues = currentFeature.getAllValues();  
        
        // depending on dimentionality of that feature: I add all values in current row
        for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
          FeatMatSingleObject.at<double>(countScene, countFeat) = (double) (currentFeatureValues.at(j));
          countFeat++;
        }
      }

      labelsSingleObject.push_back(i);
      countScene++; 
      
    }

    // I have to sum up all tis fmatrices
    FeatMat.push_back(FeatMatSingleObject);
    labels.push_back(labelsSingleObject);
    if (TESTFLAG) {
      cout << "FeatMat size is " << FeatMat.size() << endl;
      cout << "labels size is " << labels.size() << endl << labels << endl;
    }
  }

   // **********************************************************************
    // // // NORMALIZATION
    // //    Normalize the feature matrix "FeatMat": 
    
   // vector<double> meansVectorCurrentObject = computeMean(FeatMat);
   // vector<double> stdVectorCurrentObject = computeStd(FeatMat, meansVectorCurrentObject);
    //cv::Mat normalizedFeatMat = doNormalization(FeatMat, meansVectorCurrentObject, stdVectorCurrentObject);
   // meanNormalization.push_back(meansVectorCurrentObject);
   // stdNormalization.push_back(stdVectorCurrentObject);

    cv::Mat normalizedFeatMat = FeatMat.clone();
    // **********************************************************************
    cv::Mat featsTrain = normalizedFeatMat.colRange(0, 3);     
    if (DEBUG) {
      cout << endl << endl << 
         "The feature matrix dim is " << FeatMat.size() << endl;
      cout << endl << endl << "The feature matrix N ROWS is " << FeatMat.rows << endl;
      cout << endl <<  "The features are " << endl <<  FeatMat << endl;
  
    }
    // END test lower feature dimensionality 
    // **********************************************************************

    cv::EM em_model(NOBJECTCLASSES);
    cv::Mat  logLikelihoods;
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( featsTrain ,  logLikelihoods , labels);    
    if (TESTFLAG) {
    cout << endl << logLikelihoods << endl << labels << endl;

    }
    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }



}
//****************************************************************************

/*  
Train a GMM model for pairs of objects considering 1 feature (SR) at a time.
*/
/*
void DatabaseInformation::computeGMM_PairObject_SingleFeat(int nclusters) {

 // compute the number of features 
  vector<FeatureInformation> exv1 = featureMatrixPairObject.at(0);
  int nFeats = exv1.size();
  if (DEBUG) {
    cout << endl << endl << "Starting Compute GMM for Pair Objects / Single Feats " << endl 
	<< "Number of features :  " << nFeats << endl;
  }

  // for each feature (Single Feat of Pair-Of-Object)
  for ( int i = 0; i < nFeats; i++ ) { 

    vector<FeatureInformation> exv = featureMatrixPairObject.at(0);
    FeatureInformation exf = exv.at(i);

    // compute the feature size
    int fsize = exf.getFeatureSize();
    if (DEBUG) {
      cout << endl << endl << " Feat Number : " << i << endl ;
      cout << "Dimensionality of current Feat: " << fsize << endl;
    }
    cv::Mat FeatMat = cv::Mat::zeros ( numberOfScenes, fsize,  CV_64F ); // double, (nScenes x FeatSize)

    // for each scene 
    int countScene = 0;
    for(vector<vector<FeatureInformation> >::iterator it = featureMatrixPairObject.begin(); it != featureMatrixPairObject.end(); ++it) {
 
      vector<FeatureInformation> featureVectorCurrentScene = (*it);  
      
      FeatureInformation currentFeature = featureVectorCurrentScene.at(i);  
      vector<float> currentFeatureValues = currentFeature.getAllValues();   
      // depending on dimentionality of that feature: I add all values in current row
      for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
        FeatMat.at<double>(countScene, j) = (double) (currentFeatureValues.at(j));
      }
    countScene++;
     } 

    cv::EM em_model(nclusters);
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( FeatMat ); 
    learnedModelPairObject.push_back(em_model);

    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }
    cv::Mat _means = em_model.get<cv::Mat>("means");
    cv::Mat _weights = em_model.get<cv::Mat> ("weights");
    vector<cv::Mat> _covs = em_model.get<vector<cv::Mat> > ("covs");

    if (DEBUG) { 
      std::cout << "The size of the means is:  " << _means.size()  << "  and weights : "
	 << _weights.size()  << std::endl;
    }
  }
}

*/




/*  
Train a GMM model for pairs of objects considering ALL the features (SR) at a time.
*/
/*
void DatabaseInformation::computeGMM_PairObject_AllFeat(int nclusters) {


  int numberOfPairs = 6;
  int numberOfFeatTot = (featureMatrixPairObject.at(0)).size();
  int numberOfFeat =   numberOfFeatTot / numberOfPairs;

  // For each pair of objects.  
  for (int i = 0; i < numberOfPairs; i++)  {

    int countScene = 0;
    int index_feat_min = i * numberOfFeat;  
    int index_feat_max = i * numberOfFeat + numberOfFeat;
    cv::Mat FeatMat = cv::Mat::zeros ( numberOfScenes, numberOfFeat,  CV_64F ); 

    for(vector<vector<FeatureInformation> >::iterator it = featureMatrixPairObject.begin(); it != featureMatrixPairObject.end(); ++it) {
        int countFeat = 0;  
        if (DEBUG)  {
          cout << "  Current scene :  " << countScene << endl; 
        }

        // for each feature of the current scene and belonging to current object
        for (int index = index_feat_min; index < index_feat_max; index++ ) {
          FeatureInformation currentFeature = (*it).at(index) ;  
          vector<float> currentFeatureValues = currentFeature.getAllValues();  
          // depending on dimentionality of that feature: I add all values in current row
        for ( int j = 0; j < currentFeatureValues.size() ; j++ ) {
          FeatMat.at<double>(countScene, countFeat) = (double) (currentFeatureValues.at(j));
          countFeat++;
        }
      }
      countScene++; 
    }

    // cv::Mat featsTrain = FeatMat.clone();     
    if (DEBUG) {
      cout << endl << endl << "Object : " << i << endl << 
         "The feature matrix dim is " << FeatMat.size() << endl;
      cout << endl << endl << "The feature matrix N ROWS is " << FeatMat.rows << endl;
      cout << endl <<  "The features are " << endl <<  FeatMat << endl;
    }
    // End test lower feature dimensionality 

    //  Training EM model for the current object.  
    cv::EM em_model(nclusters);
    if (DEBUG) { 
      std::cout << "Training the EM model." << std::endl; 
    }
    em_model.train ( FeatMat );    // normalizedFeatMat to change
    if (DEBUG) { 
      std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
    }
    learnedModelPairObject.push_back(em_model);

  }
}
*/

// normalize ALL the values for ALL object pairs. Then compute STD. // TO DO
double DatabaseInformation::computeStdWeights(cv::Mat FeatMat, vector<double> maxvector, vector<double> minvector) {

  // the normalizde feature matrix
  cv::Mat normalizedFeatMat = FeatMat.clone(); 
  //cv::Mat normalizedFeatMat = doNormalizationMinMax(FeatMat, maxvector, minvector);
  cout << endl << FeatMat << endl;
  // compute the STD on the normalizaed feature matrix
  vector<double> meansVector = computeMean(normalizedFeatMat);
  vector<double> stdVector = computeStd(normalizedFeatMat, meansVector);
  // compute a WEIGHT for the considere OBJECT-PAIR (to be used in scene simlarity score) based on std
  double magnitude = 1;
  for (int i = 0; i < stdVector.size(); i++) {
    if ( i != 2 ) {
      magnitude = magnitude * (stdVector[i]);    // += (pow(stdVector[i], 2));  // magnitude * stdVector[i];   
      if (TESTFLAG) {
        cout << "in DatabaseInformation::computeStdWeights. std vector: " << (stdVector[i]) << endl;
      }
    }
  }
  //magnitude = sqrt(magnitude);
  double out = 1 / magnitude ; //* FeatMat.rows / 100;
  cout << "in DatabaseInformation::computeStdWeights. Final: " << out << endl;
  return out;

}


//****************************************************************************
void DatabaseInformation::computeGMM_PairObject_AllFeat(int nclusters) {

  if (TESTFLAG) {
    cout << "Inside DBInfo compute GMM PAIR O: start." << endl;
  }

  learnedModelPairObject.reserve(NOBJECTCLASSES);
  int numberOfFeat = FMPairObject[0].size();       // 5; compute it
  int countFeat;

  // loop over reference object i
  for ( int i = 0 ; i < FMPairObject.size(); i++ ) {

    vector<cv::EM> internalEMvector;  // to work with vector of vector
    vector<vector<double> > meanNormalizationPair_currentRef;
    vector<vector<double> > stdNormalizationPair_currentRef;
    vector<vector<double> > mintmp;
    vector<vector<double> > maxtmp;

    // loop over target object j
    for ( int j = 0;  j < FMPairObject[i].size(); j++) {
    
      if (TESTFLAG) {
        cout << "Inside DBInfo compute GMM PAIR O: size of model is now: " << learnedModelPairObject.size() << " for object classes :   " << i << " and " << j << endl;
      }

      int numberOfScenes = FMPairObject[i][j].size();
      int countScene = 0;
      cv::Mat FeatMat = cv::Mat::zeros ( numberOfScenes, numberOfFeat,  CV_64F ); 
     
      for(vector<vector<FeatureInformation> >::iterator it = (FMPairObject[i][j]).begin(); it != (FMPairObject[i][j]).end(); ++it) {
        countFeat = 0;
        // for each feature of the current scene and belonging to current object
        for (vector<FeatureInformation>::iterator it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
          FeatureInformation currentFeature = *it2;  
          vector<float> currentFeatureValues = currentFeature.getAllValues();  
          // depending on dimentionality of that feature: I add all values in current row
          
          for ( int k = 0; k < currentFeatureValues.size() ; k++ ) {
            FeatMat.at<double>(countScene, countFeat) = (double) (currentFeatureValues.at(k));
            countFeat++;
          }
        }
        countScene++; 
      } 

      //*****************************************************************
      // // NORMALIZATION of feature matrix

      cv::Mat FeatMatreduced = FeatMat.colRange(0, 5);     


      if (DEBUG) {cout << "Before normalization " << endl; }
      vector<double> meansVectorCurrentPair = computeMean(FeatMatreduced);
      vector<double> stdVectorCurrentPair = computeStd(FeatMatreduced, meansVectorCurrentPair);
      meanNormalizationPair_currentRef.push_back(meansVectorCurrentPair);
      stdNormalizationPair_currentRef.push_back(stdVectorCurrentPair);

      vector<double> maxVectorCurrentPair = computeMax(FeatMatreduced);  
      vector<double> minVectorCurrentPair = computeMin(FeatMatreduced);
      maxtmp.push_back(maxVectorCurrentPair);
      mintmp.push_back(minVectorCurrentPair);

      // compute weight based on STD of featuers
      double weight = computeStdWeights(FeatMatreduced, maxVectorCurrentPair, minVectorCurrentPair);

      cv::Mat normalizedFeatMat;
       if (NORMALIZEPAIR == 1) {
         normalizedFeatMat = doNormalization(FeatMatreduced, meansVectorCurrentPair, stdVectorCurrentPair);
       }
       else if (NORMALIZEPAIR == 2) {
         cout << "Before normalization Min Max do Nornmalization" << endl;
         normalizedFeatMat = doNormalizationMinMax(FeatMatreduced, maxVectorCurrentPair, minVectorCurrentPair);
       } 
       else {
         normalizedFeatMat = FeatMatreduced.clone();
       }
       //****************************************************************

      if (DEBUG) {
        cout << endl << endl << "Object : " << i << "and " << j << endl << 
           "The feature matrix dim is " << normalizedFeatMat.size() << endl;
        cout << endl << endl << "The feature matrix N ROWS is " << normalizedFeatMat.rows << endl;
        cout << endl <<  "The features are " << endl <<  normalizedFeatMat << endl;
      }

      //  Training EM model for the current object.  
      cv::EM em_model(nclusters);
      cout << endl << endl << "The feature matrix N ROWS is " << normalizedFeatMat.rows << endl;


      // Constraint: trains the GMM model for object pair features ONLY if the number of samples is sufficient!
      if (FeatMat.rows > 14) {

        if (TESTFLAG) { 
          std::cout << "Training the EM model for: "  << "Objects : " << i << " and " << j << endl << std::endl; 
        }
        em_model.train ( normalizedFeatMat );    
        if (DEBUG) { 
          std::cout << "Getting the parameters of the learned GMM model." << std::endl; 
        }
      }
      else {
          std::cout << "NOT Training the EM model for: "  << "Objects : " << i << " and " << j << endl << std::endl; 
      }
      internalEMvector.push_back(em_model);
    }
    learnedModelPairObject.push_back(internalEMvector); 
    meanNormalizationPair.push_back(meanNormalizationPair_currentRef);
    stdNormalizationPair.push_back(stdNormalizationPair_currentRef);
    minFeatPair.push_back(mintmp);
    maxFeatPair.push_back(maxtmp);
  }
}

//****************************************************************************
void DatabaseInformation::computeObjectFrequencies() {
  if (TESTFLAG) {
    cout << "Inside DatabaseInformation::computeObjectFrequencies() start" << endl;
  }
  objectFrequencies.reserve(NOBJECTCLASSES);
  int nScenes = sceneList.size();
  // initialize matrix with all values == 0
  for (int i = 0; i < NOBJECTCLASSES; i++) {
    vector<int> frequenciesCurrentObject;
    for (int j = 0; j < nScenes; j++) {
      frequenciesCurrentObject.push_back(0);
    }
    objectFrequencies.push_back(frequenciesCurrentObject);
  }
  int countScene = 0;

  if (TESTFLAG) {
    cout << "Inside DatabaseInformation::computeObjectFrequencies() 0" << endl;
  }

  // for everyScene in the database
  for (vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
    vector<Object> currentSceneObjects = (*it).getObjectList(); 
    // for every object instance in the current scene     
    for (vector<Object>::iterator it2 = currentSceneObjects.begin(); it2 != currentSceneObjects.end(); ++it2) {
      int currentObjectID = (*it2).getActualObjectID();
      if (currentObjectID != -1) {
        objectFrequencies[currentObjectID][countScene] += 1; 
      }
    } 
    countScene++;
  }
  
  if (DEBUG) {
    for (int i = 0; i < NOBJECTCLASSES; i++) {
      cout << endl;
      for (int j = 0; j < nScenes; j++) {
        cout << objectFrequencies[i][j] << "     ";
      }
    }
  }

  if (TESTFLAG) {
    cout << "Inside DatabaseInformation::computeObjectFrequencies() 1" << endl;
  }

  countObjectFrequencies.reserve(NOBJECTCLASSES);
  for (int i = 0; i < objectFrequencies.size(); i++) {
    countObjectFrequencies.push_back(0);
  }  
  for (int i = 0; i < objectFrequencies.size(); i++) {
    for (int j = 0; j < objectFrequencies[i].size() ; j++) {
      if (objectFrequencies[i][j] > 0) {
        countObjectFrequencies[i] ++;
      }  
    }
  }

  countObjectFrequencies1.reserve(NOBJECTCLASSES);
  for (int i = 0; i < objectFrequencies.size(); i++) {
    countObjectFrequencies1.push_back(0);
  }  
  for (int i = 0; i < objectFrequencies.size(); i++) {
    for (int j = 0; j < objectFrequencies[i].size() ; j++) {
      if (objectFrequencies[i][j] > 0) {
        countObjectFrequencies1[i] += objectFrequencies[i][j] ;
      }  
    }

  }

  
  // compute the frequency of the joint object presence, CO-OCCURENCE, for the different object pairs.
 
  objectPairFrequencies.reserve(NOBJECTCLASSES);
  // initialize matrix with all values == 0
  for (int i = 0; i < NOBJECTCLASSES; i++) {
    vector<vector<int> > frequenciesCurrentObjectRef;
    for (int j = 0; j < NOBJECTCLASSES; j++) {
      vector<int> frequenciesCurrentPair;
      for (int k = 0; k < nScenes; k++) {
        frequenciesCurrentPair.push_back(0);
      }
      frequenciesCurrentObjectRef.push_back(frequenciesCurrentPair);
    }
    objectPairFrequencies.push_back(frequenciesCurrentObjectRef);
  }

  // for everyScene in the database, In each scene (the 3rd matrix dimension) 
  // indicate for each pair of objects if they both apprear (1) or not (0)
  for (int i = 0; i < nScenes; i++) {
    for (int j = 0; j < NOBJECTCLASSES; j++) {
      for (int k = 0; k < NOBJECTCLASSES; k++) {
        if ( (objectFrequencies[j][i] > 0 ) && (objectFrequencies[k][i] > 0 )) { 
          objectPairFrequencies[j][k][i] = 1; 
          objectPairFrequencies[k][j][i] = 1; 
        }
      }
    } 
  }  

  // count the co-occurrence frequencies of different pairs of obj.categories in the training dataset 
  countObjectPairFrequencies.reserve(NOBJECTCLASSES);
  // inizialize to all values == 0
  for (int i = 0; i < objectPairFrequencies.size(); i++) {
    vector<int> tmp;
    for (int j = 0; j < NOBJECTCLASSES; j++) {
      tmp.push_back(0);
    }
    countObjectPairFrequencies.push_back(tmp);
  }  
  // fills in the co-occurrence count iterating over all the scenes of the database
  for (int i = 0; i < objectPairFrequencies.size(); i++) {
    for (int j = 0; j < objectPairFrequencies[i].size() ; j++) {
      for (int k = 0; k < objectPairFrequencies[i][j].size(); k++) {
        if (objectPairFrequencies[i][j][k] > 0) {
          countObjectPairFrequencies[i][j] += 1;
        }
      }  
    }
  }
  // prints the co-occurrence matrix
  if (TESTFLAG) {
    for (int i = 0; i < NOBJECTCLASSES; i++) {
      cout << endl;
      for (int j = 0; j < NOBJECTCLASSES; j++) {
        cout << countObjectPairFrequencies[i][j] << "     ";
      }
    }
  }

}


