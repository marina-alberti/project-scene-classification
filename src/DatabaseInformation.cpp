#include "DatabaseInformation.hpp"

#define DEBUG 0

DatabaseInformation::DatabaseInformation(vector<string> trainingFilesList) {
  numberOfScenes = 0;
  filesList = trainingFilesList;
  //dirname = inputDir;
}



void DatabaseInformation::loadAnnotationsInIDS() {

  for (int i = 0; i < filesList.size(); i++ ) {

    string filenameXML = filesList.at(i);
    if (DEBUG)  { cout << "The XML file name is: " << filenameXML << endl;    }
  
    ApiConvertKTHDB convertKTHannotation(filenameXML);
    SceneInformation currentScene;
    convertKTHannotation.parseFileXML(currentScene);
    currentScene.orderObjectList();
    sceneList.push_back(currentScene);
    numberOfScenes = numberOfScenes + 1; 
    if (DEBUG) {cout << "Added a new scene to the sceneList. " << endl; } 

  }
}


/*
This function - for each scene in "sceneList" - calls to apiFeatureExtraction
and stores the features in the vectors of features
of the scene
*/
void DatabaseInformation::callApiFeatureExtraction() {
  
  for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
    if (DEBUG) {cout << endl << endl << "Feature Extraction STARTS: "<< endl << "Selecting a new scene for feature extration (inside DatabaseInformation::callApiFeatureExtraction)" << endl; }
    
    SceneInformation currentScene = (*it);  
    ApiFeatureExtraction extractFeaturesApi; 
    extractFeaturesApi.extractFeaturesSingleObjects((*it));   // here sould be not local

    if (DEBUG) {
      vector<FeatureInformation> allFeats = (*it).getFeatureListSingleObject();
      cout << "In DBINFO callApiFE : The size of the vector of Feat of current scene is : " << 
	allFeats.size() << endl;
    } 
    if (DEBUG) {cout << "After extracting all single object features. " << endl<<endl<<endl; }
    extractFeaturesApi.extractFeaturesPairObjects((*it)); 
  }
}

/*
  This function collects all the features stored into the field "featureList" of 
  each scene "SceneInformation" into the 2d matrix "featureMatrix" data member of this class.
  For each "SceneInformation" object in sceneList:
  goes through vector<FeatureInformation> featureListSingleObject
  stores it in a row of featureMatrix
*/
void DatabaseInformation::setFeatureMatrix() {

  for(vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); ++it) {
    SceneInformation currentScene = (*it);

  /* The SingleObjectFeatures computation */
    vector<FeatureInformation> currentSceneFeaturesSingleObject =     currentScene.getFeatureListSingleObject();
    if (DEBUG) { 
      cout << "In DBINFO set FMatrix : The size of the vector of Feat SINGLE_OBJ of current scene is : " <<  currentSceneFeaturesSingleObject.size() << endl;
    }
    featureMatrixSingleObject.push_back(currentSceneFeaturesSingleObject);

    /* The pair of Objects Features Computation */
    vector <FeatureInformation> currentSceneFeaturesPairObject =     currentScene.getFeatureListPairObject();
    if (DEBUG) { 
      cout << "In DBINFO set FMatrix : The size of the vector of Feat PAIR_OBJ of current scene is : " <<  currentSceneFeaturesPairObject.size() << endl;
    }
    featureMatrixPairObject.push_back(currentSceneFeaturesPairObject);
  }
}

void DatabaseInformation::printFeatureMatrix() {
/* for each scene     */
  int Scenec = 0;
/* Printing the matrix with features from the Single Object */
  for(vector<vector<FeatureInformation> >::iterator it = featureMatrixSingleObject.begin(); it != featureMatrixSingleObject.end(); ++it) {
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
  cout << endl << endl << endl;

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
void  DatabaseInformation::computeGMM_SingleObject_SingleFeat(int nclusters) {

 // compute the number of features 
  vector<FeatureInformation> exv1 = featureMatrixSingleObject.at(1);
  int nFeats = exv1.size();
  if (DEBUG) {
    cout << endl << endl << "Starting Compute GMM for Single Objects / Single Feats " << endl << endl;
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



/* For each object - 
	for each scene -
		take all feats from the current scene relative to the current object	
		put all the values in 1 single vector (independently of feature dimensionality)	
		this vector is the Nscenes x Dim mat to train the EM model for the current object
  
  Return for each Object the mean variances and weight of the mixture component
  I will need more components WRT the Single Feature modelling of the SingleObject
  I need to normalize the features.
 */
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

    /* test: select lower dimensionality of feature matrix   */
    cv::Mat featsTrain = normalizedFeatMat.colRange(0,3);     
    if (DEBUG) {
      cout << endl << endl << "Object : " << i << endl << 
         "The feature matrix dim is " << normalizedFeatMat.size() << endl;
      cout << endl << endl << "The feature matrix N ROWS is " << normalizedFeatMat.rows << endl;
      cout << endl <<  "The features are " << endl <<  normalizedFeatMat << endl;
      // cv::Mat featsTrain = normalizedFeatMat.clone();  // col(5);
      // cout << "The feature dimensionality for training is now : " << featsTrain.size() << endl;
      // cout << "The features are " <<  featsTrain << endl;
    }
    /* End test lower feature dimensionality */

    /*  Training EM model for the current object.  */
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


/*  
*/
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








