#include "TestScene.hpp"
//#include "DatabaseInformation.cpp"

#define DEBUG 0
#define TESTFLAG 1


using namespace boost::property_tree;


static const std::string TAG_SCENARIO = "scenario";
static const std::string TAG_NAME = "name";
static const std::string TAG_OBJECT = "object";
static const std::string TAG_NOBJECTS = "numberOfObjects";
static const std::string TAG_ALLOBJECTS = "allObjects";
static const std::string TAG_POSE = "pose";
static const std::string TAG_DIMENSIONS = "dimensions";
static const std::string TAG_LENGTH = "length";
static const std::string TAG_WIDTH = "width";
static const std::string TAG_HEIGHT = "height";
static const std::string TAG_COLOR = "color";
static const std::string TAG_INDICES = "indices";


TestScene::TestScene(DatabaseInformation & inputDatabaseInformation) {
    
  // fileNameXML = inputname;
  /*
  learnedModelSingleObject = inputmodel;
  meanNormalization = inputmeanNormalization;
  stdNormalization = inputstdNormalization;
  learnedModelPairObject = inputmodelPair;
  countObjectFrequencies = countObjectFrequenciesin;
  countObjectFrequencies1 = countObjectFrequencies1in;
  meanNormalizationPair = meanPair;
  stdNormalizationPair = stdPair;
  countObjectPairFrequencies = countObjectPairFrequenciesin;
  maxFeatPair = maxin;
  minFeatPair = minin;
  thresholds = thre;
  */

  trainedParameters = inputDatabaseInformation;

  learnedModelSingleObject = trainedParameters.getLearnedModelSingleObject();
  learnedModelPairObject = trainedParameters.getLearnedModelPairObject();
  meanNormalization = trainedParameters.getmeanNormalization();
  stdNormalization = trainedParameters.getstdNormalization();
  countObjectFrequencies = trainedParameters.getObjectFrequencies();
  countObjectFrequencies1 = trainedParameters.getObjectFrequencies1();
  countObjectPairFrequencies = trainedParameters.getObjectPairFrequencies();
  meanNormalizationPair = trainedParameters.getmeanNormalizationPair();
  stdNormalizationPair = trainedParameters.getstdNormalizationPair();
  maxFeatPair =  trainedParameters.getmaxFeatPair();
  minFeatPair =  trainedParameters.getminFeatPair();
  thresholds = trainedParameters.getThresholds();

  removeID = -1;
}


  /* loadAnnotations In IDS */
void TestScene::loadAnnotation(string inputname, bool removeObject) {

  fileNameXML = inputname;
  cout << "The XML file name is: " << fileNameXML << endl;  
  if (removeObject) {
    removeID = REMOVEID;  // rand() % N_OBJECTS;
  }
  parseFileXML(removeObject);

}


void TestScene::parseFileXML(bool removeObject) {

  cout << "The filename of the test scene is : " << fileNameXML << endl;
  boost::property_tree::ptree root;
  read_xml(fileNameXML, root);

  std::string scenarioType = root.get<std::string>(TAG_SCENARIO + "." + "type");
  // mySceneInformation.setType(scenarioType);

  ptree& tableDimensions = root.get_child(TAG_SCENARIO + "." + TAG_DIMENSIONS);
  deskLength = tableDimensions.get<float>(TAG_LENGTH);
  deskWidth = tableDimensions.get<float>(TAG_WIDTH);
  setDeskCentroid(); 

  if (DEBUG) {
    cout << "Parsing the XML file. The Scene Desk parameters have been set to :" << endl
         << "Length = " << deskLength << endl
         << "Width = " <<  deskWidth << endl;
  } 

  boost::property_tree::ptree& allObjects = root.get_child(TAG_SCENARIO + "." + TAG_ALLOBJECTS);
  boost::property_tree::ptree::iterator it = allObjects.begin();
  
  it++;

  for(; it != allObjects.end(); it++){
    if (DEBUG) {   cout << "New object will be parsed" << endl;     }
    parseObject(it->second, removeObject);
  }
}



void TestScene::parseObject(boost::property_tree::ptree & parent, bool removeObject) {
  Object newObject;
  if (DEBUG) {
  cout << "Indide the parseObject function" << endl;
  }
  float x, y, z, roll, pitch, yaw, length, width, height;
  boost::property_tree::ptree& pose = parent.get_child(TAG_POSE);
  x = pose.get<float>("x");
  y = pose.get<float>("y");
  z = pose.get<float>("z");
  roll = pose.get<float>("roll");
  pitch = pose.get<float>("pitch");
  yaw = pose.get<float>("yaw");
  boost::property_tree::ptree& dimensions = parent.get_child(TAG_DIMENSIONS);
  length = dimensions.get<float>(TAG_LENGTH);
  width = dimensions.get<float>(TAG_WIDTH);
  height = dimensions.get<float>(TAG_HEIGHT);
  if (DEBUG) {
    cout << "The parameters are: " << x << " " << y << " " << z << " " << 
        roll << " " << pitch << " " << yaw << " " << length << " " << width 
       << " " << height << endl;
  }
  setmapKTHparameters(x, y, z, roll, pitch, yaw, length, width, height);
  if (DEBUG) {
  cout << "Indide the parseObject function: saved KTH parameters inside map." << endl;
  }
  vector<pcl::PointXYZ> myboundingBoxPoints = convertObjectParameters();
  newObject.setBoundingBox(myboundingBoxPoints); 

  // "setObjectName" will also set object actual ID "actualObjectID" field of Object 
  newObject.setObjectName(parent.get<std::string>(TAG_NAME));

    // if I randomly remove 1 object from the test scene:
    if (removeObject ) {
      if (TESTFLAG)  {
        cout << "Remove object ID is: " << removeID << endl;
      }
      if ( newObject.getActualObjectID() ==  removeID) {}
      else {
        objectList.push_back(newObject);
        numberOfObjects++;
        if (TESTFLAG) {
          cout << "Adding object ID = " << newObject.getActualObjectID() << endl;
        }
      }
    }
    // if I do not remove any object (the standard behaviour)
    else {
      objectList.push_back(newObject);
      numberOfObjects++;
    }
}



vector<pcl::PointXYZ> TestScene::convertObjectParameters(){
  if (DEBUG) {
  cout << "Indide the convertObjectParameters function" << endl;
  }
  float x = mapKTHparameters.find("x")->second;
  float y = mapKTHparameters.find("y")->second;
  float z = mapKTHparameters.find("z")->second;
  float roll = mapKTHparameters.find("roll")->second;
  float pitch = mapKTHparameters.find("pitch")->second;
  float yaw = mapKTHparameters.find("yaw")->second;
  float length = mapKTHparameters.find("length")->second;
  float width = mapKTHparameters.find("width")->second;
  float height = mapKTHparameters.find("height")->second;
  // work on a point cloud
  //pcl::PointCloud<pcl::PointXYZ> myBoundingBox;
  Eigen::Affine3f myTransformation;
  myTransformation = pcl::getTransformation(x,y,z,roll,pitch,yaw);
  pcl::PointXYZ _FLDPoint, _FRDPoint, _BLDPoint, _BRDPoint, _FLUPoint, _FRUPoint, _BLUPoint, _BRUPoint;

  // Lower face vertices
  // Front face, left lower point
  _FLDPoint.x = 0;
  _FLDPoint.y = 0;
  _FLDPoint.z = 0;
  _FLDPoint = pcl::transformPoint(_FLDPoint, myTransformation);
   // Front face, right lower point
  _FRDPoint.x = length;
  _FRDPoint.y = 0;
  _FRDPoint.z = 0;
  _FRDPoint = pcl::transformPoint(_FRDPoint, myTransformation);
  // Back face, left lower point
  _BLDPoint.x = length;
  _BLDPoint.y = width;
  _BLDPoint.z = 0;
  _BLDPoint = pcl::transformPoint(_BLDPoint, myTransformation);
  // Left face, right lower point
  _BRDPoint.x = 0;
  _BRDPoint.y = width;
  _BRDPoint.z = 0;
  _BRDPoint = pcl::transformPoint(_BRDPoint, myTransformation);

  // Upper face vertices
  // Front face, left upper point
  _FLUPoint.x = 0;
  _FLUPoint.y = 0;
  _FLUPoint.z = height;
  _FLUPoint = pcl::transformPoint(_FLUPoint, myTransformation);
   // Front face, right upper point
  _FRUPoint.x = length;
  _FRUPoint.y = 0;
  _FRUPoint.z = height;
  _FRUPoint = pcl::transformPoint(_FRUPoint, myTransformation);
  // Back face, left upper point
  _BLUPoint.x = length;
  _BLUPoint.y = width;
  _BLUPoint.z = height;
  _BLUPoint = pcl::transformPoint(_BLUPoint, myTransformation);
  // Left face, right upper point
  _BRUPoint.x = 0;
  _BRUPoint.y = width;
  _BRUPoint.z = height;
  _BRUPoint = pcl::transformPoint(_BRUPoint, myTransformation);

  // // myBoundingBox->points[0].x = 0.0; boundingBox->points[0].y = 0.0; boundingBox->points[0].z = 0.0;
 // // myBoundingBox.push_back(_FLDPoint);
 
  vector<pcl::PointXYZ> out;
  out.push_back(_FLDPoint);
  out.push_back(_FRDPoint);
  out.push_back(_BLDPoint);
  out.push_back(_BRDPoint);
  out.push_back(_FLUPoint);
  out.push_back(_FRUPoint);
  out.push_back(_BLUPoint);
  out.push_back(_BRUPoint);

  if (DEBUG) {
  cout << "Indide the convertObjectParameters function: after pushing back ALL points into the vector of points " << endl;
  }
  return out;
}

void TestScene::setmapKTHparameters(float x, float y, float z, float roll, float pitch, float yaw, float length, float width, float height) {
  mapKTHparameters["x"] = x;
  mapKTHparameters["y"] = y;
  mapKTHparameters["z"] = z;
  mapKTHparameters["roll"] = roll;
  mapKTHparameters["pitch"] = pitch;
  mapKTHparameters["yaw"] = yaw;
  mapKTHparameters["length"] = length;
  mapKTHparameters["height"] = height;
  mapKTHparameters["width"] = width;
}


void TestScene::setDeskCentroid() { 
  deskCentroid.x = deskLength / 2;
  deskCentroid.y = deskWidth / 2;
  deskCentroid.z = 0;
}


void TestScene::extractFeatures() {

  /* For each object */
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
    featureListSingleObject.push_back(singleObjectFeatures);
  }
}



void TestScene::loadAnnotationServiceFormat(vector<string> type, vector<string> object_id, vector<vector<pcl::PointXYZ> > bbox) {

  // for each object in the test scene
  for (vector<vector<pcl::PointXYZ> >::iterator it = bbox.begin(); it != bbox.end(); ++it) {

    Object newObject;

    vector<pcl::PointXYZ> bboxCurrentObject = *it;
    newObject.setBoundingBox(bboxCurrentObject); 
    // "setObjectName" will also set object actual ID "actualObjectID" field of Object 
    // newObject.setObjectName(parent.get<std::string>(TAG_NAME));

    objectList.push_back(newObject);
    numberOfObjects++;

  }

}


/* 
for each object in object list
test on the 3 models input = all the features
store probability
*/
vector<vector<double> > TestScene::predictObjectClasses() {

  int fsize = 9;
  cv::Mat feats = cv::Mat::zeros ( 1, fsize,  CV_64F );
  double prob;
  cv::Mat normalizedFeatMat;
  vector<vector<double> > vectorProbabilitiesObjects;


  // for each object in the objectList
  for (int i  = 0; i < featureListSingleObject.size(); i++ ) {  
    if (TESTFLAG) {
        cout << std::endl << "Predict object class for a new unknown object in the object list : " << i << endl;
     }
    // vectorProb: a vector with a likelihood value for each of the considered object categories 
    //  against which the test object is matched
    vector<double> vectorProb;
    double maxProbValue;
    int maxProbClassIndex;
    int countModel = 0;

    // convert the features into a cv::Mat object ("feats") for matrix multiplication  
    int countFeats = 0;    
    for ( int j = 0; j < featureListSingleObject[0].size() ; j++ ) { 
      vector<float> allFeatValues = (featureListSingleObject[i][j]).getAllValues();
      for (int k = 0; k < allFeatValues.size() ; k++ ) { 
        feats.at<double>(countFeats) = (double) allFeatValues[k];   // 1 x dims
        countFeats++; 
      }   
    } 
    if (DEBUG) { 
      cout << endl << endl << "The extracted features of test object " << i << " are : " << endl
          << feats << endl << endl;
    }
          
    // for each of the  learned GMM models ( monitor, keyboard,  mouse, ...) 
    for(vector<cv::EM>::iterator it2 = learnedModelSingleObject.begin(); it2 != learnedModelSingleObject.end(); ++it2) {
     
      bool outlier = false;

      /* extract mean, cov, and weight coefficients for current GMM model  */
      cv::Mat _means = (*it2).get<cv::Mat>("means"); 			//  dims x nclusters
      cv::Mat _weights = (*it2).get<cv::Mat> ("weights");  		//  nclusters x 1
      vector<cv::Mat> _covs = (*it2).get<vector<cv::Mat> > ("covs");    // nclusters x dims x dims
      if (DEBUG) {
        cout << endl << "The mean matrix of current GMM model is : "  << _means << endl;
        cout << "The weights of current GMM model are : "  << _weights << endl;
      }
      
      // **************************************************************************************
      // //  NORMALIZATION:   Feature matrix normalization 
      
      //  normalizedFeatMat = doNormalization(feats, meanNormalization[countModel], stdNormalization[countModel]);
      normalizedFeatMat = feats.clone();
      // **************************************************************************************

      // // test: reduce feat dimensionality    
      cv::Mat featsTrain = normalizedFeatMat.colRange(0, 9);     
      /*
      cv::Mat featsTrain = cv::Mat(normalizedFeatMat.rows, 4 , CV_64F);   
      normalizedFeatMat.col(0).copyTo(featsTrain.col(0));
      normalizedFeatMat.col(1).copyTo(featsTrain.col(1));
      normalizedFeatMat.col(3).copyTo(featsTrain.col(2));
      normalizedFeatMat.col(4).copyTo(featsTrain.col(3));
      */
      if (DEBUG) {
        cout << "The extracted features COLUMN SUBSET of test object " << i << " is " << endl
            << featsTrain << endl << feats << endl;
      }

      //****************************************************************************************
      /* Compute probability for the current GMM model: */
      // Using multivariate normal distribution computation
      prob = computeGMMProbability(featsTrain, _means, _covs, _weights);
      prob = log(prob);
      if (prob < thresholds[countModel]) {
        // outlier = true;
      }

      // ***********************************************************************************
      // compute a-priori probablity of object classes in terms of frequency of appearence
      int currentObjectCategoryFreq = countObjectFrequencies[countModel];
      double CurrentObjectCategoryProb = ((double)(currentObjectCategoryFreq))/( NSCENES + 1);
      double probPost = prob + log(CurrentObjectCategoryProb);
      // ***********************************************************************************

      if (DEBUG) {
        cout << " Likelihood Multivariate Normal Distribution for Object class : " 
		<< countModel << "  is  =  " << prob << "  with likelihood frequency  =   " << 
                (CurrentObjectCategoryProb) << endl;
      }
      
      // Using the OpenCV EM function : "predict()"
      /*
      cv::Mat Out;  
      cv::Vec2d vec = (*it2).predict(normalizedFeatMat, Out);
      if (DEBUG) {
        cout  << " Compute Likelihood (Opencv predict) for class " << countModel << " :  "   << vec(0) << endl;
      }
      prob = (double)vec(0);
      */


      if ( countModel == 0) {
        maxProbValue = -100000;
        maxProbClassIndex = -1;
      }
      
      if ( (probPost > maxProbValue) && (outlier == false)  ) {
        maxProbValue = probPost;
        maxProbClassIndex = countModel;
      }

      // push back probability of current GMM model into vector of probabilities for current object
      vectorProb.push_back(probPost);

      if (TESTFLAG) {
        cout << " Compute likelihood for object category type : " << countModel 
              << "   likelihood = " << probPost << endl;
      }
      countModel++;
    }     

    if (TESTFLAG) {
      cout << std::endl << " End Predict object class for object in the object list : " << i << 
      endl << "Prob value = " << maxProbValue << endl << "Predicted object class  " << maxProbClassIndex << endl
          << "Actual class " <<  (objectList.at(i)).getActualObjectID() << endl ;
      if ( (objectList.at(i)).getActualObjectID() != maxProbClassIndex ) {
        cout << "ERROR OBJECT CLASSIFICATION" << endl; 
      }
    }
 
    predictedClasses.push_back(maxProbClassIndex);
    (objectList.at(i)).setPredictedObjectID(maxProbClassIndex);

    vectorProbabilitiesObjects.push_back(vectorProb);
 
  }

  return vectorProbabilitiesObjects;
}


void TestScene::evaluateObjectClassificationPerformance() {

  int size = learnedModelSingleObject.size();
  cMatrix = cv::Mat::zeros(size, size+1, CV_32S);

  if (TESTFLAG) {
    cout << "The confusion matrix is : " << endl << cMatrix << endl <<" after inizialization." << endl;
  }

  int actualClass;
  int predictedClass;
  for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
    actualClass = (*it).getActualObjectID();
    predictedClass = (*it).getPredictedObjectID();
    
    if (TESTFLAG) {
      cout << "The actual class is : " << actualClass << endl <<
		" The predicted class is : " << predictedClass << endl;
    }
    if (predictedClass != -1) {
      cMatrix.at<int>(actualClass, predictedClass) ++;
    }
    else {
      cMatrix.at<int>(actualClass, (size)) ++;
    }
  }
  
  if (TESTFLAG) {
    cout << "The confusion matrix is : " << endl << cMatrix << endl <<" after the computation." << endl;
  }
    
}


/*
For each object (i) in the scene:
 - Iterates over all the other objects and for each other object: 
     -  takes this object (j)
     - computes feature for object pair (i, j)
     - adds the features to the vector<features> of the scene "inputScene".
*/
/*
void TestScene::extractFeaturesPairObjects() {

  for(vector<Object>::iterator it = orderedObjectList.begin(); it != orderedObjectList.end(); ++it) {
    Object referenceObject = *it;

    if (DEBUG) {
      cout << endl << "Adding the features of a new REFERENCE object. " << endl; 
    }
    for(vector<Object>::iterator it2 = orderedObjectList.begin(); it2 != orderedObjectList.end(); ++it2) {
      if (it2 != it) {
        Object targetObject = *it2;
        if (DEBUG) {
        cout << endl << "Adding the features of a new TARGET object. Pair is : " << 
		referenceObject.getObjectName() << ",  " << targetObject.getObjectName() << endl; 
        }
        // all feature for current object pair 
        ObjectPairFeatures pairObjectFeatureExtraction(referenceObject, targetObject);
        pairObjectFeatureExtraction.extractFeatures();
        vector<FeatureInformation> _features = pairObjectFeatureExtraction.getAllFeatures();

        //
        //for (int i = 0 ; i < _features.size() ; i++ ) {
        //  featureListPairObject.push_back(_features.at(i));
       // }
        //
        featureListPairObject.push_back(_features);
        if (TESTFLAG) { 
          cout << "The size of the features of the current object pair is : " 
               << _features.size() << endl;
        }
      }

    }
    if (TESTFLAG) { 
      cout << "The size of the features of the TOTAL featureListPairObject is : " 
               << featureListPairObject.size() << endl;
    } 
  }
}
*/


/*
For each object (i) in the scene:
 - Iterates over all the other objects and for each other object: 
     -  takes this object (j)
     - computes feature for object pair (i, j)
     - adds the features to the vector<features> of the scene "inputScene".
*/
void TestScene::extractFeaturesPairObjects_HandleMissing() {

  for(vector<Object>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
    Object referenceObject = *it;
//    int referenceObjectID = referenceObject.getActualObjectID();

    if (DEBUG) {
      cout << endl << "Adding the features of a new REFERENCE object. " << endl; 
    }
    for(vector<Object>::iterator it2 = objectList.begin(); it2 != objectList.end(); ++it2) {
    
      if (it2 != it) {
        Object targetObject = *it2;
  //      int targetObjectID = targetObject.getActualObjectID();

        if (DEBUG) {
        cout << endl << "Adding the features of a new TARGET object. Pair is : " << 
		referenceObject.getObjectName() << ",  " << targetObject.getObjectName() << endl; 
        }
        // all feature for current object pair 
        ObjectPairFeatures pairObjectFeatureExtraction(referenceObject, targetObject);
        pairObjectFeatureExtraction.extractFeatures();
        vector<FeatureInformation> _features = pairObjectFeatureExtraction.getAllFeatures();

        /*
        for (int i = 0 ; i < _features.size() ; i++ ) {
          featureListPairObject.push_back(_features.at(i));
        }
        */

        /* Check the object class IDs of the reference object and target object
        and accordingly set ID of the current Object-Pair */
        int referenceID = (*it).getPredictedObjectID(); 
        int targetID = (*it2).getPredictedObjectID();

        // compute only features between pairs of different object classes
        if (referenceID != targetID) {
      
          //int currentObjectPairID = setPairID(referenceID, targetID); // 0 to 5
          //objectPairID.push_back(currentObjectPairID);

          AllFeatPairObject currentObjectPairFeatures;
          currentObjectPairFeatures.setObjectID1(referenceID);
          currentObjectPairFeatures.setObjectID2(targetID);
          for (int i = 0; i <_features.size(); i++) {
            currentObjectPairFeatures.addFeature(_features.at(i));
          }
          featureListPair.push_back(currentObjectPairFeatures);

          if (TESTFLAG) { 
            cout << "The size of the features of the current object pair is : " 
               << _features.size() << endl;
            //cout << "The current Object-Pair ID is " << currentObjectPairID <<  endl;
          }
        }
      }

    }
    if (TESTFLAG) { 
      cout << "The size of the features of the TOTAL featureListPairObject is : " 
               << featureListPair.size() << endl;
    } 
  }
}



/* 
SINGLE FEATURES
For each of the learned probability models 
for the PairsOfObjects - considering 1 feature at a time 
compute the probability that the extracted feats 
follow the (corresponding) learned GMM distributions
*/

/*
double TestScene::computeProbObjectPairs() {
  vector<double> vectorProb;
  double prob;
  double probFunction;
  double sumProb = 0;
  double productProb = 1;
  int i = 0;
  if (DEBUG) { 
    cout << " The size of the learned model pair object is :  " << learnedModelPairObject.size()
	 << endl;
  }

  // for each of the learned GMM models for Pairs-Of-Objects here <N_Pairs * N_Feats> 

  for(vector<cv::EM>::iterator it = learnedModelPairObject.begin(); it != learnedModelPairObject.end(); ++it) {
  
    if (DEBUG) {
        cout << std::endl << "Predict object class for New Object-Pair Features : " << i << endl;
    }

    // extract mean, cov, and weight coefficients for current GMM model  
    cv::Mat _means = (*it).get<cv::Mat>("means"); 			//  dims x nclusters
    cv::Mat _weights = (*it).get<cv::Mat> ("weights");  		//  nclusters x 1
    vector<cv::Mat> _covs = (*it).get<vector<cv::Mat> > ("covs");       // nclusters x dims x dims     
    
    vector<float> allFeatValues = (featureListPairObject[i]).getAllValues();   
    float feat = allFeatValues.at(0); 
    cv::Mat feats(1, 1,  CV_64F); 
    feats = (double)feat;
    if (DEBUG) { 
      cout << "The features are : " << feats << " For the GMM : " << i << endl;  
    }
    // Computing likelihood with OpenCV predict function 
    cv::Mat Out;  
    cv::Vec2d vec = (*it).predict(feats, Out);
    if (TESTFLAG) {
      cout  << " Compute Likelihood (Opencv predict) for class " << i << " :  "   << vec(0) << endl;
    }
    prob = (double)vec(0);
    // The likelihood of scene is the sum of likelihoods of all SR 
    sumProb += prob;


    // Computing probability with developed Multivariate Normal distribution function 
    probFunction = computeGMMProbability(feats, _means, _covs, _weights);
    if (TESTFLAG) {
      cout << " Compute likelihood with function Multivariate Normal Distribution for class : " 
	<< i << " : " << probFunction << endl;
    }
    // the probability of Scene is the product of probability of each SR 
    productProb = productProb * probFunction;


    // push back probability of current GMM model into vector of probabilities for current object
    vectorProb.push_back(prob);
    i++;

  }
  if (TESTFLAG) {
    cout << "The total sum of likelihoods is : " << sumProb << endl;
    cout << "The total product of probabilities is : " << productProb << endl;
  }
  totalSceneLogP = sumProb;
  return sumProb;
}

*/





/*
double TestScene::computeProbObjectPairs_AllFeats_old() {

  double prob;
  double sumProb = 0;
  int i = 0;   // iterator over the Pairs-Of-Object models
  if (TESTFLAG) { 
    cout << " The size of the learned model pair object is :  " << learnedModelPairObject.size()
	 << endl;
  }

  // for each of the learned GMM models for Pairs-Of-Objects here <N_Pairs> 
  for(vector<cv::EM>::iterator it = learnedModelPairObject.begin(); it != learnedModelPairObject.end(); ++it) {
  
    if (TESTFLAG) {
        cout << std::endl << "Predict prob for New Object-Pair Features : " << i << endl;
    }

    // extract mean, cov, and weight coefficients for current GMM model  
    cv::Mat _means = (*it).get<cv::Mat>("means"); 			//  dims x nclusters
    cv::Mat _weights = (*it).get<cv::Mat> ("weights");  		//  nclusters x 1
    vector<cv::Mat> _covs = (*it).get<vector<cv::Mat> >("covs");       // nclusters x dims x dims     
    cv::Mat features(1, (featureListPairObject[i]).size(),  CV_64F); 

    int countFeat = 0;
    vector <FeatureInformation>  featureCurrentObjectPair = featureListPairObject[i]; 
    for (vector<FeatureInformation>::iterator it2 = featureCurrentObjectPair.begin(); it2 != featureCurrentObjectPair.end(); ++it2 )  {
      vector<float> allFeatValues = (*it2).getAllValues();   
      double feat = (double)allFeatValues.at(0); 
      features.at<double>( countFeat ) = feat;
      if (DEBUG) { 
        cout << "Feature : " << i <<  "  " << countFeat << "    : " << feat << endl; 
      }
      countFeat++;
    }
    if (TESTFLAG) {
      cout << "The features are : " << endl << features << endl;
    }

    // Computing likelihood with OpenCV predict function 
    //
  //  cv::Mat Out;  
  //  cv::Vec2d vec = (*it).predict(feats, Out);
  //  if (TESTFLAG) {
  //    cout  << " Compute Likelihood (Opencv predict) for class " << i << " :  "   << vec(0) << endl;
   // }
  //  prob = (double)vec(0);
   //

    // Computing probability with developed Multivariate Normal distribution function 
    prob = computeGMMProbability(features, _means, _covs, _weights);
    prob = log(prob);
    sumProb += prob;
    if (TESTFLAG) {
      cout << " Compute likelihood with function Multivariate Normal Distribution : " << prob << endl;
    }
    i++;
  }
  if (TESTFLAG) {
    cout << "The total sum of likelihoods is : " << sumProb << endl;
  }
  totalSceneLogP = sumProb;
  return sumProb;
}

*/


/* Compute prob scene vesion 2: handles missing objects and dupliacte objects */
double TestScene::computeProbObjectPairs_AllFeats() {
  double prob;
  double sumProb = 0;

  if (TESTFLAG) { 
    cout << " The size of the learned model pair object is :  " << learnedModelPairObject.size()
	 << endl;
  }

  cv::Mat _means;
  cv::Mat _weights;
  vector<cv::Mat> _covs;

  /* For each vector_FeatureInformation = Object-Pair*/
  for (vector<AllFeatPairObject>::iterator it = featureListPair.begin(); it != featureListPair.end(); ++it) {

    int objectID1 = (*it).getObjectID1();
    int objectID2 = (*it).getObjectID2();


    if (TESTFLAG) {
        cout << std::endl << "Predict prob for New Object-Pair Features : " << objectID1 << " and " << objectID2 << endl;
    }

    if ( (learnedModelPairObject[objectID1][objectID2]).isTrained() == true ) {

      /* extract mean, cov, and weight coefficients for CORRESPONDING GMM model  */
      _means = (learnedModelPairObject[objectID1][objectID2]).get<cv::Mat>("means"); // dims x nclusters
      _weights = (learnedModelPairObject[objectID1][objectID2]).get<cv::Mat> ("weights");  //  nclusters x 1
      _covs = (learnedModelPairObject[objectID1][objectID2]).get<vector<cv::Mat> >("covs");     
 
      vector<FeatureInformation> featureCurrentObjectPair = (*it).getAllFeatures();

      cv::Mat features(1, (featureCurrentObjectPair).size(),  CV_64F);  
      int countFeat = 0;

      for (vector<FeatureInformation>::iterator it2 = featureCurrentObjectPair.begin(); it2 != featureCurrentObjectPair.end(); ++it2 )  {
        vector<float> allFeatValues = (*it2).getAllValues();   
        double feat = (double)allFeatValues.at(0); 
        features.at<double>( countFeat ) = feat;
        countFeat++;     
      }


      // *********************************************************************************
      // NORMALIZATION OF THE FEATURE MATRIX:
      cv::Mat feat;
      if (NORMALIZEPAIR) {
        feat = doNormalization(features, meanNormalizationPair[objectID1][objectID2], stdNormalizationPair[objectID1][objectID2]);
      }
      else {
        feat = features.clone();
      }
      // *********************************************************************************


      if (TESTFLAG) {
        cout << "The features are : " << endl << feat << endl;
      }

      //// Computing likelihood with OpenCV predict function 
      /*
      cv::Mat Out;  
      cv::Vec2d vec = (*it).predict(feats, Out);
      if (TESTFLAG) {
        cout  << " Compute Likelihood (Opencv predict) for class " << i << " :  "   << vec(0) << endl;
      }
      prob = (double)vec(0);
      */

      // Computing probability with developed Multivariate Normal distribution function 
      prob = computeGMMProbability(feat, _means, _covs, _weights);
      prob = log(prob);
      // product of probabilities -> sum of log(probabilities)
      sumProb += prob;
      int coOccurrenceFrequency = countObjectPairFrequencies[objectID1][objectID2];
      double coOccurrenceProb = (double(coOccurrenceFrequency))/(NSCENES + 1);
      sumProb += log(coOccurrenceProb);
      if (TESTFLAG) {
        cout << " Compute likelihood with function Multivariate Normal Distribution : " << endl << "Likelihood =  " << prob << 
        " and the co-occurrence likelihood = " << log(coOccurrenceProb) << endl;
      }
     }
    else { 
      cout << "Not trained : " <<   objectID1 << " and " << objectID2 << endl;  
    }
  }
  if (TESTFLAG) {
    cout << "The total sum of likelihoods is : " << sumProb << endl;
  }
  totalSceneLogP = sumProb;
    if (TESTFLAG) {
    cout << "In TestScene::computeProbObjectPairs_AllFeats() before return " << endl;
  }
  return sumProb;
}


/* New strategy for simlarity score computation */
double TestScene::computeSimilarityScore() {

  double prob;
  double sumProb = 0;
  double prodProb = 1;
  if (TESTFLAG) { 
    cout << " The size of the learned model pair object is :  " << learnedModelPairObject.size()
	 << endl;
  }

  cv::Mat _means;
  cv::Mat _weights;
  vector<cv::Mat> _covs;

  /* For each vector_FeatureInformation = Object-Pair*/
  for (vector<AllFeatPairObject>::iterator it = featureListPair.begin(); it != featureListPair.end(); ++it) {

    int objectID1 = (*it).getObjectID1();
    int objectID2 = (*it).getObjectID2();


    if (TESTFLAG) {
        cout << std::endl << "Predict prob for New Object-Pair Features : " << objectID1 << " and " << objectID2 << endl;
    }
    if ((objectID1 != -1) && (objectID2 != -1 )) {
    if (  (learnedModelPairObject[objectID1][objectID2]).isTrained() == true ) {

      /* extract mean, cov, and weight coefficients for CORRESPONDING GMM model  */
      _means = (learnedModelPairObject[objectID1][objectID2]).get<cv::Mat>("means"); // dims x nclusters
      _weights = (learnedModelPairObject[objectID1][objectID2]).get<cv::Mat> ("weights");  //  nclusters x 1
      _covs = (learnedModelPairObject[objectID1][objectID2]).get<vector<cv::Mat> >("covs");     
 
      vector<FeatureInformation> featureCurrentObjectPair = (*it).getAllFeatures();

      cv::Mat features(1, (featureCurrentObjectPair).size(),  CV_64F);  
      int countFeat = 0;

      for (vector<FeatureInformation>::iterator it2 = featureCurrentObjectPair.begin(); it2 != featureCurrentObjectPair.end(); ++it2 )  {
        vector<float> allFeatValues = (*it2).getAllValues();   
        double feat = (double)allFeatValues.at(0); 
        features.at<double>( countFeat ) = feat;
        countFeat++;     
      }
      if (TESTFLAG) {
        cout << "The features are : " << endl << features << endl;
      }

      // *********************************************************************************
      // NORMALIZATION OF THE FEATURE MATRIX:
      cv::Mat feat;
      if (NORMALIZEPAIR == 1) {
        feat = doNormalization(features, meanNormalizationPair[objectID1][objectID2], stdNormalizationPair[objectID1][objectID2]);
      }
      else if (NORMALIZEPAIR == 2) {
        feat = doNormalizationMinMax(features, maxFeatPair[objectID1][objectID2], minFeatPair[objectID1][objectID2]);
      }
      else {
        feat = features.clone();
      }
      // *********************************************************************************

      cv::Mat featsTrain = feat.colRange(0, 5);     
      
      if (TESTFLAG) {
        cout << "The features are : " << endl << featsTrain << endl;
        cout << "The GMM MEANS are : " << endl << _means << endl;
        cout << "The GMM WEIGHTS are : " << endl << _weights << endl;
      }
      //// Computing likelihood with OpenCV predict function 
      bool opencvprob = false;
      if (opencvprob) {
        cv::Mat Out;  
        cv::Vec2d vec = (learnedModelPairObject[objectID1][objectID2]).predict(featsTrain, Out);
        prob = (double)vec(0);
        prob = pow(10, prob);
      }
      else {
        prob = computeGMMProbability(featsTrain, _means, _covs, _weights);
      }

      int coOccurrenceFrequency = countObjectPairFrequencies[objectID1][objectID2];
      double coOccurrenceProb = (double(coOccurrenceFrequency))/(NSCENES + 1);

      sumProb += (prob * coOccurrenceProb);
      prodProb = prodProb *  (prob * coOccurrenceProb);

      
      if (TESTFLAG) {
        cout << "Prob / likelihood =  " << prob << endl <<
        " co-occurrence probability weight = " << coOccurrenceProb << endl <<
        " product = " << (prob * coOccurrenceProb) << endl ;
        
      }
     }
    else { 
      cout << "Not trained : " <<   objectID1 << " and " << objectID2 << endl;  
    }
    }
  }
  if (TESTFLAG) {
    cout << "The total sum of likelihoods is : " << sumProb << endl;
    cout << "The total product  likelihoods  : " << log(prodProb) << endl;
  }


  //******************************************************************************
  /*
  int fsize = 9;
  cv::Mat featSO = cv::Mat::zeros ( 1, fsize,  CV_64F );
 
  for (int i  = 0; i < featureListSingleObject.size(); i++ ) {  
      
    int objectID = (objectList[i]).getPredictedObjectID();

    // convert the features into a cv::Mat object ("feats") for matrix multiplication  
    int countFeats = 0;    
    for ( int j = 0; j < featureListSingleObject[0].size() ; j++ ) { 
      vector<float> allFeatValues = (featureListSingleObject[i][j]).getAllValues();
      for (int k = 0; k < allFeatValues.size() ; k++ ) { 
        featSO.at<double>(countFeats) = (double) allFeatValues[k];   // 1 x dims
        countFeats++; 
      }   
    } 
     
    if ( (learnedModelSingleObject[objectID]).isTrained() == true ) {

      // extract mean, cov, and weight coefficients for CORRESPONDING GMM model  
      cv::Mat _means = (learnedModelSingleObject[objectID]).get<cv::Mat>("means"); // dims x nclusters
      cv::Mat _weights = (learnedModelSingleObject[objectID]).get<cv::Mat> ("weights");  //  nclusters x 1
      vector<cv::Mat> _covs = (learnedModelSingleObject[objectID]).get<vector<cv::Mat> >("covs");  

      prob = computeGMMProbability(featSO, _means, _covs, _weights);
  
     // compute a-priori probablity of object classes in terms of frequency of appearence

      int currentObjectCategoryFreq = countObjectFrequencies[objectID];
      double CurrentObjectCategoryProb = ((double)(currentObjectCategoryFreq))/( NSCENES + 1);
      prob = prob * (CurrentObjectCategoryProb);
      sumProb += prob;

      if (TESTFLAG) {
        cout << "Prob / likelihood =  " << prob << endl <<
        " occurrence probability weight = " << CurrentObjectCategoryProb << endl <<
        " product = " << (prob * CurrentObjectCategoryProb) << endl ;
      }
    }
  }
   */

  //*****************************************************************************

  totalSceneLogP = sumProb;

  return sumProb;

}




