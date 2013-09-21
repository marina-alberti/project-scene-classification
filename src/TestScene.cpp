#include "TestScene.hpp"

#define DEBUG 0
#define TESTFLAG 1
#define PI 3.14159265

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


double computeGMMProbability(cv::Mat feats, cv::Mat means, vector<cv::Mat> covs, cv::Mat weights );

TestScene::TestScene(string inputname, std::vector<cv::EM> inputmodel, vector<vector<double> > inputmeanNormalization , vector<vector<double> > inputstdNormalization , vector<cv::EM> inputmodelPair) {
    
  fileNameXML = inputname;
  learnedModelSingleObject = inputmodel;
  meanNormalization = inputmeanNormalization;
  stdNormalization = inputstdNormalization;
  learnedModelPairObject = inputmodelPair;
  orderedObjectList.reserve(N_OBJECTS);
}


  /* loadAnnotations In IDS */
void TestScene::loadAnnotation() {
 cout << "The XML file name is: " << fileNameXML << endl;  
 parseFileXML();
}


void TestScene::parseFileXML() {

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
    parseObject(it->second);
  }
}



void TestScene::parseObject(boost::property_tree::ptree & parent){
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
  newObject.setObjectName(parent.get<std::string>(TAG_NAME));

 /* Currently adds to the objectList fo TestScene only monitor keyboard and mouse */
  string currentname = parent.get<std::string>(TAG_NAME);
  const char * currentNameChar = currentname.c_str();
  if ( strcmp(currentNameChar, "Monitor") == 0 || strcmp(currentNameChar, "monitor") == 0 || strcmp(currentNameChar, "Screen") == 0  || strcmp(currentNameChar, "Keyboard") == 0 || strcmp(currentNameChar, "keyboard") == 0 || strcmp(currentNameChar, "Mouse") == 0 || strcmp(currentNameChar, "mouse") == 0 )  {
    objectList.push_back(newObject);
    orderedObjectList.push_back(newObject);
    numberOfObjects++;
    if (DEBUG) {
      cout << "Indide the parseObject function: Added object " << parent.get<std::string>(TAG_NAME) << endl;
    }
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


/* 
for each object in object list
test on the 3 models input = all the features
store probability
 */
void TestScene::predictObjectClasses() {

  int fsize = 9;
  cv::Mat feats = cv::Mat::zeros ( 1, fsize,  CV_64F );
  double prob;
  

  // for each object in the objectList
  for (int i  = 0; i < featureListSingleObject.size(); i++ ) {  
    if (TESTFLAG) {
        cout << std::endl << "Predict object class for New object in the object list : " << i << endl;
     }
    vector<double> vectorProb;
    int countModel = 0;

    /* convert the features into a cv::Mat object ("feats") for matrix multiplication  */
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
    cv::Mat normalizedFeatMat = feats.clone();

   /*  Feature matrix normalization */
   /*
    for (int c = 0 ; c < fsize ; c++ ) {
        normalizedFeatMat1.at<double>(c) = (feats.at<double>(c) - (meanNormalization[i][c]) ) / ( stdNormalization[i][c] );
      } 
   */

  /* test: reduce feat dimensionality   */
    /*
    if (TESTFLAG) {
      normalizedFeatMat = feats.col(5);
      cout << "The extracted features COLUMN of test object " << i << " is " << endl
          << normalizedFeatMat << endl << endl;
    }
    */
    
    if (DEBUG) {
      cout << "The extracted normalized features of teh test object " << i << " are : " << endl  
           << normalizedFeatMat << endl;
    }
    
    /* for each of the 3 learned GMM models ( monitor , keyboard,  mouse )  */
    for(vector<cv::EM>::iterator it2 = learnedModelSingleObject.begin(); it2 != learnedModelSingleObject.end(); ++it2) {
     
      if (DEBUG) {
        cout << std::endl << "Testing against learned GMM model : " << countModel << endl;
      }
        
      /* extract mean, cov, and weight coefficients for current GMM model  */
      cv::Mat _means = (*it2).get<cv::Mat>("means"); 			//  dims x nclusters
      cv::Mat _weights = (*it2).get<cv::Mat> ("weights");  		//  nclusters x 1
      vector<cv::Mat> _covs = (*it2).get<vector<cv::Mat> > ("covs");    // nclusters x dims x dims
      if (DEBUG) {
        cout << "The mean matrix of current GMM model is : "  << _means << endl;
        cout << "The weights of current GMM model are : "  << _weights << endl;
      }
      
      /* Compute probability for the current GMM model: */

      // Using multivariate normal distribution computation
      /*
      prob = computeGMMProbability(normalizedFeatMat, _means, _covs, _weights);   // to do: put back
      if (TESTFLAG) {
        cout << " Compute likelihood with function Multivariate Normal Distribution for class : " 
		<< countModel << endl << "   Prob =  " << prob << endl;
      }
      */
       
      // Using the OpenCV EM function : "predict()"
      cv::Mat Out;  
      cv::Vec2d vec = (*it2).predict(normalizedFeatMat, Out);
      if (TESTFLAG) {
        cout  << " Compute Likelihood (Opencv predict) for class " << countModel << " :  "   << vec(0) << endl;
      }
      prob = (double)vec(0);

      // push back probability of current GMM model into vector of probabilities for current object
      vectorProb.push_back(prob);
      if (DEBUG) {
        cout << "   After computing likelihood with predict: " << countModel << endl
              << "Prob = " << prob << endl;
      }
      countModel++;
    }     
    if (DEBUG) {
      cout << std::endl << " End Predict object class for object in the object list : " << i << 
      endl  << endl;;
    }
 
   /* compute the class which matches the object with max probability for current object */
    double maxProb = -pow(10, 100);
    int indexClass = -1;
    for (int c = 0; c < vectorProb.size(); c++) {
      if (vectorProb.at(c) > maxProb) {
        maxProb = vectorProb.at(c); 
        indexClass = c;
      }
    }
    predictedClasses.push_back(indexClass);
    orderedObjectList.at(indexClass) = objectList.at(i);
    if (TESTFLAG) { cout << "Predicted class is : " << indexClass << " forn the obj " << i << endl;  }

  }
  if (DEBUG) {
    cout << std::endl << " End match objects  "<< endl;
  }
}



/*
For each object (i) in the scene:
 - Iterates over all the other objects and for each other object: 
     -  takes this object (j)
     - computes feature for object pair (i, j)
     - adds the features to the vector<features> of the scene "inputScene".
*/
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

        for (int i = 0 ; i < _features.size() ; i++ ) {
          featureListPairObject.push_back(_features.at(i));
        }
 
        if (TESTFLAG) { 
          cout << "The size of the features of the current object pair is : " 
               << _features.size() << endl;
         
        }

      }
      if (TESTFLAG) { 
        cout << "The size of the features of the TOTAL object pair is : " 
               << featureListPairObject.size() << endl;
         
      } 
    }
  }
}


/* 
For each of the learned probability models 
for the PairsOfObjects - considering 1 feature at a time 
compute the probability that the extracted feats 
follow the (corresponding) learned GMM distributions
*/
double TestScene::computeProbObjectPairs() {
  vector<double> vectorProb;
  double prob;
  double probFunction;
  double sumProb = 0;
  double productProb = 1;
  int i = 0;
  if (TESTFLAG) { 
    cout << " The size of the learned model pair object is :  " << learnedModelPairObject.size()
	 << endl;
  }
  /* for each of the learned GMM models */
  for(vector<cv::EM>::iterator it = learnedModelPairObject.begin(); it != learnedModelPairObject.end(); ++it) {
  
    if (TESTFLAG) {
        cout << std::endl << "Predict object class for New Object-Pair Feat : " << i << endl;
    }

    /* extract mean, cov, and weight coefficients for current GMM model  */
    cv::Mat _means = (*it).get<cv::Mat>("means"); 			//  dims x nclusters
    cv::Mat _weights = (*it).get<cv::Mat> ("weights");  		//  nclusters x 1
    vector<cv::Mat> _covs = (*it).get<vector<cv::Mat> > ("covs");       // nclusters x dims x dims     
    
    vector<float> allFeatValues = (featureListPairObject[i]).getAllValues();   
    float feat = allFeatValues.at(0); 
    cv::Mat feats(1, 1,  CV_64F); 
    feats = (double)feat;
    if (TESTFLAG) { 
      cout << "The features are : " << feats << " For the GMM : " << i << endl;  
    }
    cv::Mat Out;  
    cv::Vec2d vec = (*it).predict(feats, Out);
    if (TESTFLAG) {
      cout  << " Compute Likelihood (Opencv predict) for class " << i << " :  "   << vec(0) << endl;
    }
    prob = (double)vec(0);
    sumProb += prob;

    probFunction = computeGMMProbability(feats, _means, _covs, _weights);
    if (TESTFLAG) {
      cout << " Compute likelihood with function Multivariate Normal Distribution for class : " 
	<< i << " : " << probFunction << endl;
    }
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


/* 
compute the probability value of a GMM (multivariate normal distribution) 
given means, covariance matrices, micture weights
and the input feature vector = test sample
*/
double computeGMMProbability(cv::Mat feats, cv::Mat _means, vector<cv::Mat> _covs, cv::Mat _weights) {
  int nclusters = _weights.cols;
  int dim = feats.cols;
  if (DEBUG) {
    cout << endl << endl << "The number of mixture components is : " << nclusters << endl;
      std::cout << "The size of the means is:  " << _means.size()  << endl <<
      "  and weights : " << _weights.size() << std::endl <<  
      "  and feats : " << feats.size() << std::endl << 
      "  and covs : " << _covs.size() << std::endl ;
  }
  double totProb = 0;

  for (int i = 0; i < nclusters; i++) { 
    if (DEBUG) { 
      std::cout<< std::endl << "The current cluster is :  "  << i << std::endl; 
    }
    cv::Mat current_mean = _means.row(i);
    double current_weight = _weights.at<double>(i);
    cv::Mat current_cov = _covs.at(i);

    int dim = current_mean.cols;
    if (DEBUG) { 
      std::cout << "The current mean is : " << current_mean <<  std::endl ; 
      std::cout << "The current covariance matrix is : " << current_cov <<  std::endl ; 
    }

    //  compute the determinant of standard deviation matrix
    double detCov = determinant(current_cov);
    if (DEBUG) { 
      std::cout << "The determinant of the covariance matrix is : " << detCov <<  std::endl ; 
    }

    //  compute the first term of the multivariate normal distribution
    double term1_ = sqrt ( pow ( (2*PI) , (dim) ) *  detCov  ) ; 
    double term1 = 1 / term1_; 
    if (DEBUG) { 
      std::cout << "The  first term is : " << term1 <<  std::endl ; 
    }
    cv::Mat test_samplet = feats; //.t();
    // compute the exponential term
    cv::Mat terma = (test_samplet - current_mean);
    cv::Mat termb = ((current_cov)).inv();   
    cv::Mat termc = (test_samplet - current_mean).t();
    if (DEBUG) {
      std::cout << "Term 1 is : " << terma <<  std::endl ;
      std::cout << "Term 2 is : " << termb <<  std::endl ;
      std::cout << "Term 3 is : " << termc <<  std::endl ; 
    }
    cv::Mat termexp =  - 0.5 * terma * termb * termc;
    if (DEBUG) {    
      std::cout << "The  exponential term is : " << termexp <<  std::endl ; 
    }
    double termExp = termexp.at<double>(0);
    double term2 = exp(termExp);
    double probCluster = term1 * term2;
    if (DEBUG) { 
      std::cout << "The final probability is : " << probCluster << std::endl; 
    }
    totProb += probCluster * current_weight;
  }

  return totProb;
}







