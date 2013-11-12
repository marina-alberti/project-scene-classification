#include "utils.hpp"

/* This function computes the mean per feature from a feature matrix N_samples x N_features */
vector<double> computeMean(cv::Mat & FeatMat) {
  vector<double> meansVector;
  // for each column i.e. each 1-D feature
  for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
    double s = 0;
    for (int irow = 0; irow < FeatMat.rows ; irow++ ) {
      s += FeatMat.at<double>(irow, icolumn);
    }
    // compute mean
    double _mean = s / FeatMat.rows; 
    // store the mean and std values for the current object class and the current feature
    meansVector.push_back(_mean);
  }
  return meansVector;
}


/* This function computes the std per feature from a feature matrix N_samples x N_features */
vector<double> computeStd(cv::Mat & FeatMat, vector<double> meansVector) {
  vector<double> stdVector; 
  for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
    double s_std = 0;

    for (int irow = 0; irow < FeatMat.rows ; irow++ ) {
      s_std += pow( ( FeatMat.at<double>(irow, icolumn) - meansVector[icolumn]) , 2);
    }   
    double _std =sqrt(s_std / FeatMat.rows );   
    if (_std == 0) { _std = 1; }
    stdVector.push_back(_std);
  }
  return stdVector;
}


/* This function does feature matrix normalization */
cv::Mat doNormalization(cv::Mat & FeatMat, vector<double> meansVector, vector<double> stdVector) {
  cv::Mat normalizedFeatMat = FeatMat.clone();
  // normalize the current column (i.e. feature) of the feature matrix
  for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
    for (int irow = 0 ; irow < normalizedFeatMat.rows ; irow++ ) {
      normalizedFeatMat.at<double>(irow, icolumn) = (normalizedFeatMat.at<double>(irow, icolumn) - meansVector[icolumn]) / (stdVector[icolumn] );
    }
  }
  return normalizedFeatMat;
}


vector<double> computeMin(cv::Mat FeatMat) {
  vector<double> minvector;
  if (FeatMat.rows > 0) {
    for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
      double mymin = FeatMat.at<double>(0, icolumn);
      for (int irow = 1; irow < FeatMat.rows ; irow++ ) {
        if( FeatMat.at<double>(irow, icolumn) < mymin) {
          mymin = FeatMat.at<double>(irow, icolumn);
        }
      }
      minvector.push_back(mymin);
    }
  }
  return minvector;
}

vector<double> computeMax(cv::Mat FeatMat) {
  vector<double> maxvector;
  if (FeatMat.rows > 0) {
    for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
    
      double mymax = FeatMat.at<double>(0, icolumn);
      for (int irow = 1; irow < FeatMat.rows ; irow++ ) {
        if( FeatMat.at<double>(irow, icolumn) > mymax ) {
          mymax = FeatMat.at<double>(irow, icolumn);
        }
      }
      maxvector.push_back(mymax);
    
    }
  }
  return maxvector;
}


// other way of normalization meaning (x - min)/(max - min)
cv::Mat doNormalizationMinMax(cv::Mat & FeatMat, vector<double> maxvector, vector<double> minvector) {

  cv::Mat normalizedFeatMat = FeatMat.clone();
  // normalize the current column (i.e. feature) of the feature matrix
  for ( int icolumn = 0 ; icolumn < FeatMat.cols; icolumn++ ) {
    for (int irow = 0 ; irow < normalizedFeatMat.rows ; irow++ ) {
      if (maxvector[icolumn] - minvector[icolumn] == 0) {
        //normalizedFeatMat.at<double>(irow, icolumn) = normalizedFeatMat.at<double>(irow, icolumn);
      }
      else {
        normalizedFeatMat.at<double>(irow, icolumn) = (normalizedFeatMat.at<double>(irow, icolumn) - minvector[icolumn]) / (maxvector[icolumn] - minvector[icolumn] );
      }
    }
  }
  return normalizedFeatMat;
}


/* 
compute the probability value of a GMM (multivariate normal distribution) 
given means, covariance matrices, mixture weights
and the input feature vector = test sample
*/
double computeGMMProbability(cv::Mat feats, cv::Mat _means, vector<cv::Mat> _covs, cv::Mat _weights) {
  int nclusters = _weights.cols;
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

    double dim = (double) current_mean.cols;
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
    double term1_ = sqrt ( pow ( (2*PI) , dim ) *  detCov  ) ; 
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
      std::cout << "x is : " << test_samplet <<  std::endl ;
      std::cout << "(x - mu) is : " << terma <<  std::endl ;
      std::cout << "cov.inv() is : " << termb <<  std::endl ;
      std::cout << "cov * cov.inv() = " << current_cov * current_cov.inv() << endl;
      std::cout << "(x - mu ^ T) is : " << termc <<  std::endl ; 
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



void evaluatePerformance(cv::Mat cMatrix) {

  // for each object class
  vector<double> precision;
  vector<double> recall;
  vector<double> fmeasure;
  for (int i = 0; i < cMatrix.rows; i++ )  {

    double _precision;
    double _recall;
    int TP = 0;
    int FP = 0;
    int FN = 0;

//    for (int j = 0; j < cMatrix.rows; j++ ) 

    TP +=  cMatrix.at<int>(i, i);
    for (int j = 0; j < cMatrix.cols; j++ ) {
      if ( j != i) {
        FN += cMatrix.at<int>(i, j);

      }
    }
    for (int j = 0; j < cMatrix.rows; j++ ) {
      if ( j != i) {
        FP += cMatrix.at<int>(j, i);
      }
    }


    if ((TP + FP) == 0) {
      _precision = 0;
    }
    else {
      _precision = ((double)TP) /(double)(TP + FP);
    }
    if ((TP + FN) == 0) {
      _recall = 0;
    }
    else {
      _recall = (double)TP / (double)(TP + FN);
    }
    double fm;
    if (_precision == 0 && _recall == 0) {
      fm = 0;
    }
    else {
      fm = 2 * _precision * _recall / (_precision + _recall );
    }
    precision.push_back(_precision);
    recall.push_back(_recall);
    fmeasure.push_back(fm);
    if (1) {
      cout << "Object " << i << endl << "Precision: " << _precision << endl
      << "recall:    " << _recall << endl << "Fmeasure:  " << fm << endl;

    }

  }


}


