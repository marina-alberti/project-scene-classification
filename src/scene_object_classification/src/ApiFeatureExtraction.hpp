#ifndef APIFEATUREXTRACTION_H
#define APIFEATUREXTRACTION_H

#include <string.h>
#include "Object.hpp"
#include "SceneInformation.hpp"
#include "ObjectFeatures.hpp"
#include "ObjectPairFeatures.hpp"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FeatureInformation.hpp"


class ApiFeatureExtraction{

private:


public:

  ApiFeatureExtraction();
  void extractFeaturesSingleObjects(SceneInformation &);
  void extractFeaturesPairObjects(SceneInformation &);
};

#endif
