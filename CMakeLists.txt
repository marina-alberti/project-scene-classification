cmake_minimum_required (VERSION 2.6)

project (SceneDescription)

find_package(PCL 1.4 REQUIRED COMPONENTS visualization filters segmentation)
find_package( OpenCV REQUIRED )


include_directories(${PCL_INCLUDE_DIRS})


add_library( Object Object.cpp )
add_library( SceneInformation SceneInformation.cpp )
add_library( ApiConvertKTHDB ApiConvertKTHDB.cpp )
add_library( DatabaseInformation DatabaseInformation.cpp )
add_library( ApiFeatureExtraction ApiFeatureExtraction.cpp )
add_library( ObjectFeatures ObjectFeatures.cpp )
add_library( ObjectPairFeatures ObjectPairFeatures.cpp )
add_library( FeatureInformation FeatureInformation.cpp  )
add_library(  AllFeatSingleObject AllFeatSingleObject.cpp )
add_library( AllFeatPairObject AllFeatPairObject.cpp )
add_library( TestScene TestScene.cpp  )
add_library( LOOCV LOOCV.cpp  )

add_executable(test test.cpp)

target_link_libraries( test DatabaseInformation Object SceneInformation ApiConvertKTHDB ObjectFeatures ApiFeatureExtraction FeatureInformation ObjectPairFeatures TestScene LOOCV AllFeatSingleObject AllFeatPairObject )
target_link_libraries( AllFeatSingleObject FeatureInformation)
target_link_libraries( AllFeatPairObject FeatureInformation)
target_link_libraries( DatabaseInformation SceneInformation )
target_link_libraries( DatabaseInformation ApiConvertKTHDB )
target_link_libraries( SceneInformation Object )
target_link_libraries( ObjectFeatures Object )
target_link_libraries( ApiFeatureExtraction SceneInformation )
target_link_libraries( ApiFeatureExtraction ObjectFeatures )
target_link_libraries( ApiFeatureExtraction ObjectPairFeatures )
target_link_libraries( ObjectPairFeatures Object)
target_link_libraries( DatabaseInformation ApiFeatureExtraction )
target_link_libraries( SceneInformation FeatureInformation ) 
target_link_libraries( ObjectFeatures FeatureInformation)
target_link_libraries( ObjectPairFeatures FeatureInformation)
target_link_libraries( ApiFeatureExtraction FeatureInformation)
target_link_libraries( TestScene Object)
target_link_libraries( TestScene ObjectFeatures)
target_link_libraries( LOOCV DatabaseInformation)
target_link_libraries( LOOCV TestScene)
target_link_libraries( SceneInformation AllFeatSingleObject)
target_link_libraries( DatabaseInformation AllFeatSingleObject)
target_link_libraries( SceneInformation AllFeatPairObject)
target_link_libraries( DatabaseInformation AllFeatPairObject)
target_link_libraries( DatabaseInformation ${OpenCV_LIBS} )
target_link_libraries( test ${OpenCV_LIBS} )




