#include "Object.hpp"
#include "SceneInformation.hpp"
#include "ApiConvertKTHDB.hpp"
#include "ObjectFeatures.hpp"
#include "DatabaseInformation.hpp"
#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include "TestScene.hpp"
#include "LOOCV.hpp"
#define DEBUG 0

/* 
 main containing do_loocv , argument = folder name
 do_loocv: training file list ; test file list - set now manually
 FE and learning using training_file_list (use class Database Information) and get GMMs at the end
 using test_file_list compute features
 classification
*/

using namespace std; 

int main(int argc, char *argv[]) {

  if (argc < 2) {
    cout << "Insert the data folder with the XML annotation files." << endl;
    return 0;
  }

  string dataFolderName = (argv[ 1 ]);
  cout << "The input folder name is: " << dataFolderName << endl; 
  string inputFolder =  "./" +  dataFolderName ;

  LOOCV computeLoocv(inputFolder); 
  computeLoocv.compute();

  return 0;
}

