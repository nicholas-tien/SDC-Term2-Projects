#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "UKF.h"
#include "MeasurementPackage.h"
#include "ground_truth_package.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]){
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    //make sure that the user has provided input and output files
    if (argc == 1){
        cerr << usage_instructions << endl;
    } else if (argc ==2) {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    } else if (argc == 3) {
        has_valid_args == true;
    } else if (argc > 3) {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args){
        exit(EXIT_FAILURE);
    }
}

int main(int argc,char * argv[]) {
   // std::cout << "Hello, World!" << std::endl;



    return 0;

}