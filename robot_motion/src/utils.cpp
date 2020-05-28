
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "interpolation.h" // alglib
#include <utils.hpp>
#include <eigen3/Eigen/Core>

namespace read_txt_files {

int readFile_nlines(const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file" << std::endl;
        exit(1); // terminate with error
    }

    while (std::getline(inFile, str)) {
        n_data += 1;
    }

    inFile.close();

    return n_data;
}


int readFile(Eigen::MatrixXf &mat, const std::string fileNm) {
    int n_data = 0;
    int x;
    std::ifstream inFile;
    std::string str;


    inFile.open(fileNm);
    if (!inFile) {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    boost::char_separator<char> sep(" "); //Use space as the delimiter
    while (std::getline(inFile, str)) {
        std::vector<float> line;
        tokenizer tok(str, sep);
        //
        std::transform( tok.begin(), tok.end(), std::back_inserter(line),
                &boost::lexical_cast<float,std::string> );
        int t = 0;
        for(std::vector<float>::iterator it = line.begin(); it != line.end(); ++it) {
            mat(t,n_data) = *it;
            // std::cout << line.size() << std::endl;
            t += 1;
        }

        n_data += 1;
        // std::copy(line.begin(), line.end(), std::ostream_iterator<float>(std::cout,"\n") ); //Print those for testing
    }

    inFile.close();
    return 1;

}

}

namespace utils {
// void spline_piecewise_interp(std::vector<alglib::spline1dinterpolant> &splines, Eigen::MatrixXf &data, double start_time, double end_time) {

// 		int n_vecs = data.rows(); int n_data = data.cols();

// 		alglib::real_1d_array t;
// 		t.setlength(n_data);


// 		std::vector<alglib::real_1d_array> joint_d;
// 		joint_d.resize(n_vecs);

// 		// Define dt
// 	    double dt = (end_time - start_time) / n_data;

//     	for (int j = 0;j < n_data;j++) {
//     		t(j) = start_time + j * dt;
//     	} 

// 	    for (int i = 0;i < n_vecs;i++) {
// 	    	joint_d[i].setlength(n_data);
// 	    	for (int k = 0;k < n_data;k++) {
// 	    		joint_d[i](k) = data(i,k);
// 	    	}
// 	    	alglib::spline1dbuildcubic(t, joint_d[i], n_data, 2,0.0,2,0.0, splines[i]);
// 	    	// alglib::spline1dbuildlinear(t, joint_d[i], n_data, splines[i]);
	    	
// 	    }

// 	}

// 	using ecl::CubicSpline;

//     // This function takes in 
//     void spline_interp(std::vector<ecl::CubicSpline> &splines, Eigen::MatrixXf &data, const int n_data, double start_time, double stop_time) {


//         ecl::Array<double> t(n_data);
//         int n_vecs = data.rows();
//         std::vector<ecl::Array<double>> joints;
//         joints.resize(n_vecs);


//         double dt = (stop_time - start_time) / n_data; // The rate at which controls are sent out.

//         for (int k = 0;k < n_vecs; k++) {
//             joints[k].resize(n_data);
             
//             for (int i = 0;i < n_data;i++) {
//                 t[i]  = start_time + dt * i; 
//                 joints[k].at(i) = (double)data(k,i);
//             }                     
//         }


//          for (int k = 0; k < n_vecs; k++) {
//             splines[k] = ecl::CubicSpline::ContinuousDerivatives(t, joints[k], 0, 0);

//          }

//     }


    int* linspace(double a, double b, int c){
    	int* line = new int[c];

		// dt - uniform
		double delta = (b-a) / (c);
		for (int i=0; i < c; ++i) {
        	line[i]= a + (i*delta);
		}
		// std::cout << line << std::endl;
    	return line;
	}
}
