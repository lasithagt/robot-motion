
#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/errors.hpp>
#include <ecl/concepts.hpp>
#include <ecl/converters.hpp>
#include "interpolation.h" // alglib
#include "boost/lexical_cast.hpp"

#include <eigen3/Eigen/Core>
#include <vector>

namespace read_txt_files {
    int readFile_nlines(const std::string);
    int readFile(Eigen::MatrixXf&, const std::string);
}

namespace utils {
    struct path_user {
        Eigen::MatrixXf data;
        double start_time;
        double end_time;
    } ;
    // void spline_interp(std::vector<ecl::CubicSpline> &splines, Eigen::MatrixXf &data, const int n_data, double start_time, double stop_time);
    // void spline_piecewise_interp(std::vector<alglib::spline1dinterpolant> &splines, Eigen::MatrixXf &data, double start_time, double end_time);
    int* linspace(double a, double b, int c);
}



#endif