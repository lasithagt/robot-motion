
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
    if (!inFile) 
    {
        std::cout << "Unable to open file";
        exit(1); // terminate with error
    }

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

    boost::char_separator<char> sep(" "); //Use space as the delimiter

    while (std::getline(inFile, str)) 
    {
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

namespace utils 
{

    void linear_piecewise_interp(const Eigen::Ref<Eigen::MatrixXd>& data, Eigen::MatrixXd& data_interpolated, int scale)
    {
        int current_size = data.cols();
        int new_size     = (current_size - 1) * scale;

        for (int i = 0;i < current_size;i++)
        {
            for (int j = 0;j < scale;j++)
            {
                data_interpolated.col(scale * i + j) = data.col(i) + (data.col(i+1)-data.col(i)) * (j/scale);
            }
        }
    }


    int* linspace(double a, double b, int c){
    	int* line = new int[c];

		// dt - uniform
		double delta = (b-a) / (c);

		for (int i=0; i < c; ++i) {
        	line[i]= a + (i * delta);
		}
    	return line;
	}
}
