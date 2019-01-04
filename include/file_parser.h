#ifndef ROBOAM_FILE_PARSER_H
#define ROBOAM_FILE_PARSER_H

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include "alphanum.hpp"
#include <iostream>
#include <set>
#include <Eigen/Core>
#include <string>

using namespace boost::filesystem;

namespace roboam {

    class FileParser {
    public:
        FileParser();

        ~FileParser();

        static void setAll(const path &root_path, const std::string &ext, std::vector<path> &ret);



    private:

        static void sortPath(std::vector<path> &paths);


    };
}

#endif //PROJECT_FILE_PARSER_H
