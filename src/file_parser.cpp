#include "file_parser.h"

inline bool starts_with(const std::string &str, const std::string &start) {

    const size_t start_size = start.size();
    if (str.size() < start_size) return false;
    for (size_t i = 0; i < start_size; ++i) {
        if (str[i] != start[i]) return false;
    }
    return true;
}

namespace alphanum {
    struct alphanum_path_comp {
        inline bool operator()(const boost::filesystem::path &l, const boost::filesystem::path &r) {
            return alphanum_impl(l.c_str(), r.c_str()) < 0;
        }
    };
};

namespace roboam {

    FileParser::FileParser() {}

    FileParser::~FileParser() {}


    void FileParser::setAll(const path &root_path, const std::string &ext, std::vector<path> &ret) {

        recursive_directory_iterator it(root_path);
        recursive_directory_iterator endit;

        while (it != endit) {
            if (is_regular_file(*it) && it->path().extension() == ext) ret.push_back(it->path());
            ++it;

        }

        sortPath(ret);

    }




//http://www.davekoelle.com/alphanum.html
    void FileParser::sortPath(std::vector<path> &paths) {
        std::set<boost::filesystem::path, alphanum::alphanum_less<boost::filesystem::path> > files_;

        for (unsigned i = 0; i < paths.size(); ++i) {
            files_.insert(paths[i]);
        }


        std::sort(paths.begin(), paths.end(), alphanum::alphanum_path_comp());


    }

}





