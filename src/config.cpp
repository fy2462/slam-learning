//
// Created by Feng,Yan on 2018/1/18.
//

#include "config.h"

namespace slam {

    void Config::setParameterFile(const std::string& filename) {
        if (_config == nullptr) {
            _config = std::shared_ptr<Config>(new Config());
        }
        _config->_file = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (!_config->_file.isOpened()) {
            std::cerr << "parameter file " << filename << "does not exist." << std::endl;
            _config->_file.release();
            return;
        }
    }

    Config::~Config() {
        if (_config->_file.isOpened()) {
            _config->_file.release();
        }
    }

    shared_ptr<Config> Config::_config = nullptr;

}

