//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef SLAM_LEARNING_CONFIG_H
#define SLAM_LEARNING_CONFIG_H

#include "common/common.h"

namespace slam {

    class Config {
    private:
        Config() {}
        cv::FileStorage _file;

    public:
        static std::shared_ptr<Config> _config;
        ~Config();

        static void setParameterFile(const std::string &filename);

        template<typename T>
        static T get(const std::string &key) {
            return T(Config::_config->_file[key]);
        }
    };
}

#endif //SLAM_LEARNING_CONFIG_H
