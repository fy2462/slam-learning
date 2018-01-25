//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef SLAM_LEARNING_MAP_H
#define SLAM_LEARNING_MAP_H

#include "common/common.h"
#include "frame.h"
#include "mappoint.h"

namespace slam {
    class Map {
    public:
        typedef std::shared_ptr<Map> Ptr;
        unordered_map<unsigned long, MapPoint::Ptr> _map_points;
        unordered_map<unsigned long, Frame::Ptr> _key_frames;

        Map() {};

        void insert_key_frame(Frame::Ptr frame);

        void insert_map_point(MapPoint::Ptr map_point);
    };
}

#endif //SLAM_LEARNING_MAP_H
