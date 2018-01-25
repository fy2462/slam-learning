//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef SLAM_LEARNING_MAPPOINT_H
#define SLAM_LEARNING_MAPPOINT_H

#include "common/common.h"

namespace slam {
    class Frame;
    class MapPoint{
    public:
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long _id;
        Vector3d _pos;
        Vector3d _norm;
        Mat _descriptor;
        int _observed_times;
        int _correct_times;

        MapPoint();
        MapPoint(long id, Vector3d position, Vector3d norm);

        static MapPoint::Ptr createMapPoint();
    };



}

#endif //SLAM_LEARNING_MAPPOINT_H
