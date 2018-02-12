//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef SLAM_LEARNING_MAPPOINT_H
#define SLAM_LEARNING_MAPPOINT_H

#include "common/common.h"

namespace slam {
    class Frame;

    class MapPoint {
    public:
        static unsigned long _factory_id;
        typedef std::shared_ptr<MapPoint> Ptr;
        unsigned long _id;
        bool _good_point;
        Vector3d _pos;
        Vector3d _norm;
        Mat _descriptor;
        vector<Frame*> _observed_frames; // key frame list
        int _observed_times;
        int _correct_times;
        int _matched_times;
        int _visible_times;

        MapPoint();

        MapPoint(
                unsigned long id, const Vector3d &position, const Vector3d &norm, Frame *frame = nullptr,
                const Mat &descriptor = Mat());

        inline cv::Point3f getPositionCV() const {
            return cv::Point3f(_pos(0, 0), _pos(1, 0), _pos(2, 0));
        }

        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr
        createMapPoint(const Vector3d &pos_world, const Vector3d &norm, const Mat &descriptor, Frame *frame);
    };


}

#endif //SLAM_LEARNING_MAPPOINT_H
