//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef SLAM_LEARNING_FRAME_H
#define SLAM_LEARNING_FRAME_H

#include "common/common.h"
#include "camera.h"

namespace slam {

    class MapPoint;

    class Frame {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long _id;
        double _time_stamp;
        SE3 T_c_w;
        Camera::Ptr _camera;
        Mat _color_img, _depth_img;
        bool _is_key_frame;

    public:
        Frame();

        Frame(long id, double time_stamp = 0, SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, Mat color = Mat(),
              Mat depth = Mat());

        ~Frame();

        static Frame::Ptr createFrame();

        double findDepth(const cv::KeyPoint& kp);

        Vector3d getCamCerter() const;

        bool isInFrame(const Vector3d &pt_world);
    };
}

#endif //SLAM_LEARNING_FRAME_H
