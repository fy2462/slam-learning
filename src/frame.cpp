//
// Created by Feng,Yan on 2018/1/18.
//

#include "frame.h"

namespace slam {
    Frame::Frame() : _id(-1), _time_stamp(-1), _camera(nullptr), _is_key_frame(false) {}

    Frame::Frame(long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth) :
            _id(id),
            _time_stamp(time_stamp),
            _camera(camera),
            _color_img(color),
            _depth_img(depth),
            _is_key_frame(false) {}

    Frame::~Frame() {}

    Frame::Ptr Frame::createFrame() {
        static long factory_id = 0;
        return Frame::Ptr(new Frame(factory_id++));
    }

    double Frame::findDepth(const cv::KeyPoint &kp) {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = _depth_img.ptr<ushort>(y)[x];
        if (d != 0) {
            return double(d) / _camera->_depth_scale;
        }

        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++) {
            d = _depth_img.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0) {
                return double(d) / _camera->_depth_scale;
            }
        }
        return -1.0;
    }

    Vector3d Frame::getCamCerter() const {
        return T_c_w.inverse().translation();
    }

    bool Frame::isInFrame(const Vector3d &pt_world) {
        Vector3d p_cam = _camera->world2camera(pt_world, T_c_w);
        if (p_cam(2, 0) < 0) {
            return false;
        }
        Vector2d pixel = _camera->world2pixel(pt_world, T_c_w);
        int u = pixel(0, 0);
        int v = pixel(1, 0);
        return u > 0 && v > 0 && u < _color_img.cols && v < _color_img.rows;
    }
}
