//
// Created by Feng,Yan on 2018/1/18.
//

#ifndef PROJECT_CAMERA_H
#define PROJECT_CAMERA_H

#include "common/common.h"

namespace slam {


    class Camera {

    public:
        typedef std::shared_ptr <Camera> Ptr;
        float _fx, _fy, _cx, _cy, _depth_scale; // Camera internal parameter

        Camera();

        Camera(float fx, float fy, float cx, float cy, float depth_scale = 0) :
                _fx(fx), _fy(fy), _cx(cx), _cy(cy), _depth_scale(depth_scale) {}

        Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
        Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
        Vector2d camera2pixel( const Vector3d& p_c );
        Vector3d pixel2camera( const Vector2d& p_p, double depth=1 );
        Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
        Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

    };

}

#endif //PROJECT_CAMERA_H
