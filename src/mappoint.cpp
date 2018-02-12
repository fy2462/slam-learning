//
// Created by Feng,Yan on 2018/1/18.
//

#include "mappoint.h"

namespace slam {
    MapPoint::MapPoint() : _id(-1), _pos(Vector3d(0, 0, 0)), _norm(Vector3d(0, 0, 0)), _good_point(true),
                           _visible_times(0), _matched_times(0) {}

    MapPoint::MapPoint(long unsigned int id, const Vector3d &position, const Vector3d &norm, Frame *frame,
                       const Mat &descriptor) : _id(id), _pos(position), _norm(norm), _good_point(true),
                                                _visible_times(1),
                                                _matched_times(1),
                                                _descriptor(descriptor) {
        _observed_frames.push_back(frame);
    }

    MapPoint::Ptr MapPoint::createMapPoint() {
        return MapPoint::Ptr(new MapPoint(_factory_id++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }

    MapPoint::Ptr
    MapPoint::createMapPoint(const Vector3d &pos_world, const Vector3d &norm, const Mat &descriptor, Frame *frame) {
        return MapPoint::Ptr(new MapPoint(_factory_id++, pos_world, norm, frame, descriptor));
    }

    unsigned long MapPoint::_factory_id = 0;
}