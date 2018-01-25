//
// Created by Feng,Yan on 2018/1/18.
//

#include "mappoint.h"

namespace slam {
    MapPoint::MapPoint() : _id(-1), _pos(Vector3d(0, 0, 0)), _norm(Vector3d(0, 0, 0)), _observed_times(0),
                           _correct_times(0) {}

    MapPoint::MapPoint(long id, Vector3d position, Vector3d norm) : _id(id), _pos(position), _norm(norm),
                                                                    _observed_times(0),
                                                                    _correct_times(0) {}

    MapPoint::Ptr MapPoint::createMapPoint() {
        static long factory_id = 0;
        return MapPoint::Ptr(new MapPoint(factory_id++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }
}