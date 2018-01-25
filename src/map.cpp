//
// Created by Feng,Yan on 2018/1/18.
//

#include "map.h"

namespace slam {
    void Map::insert_key_frame(Frame::Ptr frame) {
        cout << "Key frame size=" << _key_frames.size() << endl;
        if (_key_frames.find(frame->_id) == _key_frames.end()) {
            _key_frames.insert(make_pair(frame->_id, frame));
            return;
        }
        _key_frames[frame->_id] = frame;
    }

    void Map::insert_map_point(MapPoint::Ptr map_point) {
        cout << "Map point size=" << _map_points.size() << endl;
        if (_map_points.find(map_point->_id) == _map_points.end()) {
            _map_points.insert(make_pair(map_point->_id, map_point));
            return;
        }
        _map_points[map_point->_id] = map_point;
    }
}