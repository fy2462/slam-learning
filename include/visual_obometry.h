//
// Created by Feng,Yan on 2018/1/19.
//

#ifndef SLAM_LEARNING_VISUAL_OBOMETRY_H
#define SLAM_LEARNING_VISUAL_OBOMETRY_H

#include "common/common.h"
#include "map.h"
#include <opencv2/features2d/features2d.hpp>

namespace slam {
    class VisualOdometry {
    public:
        typedef std::shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING = -1, OK = 0, LOST
        };

        VOState _state;
        Map::Ptr _map;

        Frame::Ptr _ref;
        Frame::Ptr _current;

        cv::Ptr<cv::ORB> _orb;
        vector<cv::Point3f> _pts_3d_ref;

        vector<cv::KeyPoint> _keypoints_curr;
        Mat _descriptors_curr;
        Mat _descriptors_ref;
        vector<cv::DMatch> _feature_matches;
        cv::FlannBasedMatcher _matcher_flann; // flann matcher
        vector<MapPoint::Ptr> _match_3dpts; // 3d points matched
        vector<int> _match_2dkp_index; // 2d key points indexss
        vector<MapPoint::Ptr> _last_frame_3dpts; // last frame 3d points

        SE3 _T_c_r_estimated;
        int _num_inliers;
        int _num_lost;

        int _num_of_features;
        double _scale_factor;
        int _level_pyramid;
        float _match_ratio;
        int _max_num_lost;
        int _min_inliers;

        double key_frame_min_rot;
        double key_frame_min_trans;
        double _map_point_erase_ratio; // remove map point ratio

    public:
        VisualOdometry();

        ~VisualOdometry();

        bool addFrame(Frame::Ptr frame);

    protected:
        void extractKeyPoints();

        void computeDescriptors();

        void featureMatching();

        void poseEstimationPnP();

        void poseEstimationICP();

        void setRef3DPoints();

        MapPoint::Ptr keyPoint2MapPoint(cv::KeyPoint& key_point, int index);

        void addKeyFrame();

        bool checkEstimatedPose();

        bool checkKeyFrame();

        void optimizeMap();

        void addMapPoints();

        double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

    };
}

#endif //SLAM_LEARNING_VISUAL_OBOMETRY_H
