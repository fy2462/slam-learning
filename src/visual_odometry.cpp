//
// Created by Feng,Yan on 2018/1/19.
//

//
// Created by Feng,Yan on 2018/1/19.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <boost/timer.hpp>

#include "config.h"
#include "visual_obometry.h"
#include "g2o/g2o_types.h"

namespace slam {

    VisualOdometry::VisualOdometry() : _state(INITIALIZING), _ref(nullptr), _current(nullptr), _map(new Map),
                                       _num_lost(0), _num_inliers(0),
                                       _matcher_flann(new cv::flann::LshIndexParams(5, 10, 2)) {
        _num_of_features = Config::get<int>("number_of_features");
        _scale_factor = Config::get<double>("scale_factor");
        _level_pyramid = Config::get<int>("level_pyramid");
        _match_ratio = Config::get<float>("match_ratio");
        _max_num_lost = Config::get<float>("max_num_lost");
        _min_inliers = Config::get<int>("min_inliers");
        key_frame_min_rot = Config::get<double>("keyframe_rotation");
        key_frame_min_trans = Config::get<double>("keyframe_translation");
        _map_point_erase_ratio = Config::get<double>("map_point_erase_ratio");
        _orb = cv::ORB::create(_num_of_features, _scale_factor, _level_pyramid);
    }

    VisualOdometry::~VisualOdometry() {}

    bool VisualOdometry::addFrame(Frame::Ptr frame) {
        switch (_state) {
            case INITIALIZING:
                _state = OK;
                _current = _ref = frame;
                extractKeyPoints();
                computeDescriptors();
                //setRef3DPoints();
                addKeyFrame();
                break;
            case OK:
                _current = frame;
                _current->T_c_w = _ref->T_c_w;
                extractKeyPoints();
                computeDescriptors();
                featureMatching();
                //poseEstimationPnP();
                poseEstimationICP();
                if (checkEstimatedPose()) {
                    _current->T_c_w = _T_c_r_estimated;
                    optimizeMap();
                    //_ref = _current;
                    //setRef3DPoints();
                    _num_lost = 0;
                    if (checkKeyFrame()) {
                        addKeyFrame();
                    }
                } else {
                    _num_lost++;
                    if (_num_lost > _max_num_lost) {
                        _state = LOST;
                    }
                    return false;
                }
            case LOST:
                cout << "vo has lost." << endl;
                break;
        }
        return true;
    }

    void VisualOdometry::extractKeyPoints() {
        boost::timer timer;
        _orb->detect(_current->_color_img, _keypoints_curr);
        cout << "extract keypoints cost time: " << timer.elapsed() << endl;
    }

    void VisualOdometry::computeDescriptors() {
        boost::timer timer;
        _orb->compute(_current->_color_img, _keypoints_curr, _descriptors_curr);
        cout << "descriptor computation cost time: " << timer.elapsed() << endl;
    }

    // BFMatcher和FlannBasedMatcher二者的区别在于BFMatcher总是尝试所有可能的匹配，从而使得它总能够找到最佳匹配，这也是Brute Force（暴力法）的原始含义。
    // 而FlannBasedMatcher中FLANN的含义是Fast Library for Approximate Nearest Neighbors，从字面意思可知它是一种近似法，
    // 算法更快但是找到的是最近邻近似匹配，所以当我们需要找到一个相对好的匹配但是不需要最佳匹配的时候往往使用FlannBasedMatcher。
    // 当然也可以通过调整FlannBasedMatcher的参数来提高匹配的精度或者提高算法速度，但是相应地算法速度或者算法精度会受到影响。
    void VisualOdometry::featureMatching() {
        boost::timer timer;
        vector<cv::DMatch> matches;
        // select the condadates in map
        Mat desp_map;
        vector<MapPoint::Ptr> candidate;
        cout << "current map points count: " << _map->_map_points.size() << endl;
        for (auto &allPoints : _map->_map_points) {
            MapPoint::Ptr &p = allPoints.second;
            // check if p in current frame image
            if (_current->isInFrame(p->_pos)) {
                p->_visible_times++;
                candidate.push_back(p);
                desp_map.push_back(p->_descriptor);
            }
        }
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        //matcher.match(desp_map, _descriptors_curr, matches);
        _matcher_flann.match(desp_map, _descriptors_curr, matches);
        // select the best matches
        auto min_match_iter = std::min_element(matches.begin(), matches.end(),
                                               [](const cv::DMatch &m1, const cv::DMatch m2) {
                                                   return m1.distance < m2.distance;
                                               });
        float min_dis = min_match_iter->distance;

        _match_3dpts.clear();
        _match_2dkp_index.clear();
        _last_frame_3dpts.clear();
        for (cv::DMatch &m: matches) {
            if (m.distance < max<float>(min_dis * _match_ratio, 30.0)) {
                _last_frame_3dpts.push_back(candidate[m.queryIdx]);
                _match_3dpts.push_back(keyPoint2MapPoint(_keypoints_curr[m.trainIdx], m.trainIdx));
                _match_2dkp_index.push_back(m.trainIdx);
            }
        }

        cout << "good matches: " << _match_3dpts.size() << endl;
        cout << "match cost time: " << timer.elapsed() << endl;
    }

    MapPoint::Ptr VisualOdometry::keyPoint2MapPoint(cv::KeyPoint& key_point, int index) {

        double d = _ref->findDepth(key_point);
        Vector3d p_world = _ref->_camera->pixel2world(Vector2d(key_point.pt.x, key_point.pt.y),
                                                      _current->T_c_w, d);
        Vector3d n = p_world - _ref->getCamCerter();
        n.normalize();
        MapPoint::Ptr map_point = MapPoint::createMapPoint(
                p_world, n, _descriptors_curr.row(index).clone(), _current.get()
        );
        return map_point;
    }

    void VisualOdometry::setRef3DPoints() {
        _pts_3d_ref.clear();
        _descriptors_ref = Mat();
        for (size_t i = 0; i < _keypoints_curr.size(); i++) {
            double d = _ref->findDepth(_keypoints_curr[i]);
            if (d > 0) {
                Vector3d p_cam =
                        _ref->_camera->pixel2camera(Vector2d(_keypoints_curr[i].pt.x, _keypoints_curr[i].pt.y), d);
                _pts_3d_ref.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
                _descriptors_ref.push_back(_descriptors_curr.row(i));
            }
        }
    }

    void VisualOdometry::poseEstimationPnP() {

        // construct the 3d 2d observations
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for ( int index:_match_2dkp_index )
        {
            pts2d.push_back ( _keypoints_curr[index].pt );
        }
        for ( MapPoint::Ptr pt:_match_3dpts )
        {
            pts3d.push_back( pt->getPositionCV() );
        }

        Mat K = (cv::Mat_<double>(3, 3) << _ref->_camera->_fx, 0, _ref->_camera->_cx,
                0, _ref->_camera->_fy, _ref->_camera->_cy,
                0, 0, 1
        );
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac ( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
        _num_inliers = inliers.rows;
        cout<<"pnp inliers: "<<_num_inliers<<endl;
        _T_c_r_estimated = SE3 (
                SO3 ( rvec.at<double> ( 0,0 ), rvec.at<double> ( 1,0 ), rvec.at<double> ( 2,0 ) ),
                Vector3d ( tvec.at<double> ( 0,0 ), tvec.at<double> ( 1,0 ), tvec.at<double> ( 2,0 ) )
        );

        // using bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
        Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block* solver_ptr = new Block ( linearSolver );
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm ( solver );

        g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
        pose->setId ( 0 );
        pose->setEstimate ( g2o::SE3Quat (
                _T_c_r_estimated.rotation_matrix(), _T_c_r_estimated.translation()
        ));
        optimizer.addVertex ( pose );

        // edges
        for ( int i=0; i<inliers.rows; i++ )
        {
            int index = inliers.at<int> ( i,0 );
            // 3D -> 2D projection
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId ( i );
            edge->setVertex ( 0, pose );
            edge->_camera = _current->_camera.get();
            edge->_point = Vector3d ( pts3d[index].x, pts3d[index].y, pts3d[index].z );
            edge->setMeasurement ( Vector2d ( pts2d[index].x, pts2d[index].y ) );
            edge->setInformation ( Eigen::Matrix2d::Identity() );
            optimizer.addEdge ( edge );
            // set the inlier map points
            _match_3dpts[index]->_matched_times++;
        }

        optimizer.initializeOptimization();
        optimizer.optimize ( 10 );

        _T_c_r_estimated = SE3 (
                pose->estimate().rotation(),
                pose->estimate().translation()
        );

        cout<<"T_c_w_estimated_: "<<endl<<_T_c_r_estimated.matrix()<<endl;

    }

    void VisualOdometry::poseEstimationICP() {
        vector<Eigen::Vector3d> pts3d;
        vector<Eigen::Vector3d> pts3d_last;
        vector<cv::Point2f> pts2d;

        for (int index: _match_2dkp_index) {
            pts2d.push_back(_keypoints_curr[index].pt);
        }
        for (MapPoint::Ptr pt:_last_frame_3dpts) {
            Eigen::Vector3d cam3d = _ref->_camera->world2camera(pt->_pos, _T_c_r_estimated);
            pts3d_last.push_back(cam3d);
        }
        for (MapPoint::Ptr pt:_match_3dpts) {
            Eigen::Vector3d cam3d = _ref->_camera->world2camera(pt->_pos, _current->T_c_w);
            pts3d.push_back(cam3d);
        }

        // add the g2o bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Bolck;
        // 6 * 2 linearSolver
        Bolck::LinearSolverType *linearSolver = new g2o::LinearSolverEigen<Bolck::PoseMatrixType>();
        Bolck *solver_ptr = new Bolck(linearSolver);
        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // the first vertex;
        g2o::VertexSE3Expmap *pose3d = new g2o::VertexSE3Expmap();
        pose3d->setId(0);
        pose3d->setEstimate(g2o::SE3Quat(
                Eigen::Matrix3d::Identity(),
                Eigen::Vector3d(0, 0, 0)
        ));

        optimizer.addVertex(pose3d);

        // one Vertex Edges;
        for (int i = 0; i < pts3d.size(); i++) {
            EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(pts3d[i]);
            edge->setId(i);
            edge->setVertex(0, pose3d);
            edge->_point = pts3d_last[i];
            edge->setMeasurement(pts3d_last[i]);
            edge->setInformation(Eigen::Matrix3d::Identity() * 1e4);
            optimizer.addEdge(edge);
        }

        optimizer.setVerbose(true);
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        _T_c_r_estimated = SE3(pose3d->estimate().rotation(),
                               pose3d->estimate().translation());

        cout << "T_c_w_estimated_: " << endl << Eigen::Isometry3d(pose3d->estimate()).matrix() << endl;
    }

    bool VisualOdometry::checkEstimatedPose() {
        if (_num_inliers < _min_inliers) {
            cout << "reject because inlier is too small: " << _num_inliers << endl;
            return false;
        }
        // compute last frame move to current pose ??
        SE3 T_r_c = _ref->T_c_w * _T_c_r_estimated.inverse();
        Sophus::Vector6d d = T_r_c.log();
        if (d.norm() > 5.0) {
            cout << "reject because motion is too large: " << d.norm() << endl;
            return false;
        }
        return true;
    }

    bool VisualOdometry::checkKeyFrame() {
        Sophus::Vector6d d = _T_c_r_estimated.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
            return true;
        return false;
    }

    void VisualOdometry::addKeyFrame() {
        cout << "adding a key-frame" << endl;
        if (_map->_key_frames.empty()) {
            for (size_t i = 0; i < _keypoints_curr.size(); i++) {
                double d = _current->findDepth(_keypoints_curr[i]);
                if (d <= 0) {
                    continue;
                }
                Vector3d p_world = _ref->_camera->pixel2world(
                        Vector2d(_keypoints_curr[i].pt.x, _keypoints_curr[i].pt.y), _current->T_c_w, d);
                Vector3d n = p_world - _ref->getCamCerter();
                // 开方归一化
                n.normalize();
                MapPoint::Ptr map_point = MapPoint::createMapPoint(p_world, n, _descriptors_curr.row(i).clone(),
                                                                   _current.get());
                _map->insert_map_point(map_point);
            }
        }
        _map->insert_key_frame(_current);
        _ref = _current;
    }

    void VisualOdometry::addMapPoints() {
        // add the new map points into map
        vector<bool> matched(_keypoints_curr.size(), false);
        for (int i = 0; i < _keypoints_curr.size(); i++) {
            if (matched[i] == true) {
                continue;
            }
            double d = _ref->findDepth(_keypoints_curr[i]);
            if (d < 0) {
                continue;
            }
            Vector3d p_world = _ref->_camera->pixel2world(Vector2d(_keypoints_curr[i].pt.x, _keypoints_curr[i].pt.y),
                                                          _current->T_c_w, d);
            Vector3d n = p_world - _ref->getCamCerter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, _descriptors_curr.row(i).clone(), _current.get()
            );
            _map->insert_map_point(map_point);
        }
    }

    void VisualOdometry::optimizeMap() {
        // remove the hardly seen adn no visible points
        for (auto iter = _map->_map_points.begin(); iter != _map->_map_points.end(); iter++) {
            MapPoint::Ptr mpp = iter->second;
            if (!_current->isInFrame(mpp->_pos)) {
                // filter outside of current frame
                iter = _map->_map_points.erase(iter);
                continue;
            }
            float match_ratio = float(mpp->_matched_times) / mpp->_visible_times;
            if (match_ratio < _map_point_erase_ratio) {
                iter = _map->_map_points.erase(iter);
                continue;
            }
            double angle = getViewAngle(_current, mpp);
            if (angle > M_PI / 6.) {
                iter = _map->_map_points.erase(iter);
                continue;
            }
            if (mpp->_good_point == false) {
                // value the good point code
            }
        }

        if (_match_2dkp_index.size() < 100) {
            addMapPoints();
        }
        if (_map->_map_points.size() > 1000) {
            _map_point_erase_ratio += 0.05;
        } else {
            _map_point_erase_ratio = 0.1;
        }
        cout << "map points: " << _map->_map_points.size() << endl;
    }

    double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {
        Vector3d n = point->_pos - frame->getCamCerter();
        n.normalize();
        return acos(n.transpose() * point->_norm);
    }

}