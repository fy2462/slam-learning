//
// Created by Feng,Yan on 2018/1/19.
//

//
// Created by Feng,Yan on 2018/1/19.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/timer.hpp>

#include "config.h"
#include "visual_obometry.h"
#include "g2o/g2o_types.h"

namespace slam {

    VisualOdometry::VisualOdometry() : _state(INITIALIZING), _ref(nullptr), _current(nullptr), _map(new Map),
                                       _num_lost(0),
                                       _num_inliers(0) {
        _num_of_features = Config::get<int>("number_of_features");
        _scale_factor = Config::get<double>("scale_factor");
        _level_pyramid = Config::get<int>("level_pyramid");
        _match_ratio = Config::get<float>("match_ratio");
        _max_num_lost = Config::get<float>("max_num_lost");
        _min_inliers = Config::get<int>("min_inliers");
        key_frame_min_rot = Config::get<double>("keyframe_rotation");
        key_frame_min_trans = Config::get<double>("keyframe_translation");
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
                setRef3DPoints();
                break;
            case OK:
                _current = frame;
                extractKeyPoints();
                computeDescriptors();
                featureMatching();
                poseEstimationPnP();
                if (checkEstimatedPose()) {
                    _current->T_c_w = _T_c_r_estimated * _ref->T_c_w;
                    _ref = _current;
                    setRef3DPoints();
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
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(_descriptors_ref, _descriptors_curr, matches);
        //_matcher_flann.match(_descriptors_ref, _descriptors_curr, matches);
        // select the best matches
        float min_dis = std::min_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch m2) {
            return m1.distance < m2.distance;
        })->distance;

        _feature_matches.clear();
        for (cv::DMatch &m: matches) {
            if (m.distance < max<float>(min_dis * _match_ratio, 30.0)) {
                _feature_matches.push_back(m);
            }
        }
        cout << "good matches: " << _feature_matches.size() << endl;
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
        vector<cv::Point3f> pts3d;
        vector<cv::Point2f> pts2d;

        for (cv::DMatch m:_feature_matches) {
            pts3d.push_back(_pts_3d_ref[m.queryIdx]);
            pts2d.push_back(_keypoints_curr[m.trainIdx].pt);
        }
        Mat K = (cv::Mat_<double>(3, 3) << _ref->_camera->_fx, 0, _ref->_camera->_cx,
                0, _ref->_camera->_fy, _ref->_camera->_cy,
                0, 0, 1
        );
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
        _num_inliers = inliers.rows;
        cout << "pnp inliers: " << _num_inliers << endl;
        _T_c_r_estimated = SE3(
                SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
                Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
        );

        // add the g2o bundle adjustment to optimize the pose
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Bolck;
        // 6 * 2 linearSolver
        Bolck::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Bolck::PoseMatrixType>();
        Bolck* solver_ptr = new Bolck(linearSolver);
        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // the first vertex;
        g2o::VertexSE3Expmap* pose3d = new g2o::VertexSE3Expmap();
        pose3d->setId(0);
        pose3d->setEstimate(g2o::SE3Quat(
                _T_c_r_estimated.rotation_matrix(),
                _T_c_r_estimated.translation()
        ));
        optimizer.addVertex(pose3d);

        // one Vertex Edges;
        for (int i = 0; i < inliers.rows; i++) {
            int index = inliers.at<int>(i, 0);
            EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId(i);
            edge->setVertex(0, pose3d);
            edge->_camera = _current->_camera.get();
            edge->_point = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
            edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        _T_c_r_estimated = SE3(pose3d->estimate().rotation(),
                               pose3d->estimate().translation());

    }

    bool VisualOdometry::checkEstimatedPose() {
        if (_num_inliers < _min_inliers) {
            cout << "reject because inlier is too small: " << _num_inliers << endl;
            return false;
        }
        Sophus::Vector6d d = _T_c_r_estimated.log();
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
        _map->insert_key_frame(_current);
    }

}