//
// Created by Feng,Yan on 2018/1/19.
//

// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "config.h"
#include "visual_obometry.h"

int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }

    slam::Config::setParameterFile(argv[1]);
    slam::VisualOdometry::Ptr vo(new slam::VisualOdometry);

    string dataset_dir = slam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataset_dir << endl;
    ifstream fin(dataset_dir + "/associate.txt");
    if (!fin) {
        cout << "please generate the associate file called associate.txt!" << endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while (!fin.eof()) {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(dataset_dir + "/" + rgb_file);
        depth_files.push_back(dataset_dir + "/" + depth_file);

        if (fin.good() == false)
            break;
    }

    slam::Camera::Ptr camera(new slam::Camera);

    // visualization
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);

    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);

    cout << "read total " << rgb_files.size() << " entries" << endl;
    for (int i = 0; i < rgb_files.size(); i++) {
        Mat color = cv::imread(rgb_files[i]);
        Mat depth = cv::imread(depth_files[i], -1);
        if (color.data == nullptr || depth.data == nullptr)
            break;
        slam::Frame::Ptr pFrame = slam::Frame::createFrame();
        pFrame->_camera = camera;
        pFrame->_color_img = color;
        pFrame->_depth_img = depth;
        pFrame->_time_stamp = rgb_times[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        cout << "VO costs time: " << timer.elapsed() << endl;

        if (vo->_state == slam::VisualOdometry::LOST)
            break;
        SE3 Tcw = pFrame->T_c_w.inverse();

        // show the map and the camera pose
        cv::Affine3d M(
                cv::Affine3d::Mat3(
                        Tcw.rotation_matrix()(0, 0), Tcw.rotation_matrix()(0, 1), Tcw.rotation_matrix()(0, 2),
                        Tcw.rotation_matrix()(1, 0), Tcw.rotation_matrix()(1, 1), Tcw.rotation_matrix()(1, 2),
                        Tcw.rotation_matrix()(2, 0), Tcw.rotation_matrix()(2, 1), Tcw.rotation_matrix()(2, 2)
                ),
                cv::Affine3d::Vec3(
                        Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)
                )
        );

        Mat img_show = color.clone();
        for ( auto& pt:vo->_map->_map_points )
        {
            slam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->_camera->world2pixel ( p->_pos, pFrame->T_c_w );
            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow("image", color);
        cv::waitKey(1);
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}

