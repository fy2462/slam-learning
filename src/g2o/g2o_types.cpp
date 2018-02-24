//
// Created by Feng,Yan on 2018/1/26.
//

#include "g2o/g2o_types.h"
#include "common/projection.h"

namespace slam {
    void EdgeProjectXYZRGBD::computeError() {
        const g2o::VertexSBAPointXYZ *point = static_cast<const g2o::VertexSBAPointXYZ *> ( _vertices[0] );
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *> ( _vertices[1] );
        _error = _measurement - pose->estimate().map(point->estimate());
    }

    void EdgeProjectXYZRGBD::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[1] );
        g2o::SE3Quat T(pose->estimate());
        g2o::VertexSBAPointXYZ *point = static_cast<g2o::VertexSBAPointXYZ *> ( _vertices[0] );
        Eigen::Vector3d xyz = point->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi = -T.rotation().toRotationMatrix();

        _jacobianOplusXj(0, 0) = 0;
        _jacobianOplusXj(0, 1) = -z;
        _jacobianOplusXj(0, 2) = y;
        _jacobianOplusXj(0, 3) = -1;
        _jacobianOplusXj(0, 4) = 0;
        _jacobianOplusXj(0, 5) = 0;

        _jacobianOplusXj(1, 0) = z;
        _jacobianOplusXj(1, 1) = 0;
        _jacobianOplusXj(1, 2) = -x;
        _jacobianOplusXj(1, 3) = 0;
        _jacobianOplusXj(1, 4) = -1;
        _jacobianOplusXj(1, 5) = 0;

        _jacobianOplusXj(2, 0) = -y;
        _jacobianOplusXj(2, 1) = x;
        _jacobianOplusXj(2, 2) = 0;
        _jacobianOplusXj(2, 3) = 0;
        _jacobianOplusXj(2, 4) = 0;
        _jacobianOplusXj(2, 5) = -1;
    }

    void EdgeProjectXYZRGBDPoseOnly::computeError() {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *> ( _vertices[0] );
        _error = _measurement - pose->estimate().map(_point);
    }

    void EdgeProjectXYZRGBDPoseOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[0] );
        g2o::SE3Quat T(pose->estimate());
        Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    void EdgeProjectXYZ2UVPoseOnly::computeError() {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *> ( _vertices[0] );
        _error = _measurement - _camera->camera2pixel(
                pose->estimate().map(_point));
    }

    void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[0] );
        g2o::SE3Quat T(pose->estimate());
        Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z * z;

        _jacobianOplusXi(0, 0) = x * y / z_2 * _camera->_fx;
        _jacobianOplusXi(0, 1) = -(1 + (x * x / z_2)) * _camera->_fx;
        _jacobianOplusXi(0, 2) = y / z * _camera->_fx;
        _jacobianOplusXi(0, 3) = -1. / z * _camera->_fx;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = x / z_2 * _camera->_fx;

        _jacobianOplusXi(1, 0) = (1 + y * y / z_2) * _camera->_fy;
        _jacobianOplusXi(1, 1) = -x * y / z_2 * _camera->_fy;
        _jacobianOplusXi(1, 2) = -x / z * _camera->_fy;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1. / z * _camera->_fy;
        _jacobianOplusXi(1, 5) = y / z_2 * _camera->_fy;
    }

    void VertexCameraBAL::oplusImpl ( const double* update )
    {
        Eigen::VectorXd::ConstMapType v ( update, VertexCameraBAL::Dimension );
        _estimate += v;
    }

    void VertexPointBAL::oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v ( update );
        _estimate += v;
    }

    void EdgeObservationBAL::computeError()   // The virtual function comes from the Edge base class. Must define if you use edge.
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );

        ( *this ) ( cam->estimate().data(), point->estimate().data(), _error.data() );

    }

    template<typename T>
    bool EdgeObservationBAL::operator() ( const T* camera, const T* point, T* residuals ) const
    {
        T predictions[2];
        CamProjectionWithDistortion ( camera, point, predictions );
        residuals[0] = predictions[0] - T ( measurement() ( 0 ) );
        residuals[1] = predictions[1] - T ( measurement() ( 1 ) );

        return true;
    }


    void EdgeObservationBAL::linearizeOplus()
    {
        // use numeric Jacobians
        // BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        // return;

        // using autodiff from ceres. Otherwise, the system will use g2o numerical diff for Jacobians

        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*> ( vertex ( 0 ) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*> ( vertex ( 1 ) );
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        double *parameters[] = { const_cast<double*> ( cam->estimate().data() ), const_cast<double*> ( point->estimate().data() ) };
        double *jacobians[] = { dError_dCamera.data(), dError_dPoint.data() };
        double value[Dimension];
        bool diffState = BalAutoDiff::Differentiate ( *this, parameters, Dimension, value, jacobians );

        // copy over the Jacobians (convert row-major -> column-major)
        if ( diffState )
        {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        }
        else
        {
            assert ( 0 && "Error while differentiating" );
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.setZero();
        }
    }
}