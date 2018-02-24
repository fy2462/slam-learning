//
// Created by Feng,Yan on 2018/1/25.
//

#ifndef SLAM_LEARNING_G2O_TYPES_H
#define SLAM_LEARNING_G2O_TYPES_H

#include "common/common.h"
#include "camera.h"

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include "g2o/core/base_binary_edge.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include "ceres/autodiff.h"
#include "tools/rotation.h"

namespace slam {
    class EdgeProjectXYZRGBD
            : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        virtual void computeError();

        virtual void linearizeOplus();

        virtual bool read(std::istream &in) { return false; }

        virtual bool write(std::ostream &out) const { return false; }

    };

// only to optimize the pose, no point
    class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

        virtual void computeError();

        virtual void linearizeOplus();

        virtual bool read(std::istream &in) { return false; }

        virtual bool write(std::ostream &out) const { return false; }

        Vector3d _point;
    };

    class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError();

        virtual void linearizeOplus();

        virtual bool read(std::istream &in) { return false; }

        virtual bool write(std::ostream &os) const { return false; };

        Vector3d _point;
        Camera* _camera;
    };

    class VertexCameraBAL : public g2o::BaseVertex<9,Eigen::VectorXd>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexCameraBAL() {}

        virtual bool read ( std::istream& /*is*/ )
        {
            return false;
        }

        virtual bool write ( std::ostream& /*os*/ ) const
        {
            return false;
        }

        virtual void setToOriginImpl() {}

        virtual void oplusImpl ( const double* update );

    };


    class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPointBAL() {}

        virtual bool read ( std::istream& /*is*/ )
        {
            return false;
        }

        virtual bool write ( std::ostream& /*os*/ ) const
        {
            return false;
        }

        virtual void setToOriginImpl() {}

        void oplusImpl ( const double* update );
    };

    class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,VertexCameraBAL, VertexPointBAL>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeObservationBAL() {}

        virtual bool read ( std::istream& /*is*/ )
        {
            return false;
        }

        virtual bool write ( std::ostream& /*os*/ ) const
        {
            return false;
        }

        virtual void computeError();

        template<typename T>
        bool operator() ( const T* camera, const T* point, T* residuals ) const;


        virtual void linearizeOplus();
    };

}

#endif //SLAM_LEARNING_G2O_TYPES_H
