//
// Created by Feng,Yan on 2018/2/24.
//

#include "backend_optimization.h"

namespace slam {

    void BackendOptimization::BuildProblem(const BALProblem *bal_problem, g2o::SparseOptimizer *optimizer, const BundleParams &params) {
        const int num_points = bal_problem->num_points();
        const int num_cameras = bal_problem->num_cameras();
        const int camera_block_size = bal_problem->camera_block_size();
        const int point_block_size = bal_problem->point_block_size();

        // Set camera vertex with initial value in the dataset.
        const double *raw_cameras = bal_problem->cameras();
        for (int i = 0; i < num_cameras; ++i) {
            ConstVectorRef temVecCamera(raw_cameras + camera_block_size * i, camera_block_size);
            VertexCameraBAL *pCamera = new VertexCameraBAL();
            pCamera->setEstimate(temVecCamera);   // initial value for the camera i..
            pCamera->setId(i);                    // set id for each camera vertex

            // remeber to add vertex into optimizer..
            optimizer->addVertex(pCamera);

        }

        // Set point vertex with initial value in the dataset.
        const double *raw_points = bal_problem->points();
        // const int point_block_size = bal_problem->point_block_size();
        for (int j = 0; j < num_points; ++j) {
            ConstVectorRef temVecPoint(raw_points + point_block_size * j, point_block_size);
            VertexPointBAL *pPoint = new VertexPointBAL();
            pPoint->setEstimate(temVecPoint);   // initial value for the point i..
            pPoint->setId(j + num_cameras);     // each vertex should have an unique id, no matter it is a camera vertex, or a point vertex

            // remeber to add vertex into optimizer..
            pPoint->setMarginalized(true);
            optimizer->addVertex(pPoint);
        }

        // Set edges for graph..
        const int num_observations = bal_problem->num_observations();
        const double *observations = bal_problem->observations();   // pointer for the first observation..

        for (int i = 0; i < num_observations; ++i) {
            EdgeObservationBAL *bal_edge = new EdgeObservationBAL();

            const int camera_id = bal_problem->camera_index()[i]; // get id for the camera;
            const int point_id = bal_problem->point_index()[i] + num_cameras; // get id for the point

            if (params.robustify) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                rk->setDelta(1.0);
                bal_edge->setRobustKernel(rk);
            }
            // set the vertex by the ids for an edge observation
            bal_edge->setVertex(0, dynamic_cast<VertexCameraBAL *>(optimizer->vertex(camera_id)));
            bal_edge->setVertex(1, dynamic_cast<VertexPointBAL *>(optimizer->vertex(point_id)));
            bal_edge->setInformation(Eigen::Matrix2d::Identity());
            bal_edge->setMeasurement(Eigen::Vector2d(observations[2 * i + 0], observations[2 * i + 1]));

            optimizer->addEdge(bal_edge);
        }

    }

    void BackendOptimization::WriteToBALProblem(BALProblem *bal_problem, g2o::SparseOptimizer *optimizer) {
        const int num_points = bal_problem->num_points();
        const int num_cameras = bal_problem->num_cameras();
        const int camera_block_size = bal_problem->camera_block_size();
        const int point_block_size = bal_problem->point_block_size();

        double *raw_cameras = bal_problem->mutable_cameras();
        for (int i = 0; i < num_cameras; ++i) {
            VertexCameraBAL *pCamera = dynamic_cast<VertexCameraBAL *>(optimizer->vertex(i));
            Eigen::VectorXd NewCameraVec = pCamera->estimate();
            memcpy(raw_cameras + i * camera_block_size, NewCameraVec.data(), sizeof(double) * camera_block_size);
        }

        double *raw_points = bal_problem->mutable_points();
        for (int j = 0; j < num_points; ++j) {
            VertexPointBAL *pPoint = dynamic_cast<VertexPointBAL *>(optimizer->vertex(j + num_cameras));
            Eigen::Vector3d NewPointVec = pPoint->estimate();
            memcpy(raw_points + j * point_block_size, NewPointVec.data(), sizeof(double) * point_block_size);
        }
    }

    void BackendOptimization::SetSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params, g2o::SparseOptimizer* optimizer)
    {
        BalBlockSolver* solver_ptr;

        g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver = 0;

        if(params.linear_solver == "dense_schur" ){
            linearSolver = new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType>();
        }
        else if(params.linear_solver == "sparse_schur"){
            linearSolver = new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
            dynamic_cast<g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);  // AMD ordering , only needed for sparse cholesky solver
        }

        solver_ptr = new BalBlockSolver(linearSolver);
        g2o::OptimizationAlgorithmWithHessian* solver;
        if(params.trust_region_strategy == "levenberg_marquardt"){
            solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        }
        else if(params.trust_region_strategy == "dogleg"){
            solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
        }
        else
        {
            std::cout << "Please check your trust_region_strategy parameter again.."<< std::endl;
            exit(EXIT_FAILURE);
        }

        optimizer->setAlgorithm(solver);
    }
}



