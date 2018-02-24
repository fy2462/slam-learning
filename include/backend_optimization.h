//
// Created by Feng,Yan on 2018/2/24.
//

#include <Eigen/StdVector>

#include <iostream>
#include <unordered_set>

#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"

#include "beop/BundleParams.h"
#include "beop/BALProblem.h"
#include "g2o/g2o_types.h"

#ifndef SLAM_LEARNING_BACKEND_OPTIMIZATION_H
#define SLAM_LEARNING_BACKEND_OPTIMIZATION_H

using namespace Eigen;
using namespace std;
using namespace slam;

namespace slam {

    typedef Eigen::Map<Eigen::VectorXd> VectorRef;
    typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3> > BalBlockSolver;

    class BackendOptimization {
    public:

        BackendOptimization() {};

        ~BackendOptimization() {};

        void BuildProblem(const BALProblem *bal_problem, g2o::SparseOptimizer *optimizer, const BundleParams &params);

        void WriteToBALProblem(BALProblem *bal_problem, g2o::SparseOptimizer *optimizer);

        void SetSolverOptionsFromFlags(BALProblem *bal_problem, const BundleParams &params, g2o::SparseOptimizer *optimizer);

    };
}

#endif //SLAM_LEARNING_BACKEND_OPTIMIZATION_H
