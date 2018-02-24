//
// Created by Feng,Yan on 2018/2/24.
//

#include "backend_optimization.h"
#include "config.h"

void SolveProblem(const char *filename, const BundleParams &params) {

    std::shared_ptr<BackendOptimization> beop_ptr(new BackendOptimization());

    BALProblem bal_problem(filename);

    // show some information here ...
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observatoins. " << std::endl;

    // store the initial 3D cloud points and camera pose..
    if (!params.initial_ply.empty()) {
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    std::cout << "beginning problem..." << std::endl;

    // add some noise for the intial value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma,
                        params.point_sigma);

    std::cout << "Normalization complete..." << std::endl;

    g2o::SparseOptimizer optimizer;
    beop_ptr->SetSolverOptionsFromFlags(&bal_problem, params, &optimizer);
    beop_ptr->BuildProblem(&bal_problem, &optimizer, params);

    std::cout << "begin optimizaiton .." << std::endl;
    // perform the optimizaiton
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(params.num_iterations);

    std::cout << "optimization complete.. " << std::endl;
    // write the optimized data into BALProblem class
    beop_ptr->WriteToBALProblem(&bal_problem, &optimizer);

    // write the result into a .ply file.
    if (!params.final_ply.empty()) {
        bal_problem.WriteToPLYFile(params.final_ply);
    }

}

int main(int argc, char **argv) {

    slam::BundleParams params(argc, argv);  // set the parameters here.

    if (params.input.empty()) {
        slam::Config::setParameterFile(argv[1]);
        string default_file_path = slam::Config::get<string>("backend_dataset_path");
        params.input = default_file_path;
    }

    SolveProblem(params.input.c_str(), params);

    return 0;
}