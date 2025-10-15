// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#include "Solver.hpp"
#include <Eigen/Geometry>
#include <igl/readOBJ.h>
#include <iostream>

using namespace Eigen;
using namespace cloth;

int
main(int argc, char* argv[])
{
    (void)(argc);
    (void)(argv);

    std::string plane = CLOTH_ROOT_DIR "/data/plane.obj";
    std::string sphere = CLOTH_ROOT_DIR "/data/sphere.obj";
    MatrixXd V, cV;
    MatrixXi F, cF;
    if (!igl::readOBJ(plane, V, F) || !igl::readOBJ(sphere, cV, cF)) {
        return EXIT_FAILURE;
    }

    // Move cloth above the sphere
    V.col(1).array() += 0.5;

    // Create cloth object
    ClothMesh cloth;
    cloth.add_mesh(V, F);

    // Objective function
    Objective objective;

    // Create solver
    double dt = 1.0 / 24.0;
    Solver solver;

    // 20 iters won't converge but it will test
    // all the functions are operational.
    solver.max_solver_iter = 20;

    if (!solver.initialize(cloth)) {
        return EXIT_FAILURE;
    }

    int timesteps = 10;
    for (int step = 0; step < timesteps; ++step) {
        solver.solve(cloth, objective, dt); // solve the time step
    }

    return EXIT_SUCCESS;
}
