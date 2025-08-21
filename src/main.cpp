// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <iostream>
#include <Eigen/Geometry>
#include "Solver.hpp"

using namespace Eigen;
using namespace cloth;

int main(int argc, char *argv[])
{
	(void)(argc);
	(void)(argv);

	std::string plane = CLOTH_ROOT_DIR "/data/plane.obj";
	std::string sphere = CLOTH_ROOT_DIR "/data/sphere.obj";
	MatrixXd V, sphereV;
	MatrixXi F, sphereF;
	if (!igl::readOBJ(plane, V, F) || !igl::readOBJ(sphere, sphereV, sphereF))
	{
		return EXIT_FAILURE;
	}

	// Move cloth above the sphere
	V.col(1).array() += 0.6;

	// Create cloth object
	ClothMesh cloth;
	cloth.add_mesh(V, F);

	// Objective function
	Objective objective;

	// Create solver
	double dt = 1.0 / 24.0;
	Solver solver;
	if (!solver.initialize(cloth))
	{
		return EXIT_FAILURE;
	}

	std::cout << "Press A to simulate, R to reset" << std::endl;

	// Create viewer
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_face_based(true);
	viewer.data().set_mesh(V, F);
	viewer.data().set_colors(RowVector3d(0.659, 0.847, 1));
	viewer.append_mesh();
	viewer.data(1).set_mesh(sphereV, sphereF);
	viewer.data(1).set_colors(RowVector3d(0.7, 0.7, 0.7));
	viewer.core().is_animating = false;

	viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer&, unsigned int key, int)->bool
	{
		char c = char(key);
		if (c == 'r')
		{
			solver.initialize(cloth);
		}
		return false;
	};

	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer&)->bool
	{
		if (viewer.core().is_animating)
		{
			solver.solve(cloth, objective, dt); // solve the time step
			V = solver.x; // update vertex positions
			viewer.data(0).set_mesh(V, F);
			viewer.data(0).compute_normals(); // update normals after defo
		}
		return false;
	};

	viewer.launch();

	return EXIT_SUCCESS;
}
