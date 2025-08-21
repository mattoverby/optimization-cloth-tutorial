// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#ifndef SOLVER_HPP
#define SOLVER_HPP 1

#include "Objective.hpp"
#include "ClothMesh.hpp"

namespace cloth
{

class Solver
{
public:

	// State variables
	Eigen::MatrixXd x; // positions [nx3]
	Eigen::MatrixXd v; // velocities [nx3]
	Eigen::MatrixXd x_start; // start of timestep [nx3]
	double spring_k;
	double bend_k;
	double collision_k;

	Solver() :
		spring_k(100),
		bend_k(100),
		collision_k(300)
		{}

	// Initalizes state variables, returns true if success
	bool initialize(const ClothMesh &cloth)
	{
		if (cloth.V.rows() == 0 || cloth.V.cols() != 3)
		{
			return false;
		}

		x = cloth.V;
		v = Eigen::MatrixXd::Zero(x.rows(), 3);
		x_start = x;
		return true;
	}

	// Computes the energy and gradient (if desired) of the objective
	double gradient(const ClothMesh &cloth, const Objective &objective,
		double dt, const Eigen::MatrixXd &xk, Eigen::MatrixXd &grad) const
	{
		double tot_energy = 0;

		// Momentum potential
		tot_energy += objective.momentum_gradient(
			xk, x_start, v, cloth.masses, dt, grad);

		// Stretch springs
		int num_edges = cloth.E.rows();
		for (int i=0; i<num_edges; ++i)
		{
			Eigen::RowVector2i e = cloth.E.row(i);
			tot_energy += objective.spring_gradient(
				xk, e[0], e[1], cloth.E0[i], spring_k, grad);
		}

		// Linear bend springs
		int num_bend = cloth.B.rows();
		for (int i=0; i<num_bend; ++i)
		{
			Eigen::RowVector2i b = cloth.B.row(i);
			tot_energy += objective.spring_gradient(
				xk, b[0], b[1], cloth.B0[i], bend_k, grad);
		}

		// Collision springs
		int num_verts = xk.rows();
		for (int i=0; i<num_verts; ++i)
		{
			tot_energy += objective.collision_gradient(
				xk, i, collision_k, grad);
		}

		return tot_energy;
	}

	double energy(const ClothMesh &cloth, const Objective &objective, double dt, const Eigen::MatrixXd &xk) const
	{
		Eigen::MatrixXd dummy; // grad computation ignored if grad.size != x.size
		return gradient(cloth, objective, dt, xk, dummy);
	}

	void solve(const ClothMesh &cloth, const Objective &objective, double dt)
	{
		using namespace Eigen;
		ClothAssert(x.rows() == cloth.V.rows() && x.cols() == 3);
		ClothAssert(v.rows() == x.rows() && v.cols() == x.cols());

		x_start = x; // cache start state
		v.col(1).array() += dt * -9.8; // gravity
		x = x_start + dt * v; // initial guess
		MatrixXd grad = MatrixXd::Zero(x.rows(), 3);
		MatrixXd xk = MatrixXd::Zero(x.rows(), 3);

		int max_iters = 400;
		for (int iter = 0; iter < max_iters; ++iter)
		{
			// Compute current energy and gradient
			xk = x; // store state at current iteration
			grad.setZero();
			double energy_k = gradient(cloth, objective, dt, x, grad);

			// Check for convergence
			if (grad.norm() < 0.01)
			{
				break;
			}

			// Take a full step, i.e. alpha=1
			x = xk - grad;

			// Line search until objective decreased
			double alpha = 1;
			while (energy(cloth, objective, dt, x) > energy_k)
			{
				alpha *= 0.5;
				x = xk - alpha * grad;
			}
		}
	}

};

}

#endif
