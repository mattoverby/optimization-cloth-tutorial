// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#ifndef OBJECTIVE_HPP
#define OBJECTIVE_HPP 1

#include "ClothAssert.hpp"
#include "ClothMesh.hpp"

namespace cloth {

class Objective
{
  public:
    // Computes and returns the Hookean energy of a spring with rest length r and stiffness k.
    // If x.rows == grad.rows, the gradients (f, -f) are added to grad.
    double spring_gradient(const Eigen::MatrixXd& x, int e0, int e1, double r, double k, Eigen::MatrixXd& grad) const
    {
        using namespace Eigen;
        Vector3d x0 = x.row(e0);
        Vector3d x1 = x.row(e1);
        Vector3d edge = (x0 - x1);
        double l = edge.norm();
        if (grad.rows() == x.rows() && l > 1e-12) {
            edge /= l;
            edge *= (l - r) * k;
            ClothAssert(edge.allFinite());
            grad.row(e0) += edge;
            grad.row(e1) -= edge;
        }
        double energy = 0.5 * k * (l - r) * (l - r); // (k/2)||l-l0||^2
        ClothAssert(std::isfinite(energy));
        return energy;
    }

    // Spring energies that push vertices out of collision.
    // We'll cheat a bit and hard code the sphere radius and center since we know it.
    double collision_gradient(const Eigen::MatrixXd& x, int idx, double k, Eigen::MatrixXd& grad) const
    {
        using namespace Eigen;
        Vector3d cent(0, 0, 0);
        double rad = 0.51;
        Vector3d xi = x.row(idx);
        Vector3d dir = (xi - cent);
        double l = dir.norm();
        if (l > rad) {
            return 0;
        }
        if (grad.rows() == x.rows() && l > 1e-12) {
            dir /= l;
            dir *= (l - rad) * k;
            ClothAssert(dir.allFinite());
            grad.row(idx) += dir;
        }
        double energy = 0.5 * k * (l - rad) * (l - rad); // (k/2)||l-l0||^2
        ClothAssert(std::isfinite(energy));
        return energy;
    }

    // Momentum potential: ||x-xbar||^2_M / (2dt^2)
    // Like springs above, if xrows == grad.rows, gradient is added.
    // Unlike springs, the momentum is a global term.
    inline double momentum_gradient(const Eigen::MatrixXd& x,
                                    const Eigen::MatrixXd& x_start,
                                    const Eigen::MatrixXd& v,
                                    const Eigen::VectorXd& masses,
                                    double dt,
                                    Eigen::MatrixXd& grad) const
    {
        using namespace Eigen;
        // Some of this can be precomputed to speed up evaluations
        MatrixXd x_bar = x_start + dt * v;
        MatrixXd dx = x - x_bar;
        MatrixXd M_dx = masses.asDiagonal() * dx;
        double dot = 0;
        for (int i = 0; i < 3; ++i) {
            dot += dx.col(i).dot(M_dx.col(i));
        }

        double energy = dot / (2.0 * dt * dt);
        if (grad.rows() == x.rows()) {
            M_dx *= (1.0 / (dt * dt));
            grad += M_dx;
        }
        ClothAssert(std::isfinite(energy));
        return energy;
    }
};

}

#endif
