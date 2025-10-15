// Copyright Matt Overby 2021.
// Distributed under the MIT License.

#ifndef CLOTHMESH_HPP
#define CLOTHMESH_HPP 1

#include "ClothAssert.hpp"
#include <Eigen/Dense>
#include <map>

namespace cloth {

class ClothMesh
{
  public:
    Eigen::MatrixXd V;        // rest vertices [nx3]
    Eigen::MatrixXi F;        // faces [fx3]
    Eigen::MatrixXi E;        // edges [ex3]
    Eigen::VectorXd E0;       // edge rest length
    Eigen::MatrixXi B;        // bend edges [bx3]
    Eigen::VectorXd B0;       // bend rest length
    Eigen::VectorXd masses;   // per-vertex masses
    Eigen::VectorXi V_offset; // per-mesh index into V
    Eigen::VectorXi F_offset; // per-mesh index into F

    // Adds a mesh to the collection of deformables and returns mesh number
    inline int add_mesh(const Eigen::MatrixXd& inV, const Eigen::MatrixXi& inF)
    {
        int mesh_idx = -1;
        ClothAssert(inV.rows() > 0 && inV.cols() == 3);
        ClothAssert(inF.rows() > 0 && inF.cols() == 3);

        // First mesh?
        if (V_offset.size() == 0) {
            using namespace Eigen;
            V_offset = VectorXi::Zero(2);
            F_offset = VectorXi::Zero(2);
            V_offset[1] = inV.rows();
            F_offset[1] = inF.rows();
            V = inV;
            F = inF;
            mesh_idx = 0;
        } else {
            mesh_idx = V_offset.size() - 1;
            V_offset.conservativeResize(mesh_idx + 2);
            F_offset.conservativeResize(mesh_idx + 2);
            V_offset[mesh_idx + 1] = inV.rows();
            F_offset[mesh_idx + 1] = inF.rows();
            V.conservativeResize(V.rows() + inV.rows(), 3);
            V.bottomRows(inV.rows()) = inV;
            F.conservativeResize(F.rows() + inF.rows(), 3);
            F.bottomRows(inF.rows()) = inF;
        }

        masses = Eigen::VectorXd::Constant(V.rows(), 0.1 / V.rows());

        compute_edges();
        return mesh_idx;
    }

    // Computes edges and cross-edges from faces
    inline void compute_edges()
    {
        using namespace Eigen;
        std::map<std::pair<int, int>, std::pair<int, int>> edges; // [edge, bend]
        typedef std::map<std::pair<int, int>, std::pair<int, int>>::iterator EdgeIter;
        int n_bend = 0; // keep track of valid bend edges (non-border)

        int nf = F.rows();
        for (int i = 0; i < nf; ++i) {
            for (int j = 0; j < 3; ++j) {
                std::pair<int, int> edge = std::make_pair(F(i, j), F(i, (j + 1) % 3));
                int adj = F(i, (j + 2) % 3);
                if (edge.second < edge.first) {
                    std::swap(edge.first, edge.second);
                }

                EdgeIter it = edges.find(edge);
                if (it == edges.end()) {
                    edges[edge] = std::make_pair(adj, -1);
                } else {
                    n_bend++;
                    std::pair<int, int>& adj_pair = it->second;
                    adj_pair.second = adj;
                    if (adj_pair.second < adj_pair.first) {
                        std::swap(adj_pair.first, adj_pair.second);
                    }
                }
            }
        }

        // Loop over map and create eigen buffers
        E = MatrixXi::Zero(edges.size(), 2);
        E0 = VectorXd::Constant(edges.size(), -1);
        B = MatrixXi::Zero(n_bend, 2);
        B0 = VectorXd::Constant(n_bend, -1);

        int edge_idx = 0;
        int bend_idx = 0;
        for (EdgeIter it = edges.begin(); it != edges.end(); ++it, ++edge_idx) {
            const std::pair<int, int>& e = it->first;
            E(edge_idx, 0) = e.first;
            E(edge_idx, 1) = e.second;
            E0[edge_idx] = (V.row(e.first) - V.row(e.second)).norm();

            const std::pair<int, int>& b = it->second;
            if (b.first >= 0 && b.second >= 0) {
                ClothAssert(bend_idx < B.rows());
                B(bend_idx, 0) = b.first;
                B(bend_idx, 1) = b.second;
                B0[bend_idx] = (V.row(b.first) - V.row(b.second)).norm();
                bend_idx++;
            }
        }

        // Check to make sure we set every edge
        ClothAssert(E0.minCoeff() >= 0);
        ClothAssert(B0.minCoeff() >= 0);

    } // end compute edges
};

}

#endif
