#include "optimization.hpp"

Optimization::Optimization() = default;

OptimizationResults Optimization::Solve(Correspondences& correspondences,
                                        const std::vector<double>& weights_zero_observations) {
  OptimizationResults optimization_results{};  // returned

  CorrespondencesPointsWithAttributes X{correspondences.GetCorrespondences()};

  auto J_pc_mov_x_triplets{correspondences.pc_mov().x_translation_grid().J(X.pc_mov_X)};
  auto J_pc_mov_y_triplets{correspondences.pc_mov().y_translation_grid().J(X.pc_mov_X)};
  auto J_pc_mov_z_triplets{correspondences.pc_mov().z_translation_grid().J(X.pc_mov_X)};

  auto J_pc_mov_x_nx_triplets{
      Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_x_triplets, X.pc_fix_nx)};
  auto J_pc_mov_y_ny_triplets{
      Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_y_triplets, X.pc_fix_ny)};
  auto J_pc_mov_z_nz_triplets{
      Optimization::MultiplyWithComponentsOfNormalVectors(J_pc_mov_z_triplets, X.pc_fix_nz)};

  J_pc_mov_x_triplets.clear();
  J_pc_mov_y_triplets.clear();
  J_pc_mov_z_triplets.clear();

  int num_unknowns{correspondences.pc_mov().x_translation_grid().num_grid_vals() +
                   correspondences.pc_mov().y_translation_grid().num_grid_vals() +
                   correspondences.pc_mov().z_translation_grid().num_grid_vals()};

  auto J_direct_obs_triplets(Optimization::SparseIdentity(num_unknowns));

  int num_observations{X.num + num_unknowns};

  std::vector<Eigen::Triplet<double>> J_triplets;
  J_triplets.reserve(J_pc_mov_x_nx_triplets.size() + J_pc_mov_y_ny_triplets.size() +
                     J_pc_mov_z_nz_triplets.size() + J_direct_obs_triplets.size());

  // clang-format off
  Optimization::AddSubblockTriplets(0,
                      0,
                      J_pc_mov_x_nx_triplets,
                      J_triplets);
  Optimization::AddSubblockTriplets(0,
                      0,
                      J_pc_mov_y_ny_triplets,
                      J_triplets);
  Optimization::AddSubblockTriplets(0,
                      0,
                      J_pc_mov_z_nz_triplets,
                      J_triplets);
  Optimization::AddSubblockTriplets(X.num,
                      0,
                      J_direct_obs_triplets,
                      J_triplets);
  // clang-format on

  Eigen::SparseMatrix<double> J(num_observations, num_unknowns);
  J.setFromTriplets(J_triplets.begin(), J_triplets.end());

  Eigen::VectorXd p(num_observations);
  // Todo Use weights_zero_observations[0] to weights_zero_observations[3] for observation of
  // f,fx,fy,fz,...
  p << Eigen::VectorXd::Ones(correspondences.num()),
      Eigen::VectorXd::Ones(num_unknowns) * weights_zero_observations[0];
  auto P{p.asDiagonal()};

  Eigen::VectorXd b(num_observations);
  Eigen::VectorXd b0(num_observations);
  b = Eigen::VectorXd::Zero(num_observations);
  b0 << correspondences.point_to_plane_dists().dists, Eigen::VectorXd::Zero(num_unknowns);
  auto l{b - b0};

  // Solve!
  Eigen::VectorXd xhat(num_unknowns);
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double>> solver;
  solver.compute(J.transpose() * P * J);
  if (solver.info() != Eigen::Success) {
    optimization_results.success = false;
    return optimization_results;
  }
  xhat = solver.solve(J.transpose() * P * l);
  if (solver.info() != Eigen::Success) {
    optimization_results.success = false;
    return optimization_results;
  }
  optimization_results.success = true;

  // auto v{J * xhat - l};

  // Save estimated unknowns to translation grids
  correspondences.pc_mov().x_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().y_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().z_translation_grid().UpdateAllGridValsFromVector(xhat);
  correspondences.pc_mov().UpdateXt();
  correspondences.ComputeDists();

  optimization_results.num_observations = num_observations;
  optimization_results.num_unknowns = num_unknowns;

  return optimization_results;
}

std::vector<Eigen::Triplet<double>> Optimization::SparseIdentity(const int& n) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(n);
  for (int i = 0; i < n; i++) {
    triplets.emplace_back(i, i, 1.0);
  }
  return triplets;
}

std::vector<Eigen::Triplet<double>> Optimization::MultiplyWithComponentsOfNormalVectors(
    const std::vector<Eigen::Triplet<double>>& triplets_in, const Eigen::VectorXd& n_component) {
  std::vector<Eigen::Triplet<double>> triplets_out;
  triplets_out.reserve(triplets_in.size());
  for (auto const& triplet : triplets_in) {
    int row{triplet.row()};
    int col{triplet.col()};
    double val{triplet.value() * n_component(triplet.row())};
    triplets_out.emplace_back(row, col, val);
  }

  return triplets_out;
}

void Optimization::AddSubblockTriplets(const int& first_row, const int& first_col,
                                       const std::vector<Eigen::Triplet<double>>& subblock_triplets,
                                       std::vector<Eigen::Triplet<double>>& triplets) {
  for (auto const& triplet : subblock_triplets) {
    int row{first_row + triplet.row()};
    int col{first_col + triplet.col()};
    double val{triplet.value()};
    triplets.emplace_back(row, col, val);
  }
}
