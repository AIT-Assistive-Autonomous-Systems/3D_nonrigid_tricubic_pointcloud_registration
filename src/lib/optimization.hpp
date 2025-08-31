#ifndef OPTIMIZATION_H_
#define OPTIMIZATION_H_

#include <Eigen/Sparse>

#include "correspondences.hpp"

struct OptimizationResults {
  bool success{};
  int num_observations{};
  int num_unknowns{};
};

class Optimization {
 public:
  Optimization();
  static OptimizationResults Solve(Correspondences& correspondences,
                                   const std::vector<double>& weights_zero_observations);

 private:
  static std::vector<Eigen::Triplet<double>> SparseIdentity(const int& n);
  static std::vector<Eigen::Triplet<double>> MultiplyWithComponentsOfNormalVectors(
      const std::vector<Eigen::Triplet<double>>& triplets_in, const Eigen::VectorXd& n_component);
  static void AddSubblockTriplets(const int& first_row, const int& first_col,
                                  const std::vector<Eigen::Triplet<double>>& subblock_triplets,
                                  std::vector<Eigen::Triplet<double>>& triplets);
};

#endif  // OPTIMIZATION_H_
