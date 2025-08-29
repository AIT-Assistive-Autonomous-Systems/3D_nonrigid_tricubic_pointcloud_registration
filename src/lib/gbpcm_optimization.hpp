#ifndef GBPCM_OPTIMIZATION_H_
#define GBPCM_OPTIMIZATION_H_

#include <Eigen/Sparse>
#include "correspondences.hpp"

struct OptimizationResults
{
  bool success{};
  int num_observations{};
  int num_unknowns{};
};

class GBPCMOptimization
{
 public:
  GBPCMOptimization();
  static OptimizationResults Solve(Correspondences& correspondences,
                                   const std::vector<double>& weights_zero_observations);

 private:
  static std::vector<Eigen::Triplet<double>> SparseIdentity(const int& n);
  static std::vector<Eigen::Triplet<double>> MultiplyWithComponentsOfNormalVectors(
      const std::vector<Eigen::Triplet<double>>& triplets_in, const Eigen::VectorXd& n_component);
  static void AddSubblockTriplets(const int& first_row,
                                  const int& first_col,
                                  const std::vector<Eigen::Triplet<double>>& subblock_triplets,
                                  std::vector<Eigen::Triplet<double>>& triplets);
};

#endif  // GBPCM_OPTIMIZATION_H_
