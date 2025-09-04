#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <tuple>
#include <vector>

typedef Eigen::Matrix<double, 64, 1> Vector64d;
typedef Eigen::Matrix<int, 64, 1> Vector64i;
typedef Eigen::Matrix<int, 8, 1> Vector8i;

struct GridVals {
  double f{0};
  double fx{0};
  double fy{0};
  double fz{0};
  double fxy{0};
  double fxz{0};
  double fyz{0};
  double fxyz{0};
};

struct GridIdxAdj {
  int f{0};
  int fx{0};
  int fy{0};
  int fz{0};
  int fxy{0};
  int fxz{0};
  int fyz{0};
  int fxyz{0};
};

class TranslationGrid {
 public:
  void Initialize(const Eigen::RowVector3d& grid_origin, const int& x_num_voxels,
                  const int& y_num_voxels, const int& z_num_voxels, const double& voxel_size,
                  const int& first_idx_adj);
  Eigen::VectorXd p(const Eigen::MatrixX3d& X);
  // This version of p() can be used to save computation time if >1 translation grid is used, e.g.
  // for x, y, z
  Eigen::VectorXd p(const Eigen::MatrixX3d& X,
                    const Eigen::Matrix<double, Eigen::Dynamic, 64>& X_power,
                    const Eigen::MatrixX3i& X_voxel_idx);
  std::vector<Eigen::Triplet<double>> J(const Eigen::MatrixX3d& X);
  void UpdateAllGridValsFromVector(const Eigen::VectorXd& grid_vals_new);
  void UpdateVoxelGridVals(const int& x_voxel_idx, const int& y_voxel_idx, const int& z_voxel_idx,
                           const GridVals grid_vals_new);
  static Eigen::Matrix<double, Eigen::Dynamic, 64> Compute_X_power(
      const Eigen::MatrixX3d& Xn_voxel);
  std::tuple<Eigen::MatrixX3i, Eigen::MatrixX3d> GetGridReference(const Eigen::MatrixX3d& X);

  // Getters
  const Eigen::RowVector3d& grid_origin() const;
  const double& voxel_size() const;
  const int& x_num_voxels() const;
  const int& y_num_voxels() const;
  const int& z_num_voxels() const;
  const int& num_grid_vals() const;
  const int& min_idx_adj() const;
  const int& max_idx_adj() const;
  const std::vector<std::vector<std::vector<GridVals>>>& grid_vals() const;

 private:
  std::tuple<Vector64d, Vector64i> Get_f(const Eigen::RowVector3i& X_voxel_idx);

  Eigen::RowVector3d grid_origin_;
  double voxel_size_;
  std::vector<std::vector<std::vector<GridVals>>> grid_vals_;
  std::vector<std::vector<std::vector<GridIdxAdj>>> grid_idx_adj_;
  Eigen::Matrix<double, 64, 64> inv_A_;
  int x_num_voxels_;
  int y_num_voxels_;
  int z_num_voxels_;
  int num_grid_vals_;
  int min_idx_adj_;
  int max_idx_adj_;
};