#ifndef PTCLOUD_H
#define PTCLOUD_H

#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

#include "translationgrid.h"

class PtCloud {
 public:
  PtCloud(Eigen::MatrixXd X);

  void SetNormals(Eigen::VectorXd nx, Eigen::VectorXd ny, Eigen::VectorXd nz);
  void InitializeTranslationGrids(const double &voxel_size,
                                  const uint32_t &buffer_voxels,
                                  const std::vector<double> &grid_limits);
  void ImportTranslationGrids(const std::string &filepath);
  void ExportTranslationGrids(const std::string &filepath);
  void UpdateXt();
  void InitMatricesForUpdateXt();

  long NumPts();
  double x_min();
  double x_max();
  double y_min();
  double y_max();
  double z_min();
  double z_max();

  // Getters
  const Eigen::MatrixXd &X();
  const Eigen::MatrixXd &Xt();
  const Eigen::VectorXd &nx();
  const Eigen::VectorXd &ny();
  const Eigen::VectorXd &nz();
  TranslationGrid &x_translation_grid();
  TranslationGrid &y_translation_grid();
  TranslationGrid &z_translation_grid();

 private:
  Eigen::MatrixXd X_;
  Eigen::MatrixXd Xt_;

  // Point attributes
  Eigen::VectorXd nx_;
  Eigen::VectorXd ny_;
  Eigen::VectorXd nz_;

  // Translation grids
  TranslationGrid x_translation_grid_;
  TranslationGrid y_translation_grid_;
  TranslationGrid z_translation_grid_;
  Eigen::MatrixX3i X_voxel_idx_;
  Eigen::MatrixX3d Xn_voxel_;
  Eigen::Matrix<double, Eigen::Dynamic, 64> X_power_;
};

struct HeaderInfo {
  char identifier[10]{"gbpcm"};
  int fileversion{1};
  const int length{1000};  // bytes
};

#endif  // PTCLOUD_H
