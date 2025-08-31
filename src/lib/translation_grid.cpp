#include "translation_grid.hpp"

#include <stdexcept>

void TranslationGrid::Initialize(const Eigen::RowVector3d& grid_origin, const int& x_num_voxels,
                                 const int& y_num_voxels, const int& z_num_voxels,
                                 const double& voxel_size, const int& first_idx_adj) {
  grid_origin_ = grid_origin;
  voxel_size_ = voxel_size;

  x_num_voxels_ = x_num_voxels;
  y_num_voxels_ = y_num_voxels;
  z_num_voxels_ = z_num_voxels;

  // clang-format off
  grid_vals_ = std::vector<std::vector<std::vector<GridVals>>>
      (x_num_voxels+1,std::vector<std::vector<GridVals>>
      (y_num_voxels+1,std::vector<GridVals>
      (z_num_voxels+1, GridVals{})));

  grid_idx_adj_ = std::vector<std::vector<std::vector<GridIdxAdj>>>
      (x_num_voxels+1,std::vector<std::vector<GridIdxAdj>>
      (y_num_voxels+1,std::vector<GridIdxAdj>
      (z_num_voxels+1, GridIdxAdj{})));
  // clang-format on

  int idx_adj{first_idx_adj};
  for (int x_voxel_idx = 0; x_voxel_idx < x_num_voxels + 1; x_voxel_idx++)
    for (int y_voxel_idx = 0; y_voxel_idx < y_num_voxels + 1; y_voxel_idx++)
      for (int z_voxel_idx = 0; z_voxel_idx < z_num_voxels + 1; z_voxel_idx++) {
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f = idx_adj;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx = idx_adj + 1;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy = idx_adj + 2;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz = idx_adj + 3;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy = idx_adj + 4;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz = idx_adj + 5;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz = idx_adj + 6;
        grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz = idx_adj + 7;
        idx_adj += 8;
      }
  min_idx_adj_ = first_idx_adj;
  max_idx_adj_ = idx_adj - 1;

  num_grid_vals_ = (x_num_voxels + 1) * (y_num_voxels + 1) * (z_num_voxels + 1) * 8;

  // clang-format off
  const int inv_A_array[64][64] =
      {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {-3,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {2,0,0,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
       {-3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {9,0,-9,0,-9,0,9,0,0,0,0,0,0,0,0,0,6,0,3,0,-6,0,-3,0,6,0,-6,0,3,0,-3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,2,0,2,0,1,0,0,0,0,0,0,0,0,0},
       {-6,0,6,0,6,0,-6,0,0,0,0,0,0,0,0,0,-4,0,-2,0,4,0,2,0,-3,0,3,0,-3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-1,0,-2,0,-1,0,0,0,0,0,0,0,0,0},
       {2,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {-6,0,6,0,6,0,-6,0,0,0,0,0,0,0,0,0,-3,0,-3,0,3,0,3,0,-4,0,4,0,-2,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-2,0,-1,0,-1,0,0,0,0,0,0,0,0,0},
       {4,0,-4,0,-4,0,4,0,0,0,0,0,0,0,0,0,2,0,2,0,-2,0,-2,0,2,0,-2,0,2,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,-3,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,0,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,2,0,0,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,0,0,-1,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0},
       {0,0,0,0,0,0,0,0,-3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,0,-1,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,9,0,-9,0,-9,0,9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,0,3,0,-6,0,-3,0,6,0,-6,0,3,0,-3,0,0,0,0,0,0,0,0,0,4,0,2,0,2,0,1,0},
       {0,0,0,0,0,0,0,0,-6,0,6,0,6,0,-6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-4,0,-2,0,4,0,2,0,-3,0,3,0,-3,0,3,0,0,0,0,0,0,0,0,0,-2,0,-1,0,-2,0,-1,0},
       {0,0,0,0,0,0,0,0,2,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,-6,0,6,0,6,0,-6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,0,-3,0,3,0,3,0,-4,0,4,0,-2,0,2,0,0,0,0,0,0,0,0,0,-2,0,-2,0,-1,0,-1,0},
       {0,0,0,0,0,0,0,0,4,0,-4,0,-4,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,2,0,-2,0,-2,0,2,0,-2,0,2,0,-2,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0},
       {-3,3,0,0,0,0,0,0,-2,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {9,-9,0,0,-9,9,0,0,6,3,0,0,-6,-3,0,0,0,0,0,0,0,0,0,0,6,-6,0,0,3,-3,0,0,0,0,0,0,0,0,0,0,4,2,0,0,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {-6,6,0,0,6,-6,0,0,-4,-2,0,0,4,2,0,0,0,0,0,0,0,0,0,0,-3,3,0,0,-3,3,0,0,0,0,0,0,0,0,0,0,-2,-1,0,0,-2,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-2,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-3,3,0,0,0,0,0,0,-2,-1,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,-9,0,0,-9,9,0,0,0,0,0,0,0,0,0,0,6,3,0,0,-6,-3,0,0,0,0,0,0,0,0,0,0,6,-6,0,0,3,-3,0,0,4,2,0,0,2,1,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-6,6,0,0,6,-6,0,0,0,0,0,0,0,0,0,0,-4,-2,0,0,4,2,0,0,0,0,0,0,0,0,0,0,-3,3,0,0,-3,3,0,0,-2,-1,0,0,-2,-1,0,0},
       {9,-9,-9,9,0,0,0,0,6,3,-6,-3,0,0,0,0,6,-6,3,-3,0,0,0,0,0,0,0,0,0,0,0,0,4,2,2,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,9,-9,-9,9,0,0,0,0,0,0,0,0,0,0,0,0,6,3,-6,-3,0,0,0,0,6,-6,3,-3,0,0,0,0,4,2,2,1,0,0,0,0},
       {-27,27,27,-27,27,-27,-27,27,-18,-9,18,9,18,9,-18,-9,-18,18,-9,9,18,-18,9,-9,-18,18,18,-18,-9,9,9,-9,-12,-6,-6,-3,12,6,6,3,-12,-6,12,6,-6,-3,6,3,-12,12,-6,6,-6,6,-3,3,-8,-4,-4,-2,-4,-2,-2,-1},
       {18,-18,-18,18,-18,18,18,-18,12,6,-12,-6,-12,-6,12,6,12,-12,6,-6,-12,12,-6,6,9,-9,-9,9,9,-9,-9,9,8,4,4,2,-8,-4,-4,-2,6,3,-6,-3,6,3,-6,-3,6,-6,3,-3,6,-6,3,-3,4,2,2,1,4,2,2,1},
       {-6,6,6,-6,0,0,0,0,-4,-2,4,2,0,0,0,0,-3,3,-3,3,0,0,0,0,0,0,0,0,0,0,0,0,-2,-1,-2,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-6,6,6,-6,0,0,0,0,0,0,0,0,0,0,0,0,-4,-2,4,2,0,0,0,0,-3,3,-3,3,0,0,0,0,-2,-1,-2,-1,0,0,0,0},
       {18,-18,-18,18,-18,18,18,-18,12,6,-12,-6,-12,-6,12,6,9,-9,9,-9,-9,9,-9,9,12,-12,-12,12,6,-6,-6,6,6,3,6,3,-6,-3,-6,-3,8,4,-8,-4,4,2,-4,-2,6,-6,6,-6,3,-3,3,-3,4,2,4,2,2,1,2,1},
       {-12,12,12,-12,12,-12,-12,12,-8,-4,8,4,8,4,-8,-4,-6,6,-6,6,6,-6,6,-6,-6,6,6,-6,-6,6,6,-6,-4,-2,-4,-2,4,2,4,2,-4,-2,4,2,-4,-2,4,2,-3,3,-3,3,-3,3,-3,3,-2,-1,-2,-1,-2,-1,-2,-1},
       {2,-2,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {-6,6,0,0,6,-6,0,0,-3,-3,0,0,3,3,0,0,0,0,0,0,0,0,0,0,-4,4,0,0,-2,2,0,0,0,0,0,0,0,0,0,0,-2,-2,0,0,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {4,-4,0,0,-4,4,0,0,2,2,0,0,-2,-2,0,0,0,0,0,0,0,0,0,0,2,-2,0,0,2,-2,0,0,0,0,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,-2,0,0,0,0,0,0,1,1,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-6,6,0,0,6,-6,0,0,0,0,0,0,0,0,0,0,-3,-3,0,0,3,3,0,0,0,0,0,0,0,0,0,0,-4,4,0,0,-2,2,0,0,-2,-2,0,0,-1,-1,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,-4,0,0,-4,4,0,0,0,0,0,0,0,0,0,0,2,2,0,0,-2,-2,0,0,0,0,0,0,0,0,0,0,2,-2,0,0,2,-2,0,0,1,1,0,0,1,1,0,0},
       {-6,6,6,-6,0,0,0,0,-3,-3,3,3,0,0,0,0,-4,4,-2,2,0,0,0,0,0,0,0,0,0,0,0,0,-2,-2,-1,-1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-6,6,6,-6,0,0,0,0,0,0,0,0,0,0,0,0,-3,-3,3,3,0,0,0,0,-4,4,-2,2,0,0,0,0,-2,-2,-1,-1,0,0,0,0},
       {18,-18,-18,18,-18,18,18,-18,9,9,-9,-9,-9,-9,9,9,12,-12,6,-6,-12,12,-6,6,12,-12,-12,12,6,-6,-6,6,6,6,3,3,-6,-6,-3,-3,6,6,-6,-6,3,3,-3,-3,8,-8,4,-4,4,-4,2,-2,4,4,2,2,2,2,1,1},
       {-12,12,12,-12,12,-12,-12,12,-6,-6,6,6,6,6,-6,-6,-8,8,-4,4,8,-8,4,-4,-6,6,6,-6,-6,6,6,-6,-4,-4,-2,-2,4,4,2,2,-3,-3,3,3,-3,-3,3,3,-4,4,-2,2,-4,4,-2,2,-2,-2,-1,-1,-2,-2,-1,-1},
       {4,-4,-4,4,0,0,0,0,2,2,-2,-2,0,0,0,0,2,-2,2,-2,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
       {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,-4,-4,4,0,0,0,0,0,0,0,0,0,0,0,0,2,2,-2,-2,0,0,0,0,2,-2,2,-2,0,0,0,0,1,1,1,1,0,0,0,0},
       {-12,12,12,-12,12,-12,-12,12,-6,-6,6,6,6,6,-6,-6,-6,6,-6,6,6,-6,6,-6,-8,8,8,-8,-4,4,4,-4,-3,-3,-3,-3,3,3,3,3,-4,-4,4,4,-2,-2,2,2,-4,4,-4,4,-2,2,-2,2,-2,-2,-2,-2,-1,-1,-1,-1},
       {8,-8,-8,8,-8,8,8,-8,4,4,-4,-4,-4,-4,4,4,4,-4,4,-4,-4,4,-4,4,4,-4,-4,4,4,-4,-4,4,2,2,2,2,-2,-2,-2,-2,2,2,-2,-2,2,2,-2,-2,2,-2,2,-2,2,-2,2,-2,1,1,1,1,1,1,1,1}};
  // clang-format on

  for (int row = 0; row < 64; row++)
    for (int col = 0; col < 64; col++) {
      inv_A_(row, col) = inv_A_array[row][col];
    }
}

std::tuple<Eigen::MatrixX3i, Eigen::MatrixX3d> TranslationGrid::GetGridReference(
    const Eigen::MatrixX3d& X) {
  int64_t num_obs{X.rows()};

  Eigen::MatrixX3i X_voxel_idx(num_obs, 3);  // returned
  Eigen::MatrixX3d Xn_voxel(num_obs, 3);     // returned

  Eigen::RowVector3d X_grid{};   // intermediate result
  Eigen::RowVector3d X_voxel{};  // intermediate result

  for (int i = 0; i < num_obs; i++) {
    X_grid(0) = X(i, 0) - grid_origin_(0);
    X_grid(1) = X(i, 1) - grid_origin_(1);
    X_grid(2) = X(i, 2) - grid_origin_(2);

    X_voxel_idx(i, 0) = floor(X_grid(0) / voxel_size_);
    X_voxel_idx(i, 1) = floor(X_grid(1) / voxel_size_);
    X_voxel_idx(i, 2) = floor(X_grid(2) / voxel_size_);

    // Check bounds and handle points outside the transformation domain
    if (X_voxel_idx(i, 0) < 0 || X_voxel_idx(i, 0) >= x_num_voxels_ || X_voxel_idx(i, 1) < 0 ||
        X_voxel_idx(i, 1) >= y_num_voxels_ || X_voxel_idx(i, 2) < 0 ||
        X_voxel_idx(i, 2) >= z_num_voxels_) {
      throw std::out_of_range(
          "Point (" + std::to_string(X(i, 0)) + ", " + std::to_string(X(i, 1)) + ", " +
          std::to_string(X(i, 2)) +
          ") is outside the transformation domain. Grid bounds: xmin/ymin/zmin = " +
          std::to_string(grid_origin_(0)) + "/" + std::to_string(grid_origin_(1)) + "/" +
          std::to_string(grid_origin_(2)) +
          ", xmax/ymax/zmax = " + std::to_string(grid_origin_(0) + x_num_voxels_ * voxel_size_) +
          "/" + std::to_string(grid_origin_(1) + y_num_voxels_ * voxel_size_) + "/" +
          std::to_string(grid_origin_(2) + z_num_voxels_ * voxel_size_));
    }

    // Reduce index by 1 for points exactly on the upper boundaries of the grid
    if (X_voxel_idx(i, 0) == x_num_voxels_) X_voxel_idx(i, 0) -= 1;
    if (X_voxel_idx(i, 1) == y_num_voxels_) X_voxel_idx(i, 1) -= 1;
    if (X_voxel_idx(i, 2) == z_num_voxels_) X_voxel_idx(i, 2) -= 1;

    X_voxel(0) = X_grid(0) - static_cast<double>(X_voxel_idx(i, 0)) * voxel_size_;
    X_voxel(1) = X_grid(1) - static_cast<double>(X_voxel_idx(i, 1)) * voxel_size_;
    X_voxel(2) = X_grid(2) - static_cast<double>(X_voxel_idx(i, 2)) * voxel_size_;

    Xn_voxel(i, 0) = X_voxel(0) / voxel_size_;
    Xn_voxel(i, 1) = X_voxel(1) / voxel_size_;
    Xn_voxel(i, 2) = X_voxel(2) / voxel_size_;
  }

  return {X_voxel_idx, Xn_voxel};
}

std::tuple<Vector64d, Vector64i> TranslationGrid::Get_f(const Eigen::RowVector3i& X_voxel_idx) {
  Vector64d f_vals{};     // returned
  Vector64i f_idx_adj{};  // returned

  Vector8i dx_voxel_idx, dy_voxel_idx, dz_voxel_idx;
  dx_voxel_idx << 0, 1, 0, 1, 0, 1, 0, 1;
  dy_voxel_idx << 0, 0, 1, 1, 0, 0, 1, 1;
  dz_voxel_idx << 0, 0, 0, 0, 1, 1, 1, 1;

  for (int i = 0; i < 8; i++) {
    // clang-format off
    int x_voxel_idx{X_voxel_idx(0)+dx_voxel_idx(i)};
    int y_voxel_idx{X_voxel_idx(1)+dy_voxel_idx(i)};
    int z_voxel_idx{X_voxel_idx(2)+dz_voxel_idx(i)};

    f_vals(8*0+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f;
    f_vals(8*1+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx;
    f_vals(8*2+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy;
    f_vals(8*3+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz;
    f_vals(8*4+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy;
    f_vals(8*5+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz;
    f_vals(8*6+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz;
    f_vals(8*7+i) = grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz;

    f_idx_adj(8*0+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f;
    f_idx_adj(8*1+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx;
    f_idx_adj(8*2+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy;
    f_idx_adj(8*3+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz;
    f_idx_adj(8*4+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy;
    f_idx_adj(8*5+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz;
    f_idx_adj(8*6+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz;
    f_idx_adj(8*7+i) = grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz;
    // clang-format on
  }

  return {f_vals, f_idx_adj};
}

Eigen::VectorXd TranslationGrid::p(const Eigen::MatrixX3d& X) {
  int64_t num_points{X.rows()};

  Eigen::VectorXd p(num_points);

  auto [X_voxel_idx, Xn_voxel]{GetGridReference(X)};
  auto X_power{Compute_X_power(Xn_voxel)};

  for (int i = 0; i < num_points; i++) {
    auto [f_vals, coeff_cols]{Get_f(X_voxel_idx.row(i))};
    auto J_voxel{X_power.row(i) * inv_A_};
    p(i) = J_voxel * f_vals;
  }

  return p;
}

Eigen::VectorXd TranslationGrid::p(const Eigen::MatrixX3d& X,
                                   const Eigen::Matrix<double, Eigen::Dynamic, 64>& X_power,
                                   const Eigen::MatrixX3i& X_voxel_idx) {
  int64_t num_points{X.rows()};

  Eigen::VectorXd p(num_points);

  for (int i = 0; i < num_points; i++) {
    auto [f_vals, coeff_cols]{Get_f(X_voxel_idx.row(i))};
    auto J_voxel{X_power.row(i) * inv_A_};
    p(i) = J_voxel * f_vals;
  }

  return p;
}

Eigen::Matrix<double, Eigen::Dynamic, 64> TranslationGrid::Compute_X_power(
    const Eigen::MatrixX3d& Xn_voxel) {
  int64_t num_points{Xn_voxel.rows()};
  Eigen::Matrix<double, Eigen::Dynamic, 64> X_power(num_points, 64);
  int col{-1};
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 4; k++) {
        col++;
        X_power.col(col) = Xn_voxel.col(0).array().pow(double(i)) *
                           Xn_voxel.col(1).array().pow(double(j)) *
                           Xn_voxel.col(2).array().pow(double(k));
      }
    }
  }
  return X_power;
}

std::vector<Eigen::Triplet<double>> TranslationGrid::J(const Eigen::MatrixX3d& X) {
  auto [X_voxel_idx, Xn_voxel]{GetGridReference(X)};
  auto X_power{Compute_X_power(Xn_voxel)};

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(X.rows() * 64);

  Vector64d coeff_vals{};
  Vector64i coeff_rows{};

  for (int i = 0; i < X.rows(); i++) {
    coeff_vals = X_power.row(i) * inv_A_;
    coeff_rows.setConstant(i);
    auto [f_vals, coeff_cols]{Get_f(X_voxel_idx.row(i))};
    for (int j = 0; j < 64; j++) {
      triplets.emplace_back(coeff_rows(j), coeff_cols(j), coeff_vals(j));
    }
  }

  return triplets;
}

void TranslationGrid::UpdateAllGridValsFromVector(const Eigen::VectorXd& grid_vals_new) {
  for (int x_voxel_idx = 0; x_voxel_idx < x_num_voxels_ + 1; x_voxel_idx++)
    for (int y_voxel_idx = 0; y_voxel_idx < y_num_voxels_ + 1; y_voxel_idx++)
      for (int z_voxel_idx = 0; z_voxel_idx < z_num_voxels_ + 1; z_voxel_idx++) {
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz);
        grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz =
            grid_vals_new(grid_idx_adj_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz);
      }
}

void TranslationGrid::UpdateVoxelGridVals(const int& x_voxel_idx, const int& y_voxel_idx,
                                          const int& z_voxel_idx, const GridVals grid_vals_new) {
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].f = grid_vals_new.f;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fx = grid_vals_new.fx;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fy = grid_vals_new.fy;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fz = grid_vals_new.fz;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxy = grid_vals_new.fxy;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxz = grid_vals_new.fxz;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fyz = grid_vals_new.fyz;
  grid_vals_[x_voxel_idx][y_voxel_idx][z_voxel_idx].fxyz = grid_vals_new.fxyz;
}

const Eigen::RowVector3d& TranslationGrid::grid_origin() const { return grid_origin_; }
const double& TranslationGrid::voxel_size() const { return voxel_size_; }
const int& TranslationGrid::x_num_voxels() const { return x_num_voxels_; }
const int& TranslationGrid::y_num_voxels() const { return y_num_voxels_; }
const int& TranslationGrid::z_num_voxels() const { return z_num_voxels_; }
const int& TranslationGrid::num_grid_vals() const { return num_grid_vals_; }
const int& TranslationGrid::min_idx_adj() const { return min_idx_adj_; }
const int& TranslationGrid::max_idx_adj() const { return max_idx_adj_; }
const std::vector<std::vector<std::vector<GridVals>>>& TranslationGrid::grid_vals() const {
  return grid_vals_;
}
