#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include "src/lib/correspondences.h"
#include "src/lib/gbpcm_optimization.h"
#include "src/lib/io_utils.h"
#include "src/lib/pt_cloud.h"
#include "src/prog/3rdparty/cxxopts/cxxopts.hpp"

struct CorrespondencesResults
{
  int num{};
  double mean_point_to_plane_dists_before_optimization{};
  double std_point_to_plane_dists_before_optimization{};
  double mean_point_to_plane_dists_after_optimization{};
  double std_point_to_plane_dists_after_optimization{};
};

struct IterationResults
{
  int it{};
  OptimizationResults optimization_results{};
  CorrespondencesResults correspondences_results{};
};

struct Params
{
  std::string fixed;
  std::string movable;
  std::string transform;
  double voxel_size;
  std::vector<double> grid_limits;
  uint32_t buffer_voxels;
  std::string matching_mode;
  uint32_t num_correspondences;
  double max_euclidean_distance;
  uint32_t num_iterations;
  std::vector<double> weights;
  std::string debug_dir;
  bool suppress_logging;
};

Params ParseUserInputs(int argc, char** argv);

void ReportIterationResults(const IterationResults& iteration_results);

int main(int argc, char** argv)
{
  try
  {
    Params params = ParseUserInputs(argc, argv);

    if (params.suppress_logging)
    {
      spdlog::set_level(spdlog::level::off);
    }

    spdlog::set_pattern("%v");
    spdlog::stopwatch sw;
    spdlog::info("Start of \"gbpcm\"");

    spdlog::info("Create point cloud objects");
    auto X_fix =
        ImportFileToMatrix(params.fixed, true, params.matching_mode == "id" ? true : false);
    auto X_mov =
        ImportFileToMatrix(params.movable, true, params.matching_mode == "id" ? true : false);
    auto pc_fix{PtCloud(X_fix.leftCols(3))};
    auto pc_mov{PtCloud(X_mov.leftCols(3))};
    pc_fix.SetNormals(X_fix.col(3), X_fix.col(4), X_fix.col(5));
    pc_mov.SetNormals(X_mov.col(3), X_mov.col(4), X_mov.col(5));
    if (params.matching_mode == "id")
    {
      pc_fix.SetCorrespondenceId(X_fix.col(X_fix.cols() - 1));
      pc_mov.SetCorrespondenceId(X_mov.col(X_mov.cols() - 1));
    }
    spdlog::info("  Fixed point cloud has {:d} points", pc_fix.NumPts());
    spdlog::info("  Movable point cloud has {:d} points", pc_mov.NumPts());

    spdlog::info("Initialize x/y/z translation grids for movable point cloud");
    pc_mov.InitializeTranslationGrids(params.voxel_size, params.buffer_voxels, params.grid_limits);
    pc_mov.InitMatricesForUpdateXt();
    spdlog::info("Each translation grid (including buffer voxels) has the properties:");
    spdlog::info(
        "  x_min/x_max/x_num_voxels = {:.3f}/{:.3f}/{:d}",
        pc_mov.x_translation_grid().grid_origin()(0),
        pc_mov.x_translation_grid().grid_origin()(0) +
            pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().x_num_voxels(),
        pc_mov.x_translation_grid().x_num_voxels());
    spdlog::info(
        "  y_min/y_max/y_num_voxels = {:.3f}/{:.3f}/{:d}",
        pc_mov.x_translation_grid().grid_origin()(1),
        pc_mov.x_translation_grid().grid_origin()(1) +
            pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().y_num_voxels(),
        pc_mov.x_translation_grid().y_num_voxels());
    spdlog::info(
        "  z_min/z_max/z_num_voxels = {:.3f}/{:.3f}/{:d}",
        pc_mov.x_translation_grid().grid_origin()(2),
        pc_mov.x_translation_grid().grid_origin()(2) +
            pc_mov.x_translation_grid().voxel_size() * pc_mov.x_translation_grid().z_num_voxels(),
        pc_mov.x_translation_grid().z_num_voxels());
    spdlog::info("  num_grid_vals = {:d}", pc_mov.x_translation_grid().num_grid_vals());

    spdlog::info("Selection of correspondences in fixed point cloud");
    Correspondences correspondences{pc_fix, pc_mov};
    correspondences.SelectPointsByRandomSampling(params.num_correspondences);
    auto idx_pc_fix{correspondences.GetSelectedPoints()};
    spdlog::info("Selected {:d} points in fixed point cloud", correspondences.num());

    auto debug_mode = (params.debug_dir != "");

    spdlog::info("Start iterative point cloud matching");
    IterationResults iteration_results{};
    for (uint32_t it = 0; it < params.num_iterations; it++)
    {
      iteration_results.it = it + 1;

      correspondences.SetSelectedPoints(idx_pc_fix);
      if (params.matching_mode == "nn")
      {
        correspondences.MatchPointsByNearestNeighbor();
      }
      else if (params.matching_mode == "id")
      {
        correspondences.MatchPointsByCorrespondenceId();
      }
      correspondences.RejectMaxEuclideanDistanceCriteria(params.max_euclidean_distance);
      correspondences.RejectStdMadCriteria();

      if (debug_mode)
      {
        char it_string[100];
        std::sprintf(it_string, "%03d", iteration_results.it);
        auto debug_file_name =
            params.debug_dir + "correspondences_it" + std::string(it_string) + ".poly";
        correspondences.ExportCorrespondences(debug_file_name);
      }

      iteration_results.correspondences_results.num = correspondences.num();
      iteration_results.correspondences_results.mean_point_to_plane_dists_before_optimization =
          correspondences.point_to_plane_dists_t().mean;
      iteration_results.correspondences_results.std_point_to_plane_dists_before_optimization =
          correspondences.point_to_plane_dists_t().std;

      GBPCMOptimization optimization{};
      iteration_results.optimization_results =
          GBPCMOptimization::Solve(correspondences, params.weights);

      if (iteration_results.optimization_results.success)
      {
        iteration_results.correspondences_results.mean_point_to_plane_dists_after_optimization =
            correspondences.point_to_plane_dists_t().mean;
        iteration_results.correspondences_results.std_point_to_plane_dists_after_optimization =
            correspondences.point_to_plane_dists_t().std;
        ReportIterationResults(iteration_results);
      }
      else
      {
        throw std::runtime_error("Optimization was not successful!");
      }
    }

    spdlog::info("Export of estimated translation grids to \"{}\"", params.transform);
    pc_mov.ExportTranslationGrids(params.transform);

    spdlog::info("Finished \"gbpcm\" in {:.3}s!", sw);
  }
  catch (const std::exception& e)
  {
    std::cerr << "Caught exception: " << e.what() << std::endl;
    return 1;
  }
  catch (...)
  {
    std::cerr << "Caught unknown exception." << std::endl;
    return 1;
  }

  return 0;
}

Params ParseUserInputs(int argc, char** argv)
{
  cxxopts::Options options("gbpcm", "Grid based point cloud matching.");

  // clang-format off
  options.add_options()
    ("f,fixed",
    "Path to fixed point cloud",
    cxxopts::value<std::string>())
    ("m,movable",
    "Path to movable point cloud",
    cxxopts::value<std::string>())
    ("t,transform",
    "Path to generated transform file. This file contains the estimated translation grids for "
    "the movable point cloud. The executable \"gbpcm-transform\" can be used to transform a "
    "point cloud with this transform file.",
    cxxopts::value<std::string>())
    ("v,voxel_size",
    "Voxel size of translation grids",
    cxxopts::value<double>()->default_value("1"))
    ("g,grid_limits",
    "Limits of translation grids to be defined as \"x_min,y_min,z_min,x_max,y_max,z_max\". Note "
    "that the extent of the grids in x,y,z must be an integer multiple of the voxel size. The "
    "grid limits are chosen automatically by passing \"0,0,0,0,0,0\".",
    cxxopts::value<std::vector<double>>()->default_value("0,0,0,0,0,0"))
    ("b,buffer_voxels",
    "Number of voxels to be used as buffer around the translation grids",
    cxxopts::value<uint32_t>()->default_value("2"))
    ("a,matching_mode",
    "Matching mode for correspondences. Available modes are \"nn\" (nearest neighbor) and \"id\" (correspondence_id).",
    cxxopts::value<std::string>()->default_value("nn"))
    ("n,num_correspondences",
    "Number of correspondences",
    cxxopts::value<uint32_t>()->default_value("10000"))
    ("e,max_euclidean_distance",
    "Maximum euclidean distance between corresponding points",
    cxxopts::value<double>()->default_value("1"))
    ("i,num_iterations",
    "Number of iterations",
    cxxopts::value<uint32_t>()->default_value("5"))
    ("w,weights",
    "Weights of zero observations as list for \"f,fx/fy/fz,fxy/fxz/fyz,fxyz\"",
    cxxopts::value<std::vector<double>>()->default_value("1,1,1,1"))
    ("d,debug_dir",
    "Directory for debug output for correspondences.",
    cxxopts::value<std::string>()->default_value(""))
    ("s,suppress_logging",
    "Suppress log output",
    cxxopts::value<bool>()->default_value("false"))
    ("h,help",
    "Print usage");
  // clang-format on

  auto result = options.parse(argc, argv);

  if (result.count("help"))
  {
    std::cout << options.help() << std::endl;
    exit(0);
  }

  // Save to params
  Params params{};
  params.fixed = result["fixed"].as<std::string>();
  params.movable = result["movable"].as<std::string>();
  params.transform = result["transform"].as<std::string>();
  params.voxel_size = result["voxel_size"].as<double>();
  params.grid_limits = result["grid_limits"].as<std::vector<double>>();
  params.buffer_voxels = result["buffer_voxels"].as<uint32_t>();
  params.matching_mode = result["matching_mode"].as<std::string>();
  params.num_correspondences = result["num_correspondences"].as<uint32_t>();
  params.max_euclidean_distance = result["max_euclidean_distance"].as<double>();
  params.num_iterations = result["num_iterations"].as<uint32_t>();
  params.weights = result["weights"].as<std::vector<double>>();
  params.debug_dir = result["debug_dir"].as<std::string>();
  params.suppress_logging = result["suppress_logging"].as<bool>();

  // Check parameter inputs
  if (params.matching_mode == "id")
  {
    params.num_iterations = 1;
    spdlog::info("Set num_iterations to {:d} as matching mode \"{}\" was selected.",
                 params.num_iterations,
                 params.matching_mode.c_str());
  }

  if (params.matching_mode != "nn" && params.matching_mode != "id")
  {
    std::string error_string = "Matching mode \"" + params.matching_mode + "\" is not available!";
    throw std::runtime_error(error_string);
  }

  if (params.debug_dir != "")
  {
    // Add trailing slash if not present
    if (params.debug_dir.back() != '/')
    {
      params.debug_dir += '/';
    }

    // Check if path exists
    if (!std::filesystem::exists(params.debug_dir))
    {
      std::string error_string = "Debug directory \"" + params.debug_dir + "\" does not exist!";
      throw std::runtime_error(error_string);
    }
  }

  return params;
}

void ReportIterationResults(const IterationResults& iteration_results)
{
  if (iteration_results.it == 1)
  {
    spdlog::info("{:>4} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10} {:>10}",
                 "it",
                 "num_corr",
                 "num_obs",
                 "num_unkn",
                 "mean(dp)",
                 "mean(dp)",
                 "std(dp)",
                 "std(dp)");
    spdlog::info("{:37} {:>10} {:>10} {:>10} {:>10}", "", "before", "after", "before", "after");
  }
  spdlog::info(
      "{:4d} {:10d} {:10d} {:10d} {:10.3f} {:10.3f} {:10.3f} {:10.3f}",
      iteration_results.it,
      iteration_results.correspondences_results.num,
      iteration_results.optimization_results.num_observations,
      iteration_results.optimization_results.num_unknowns,
      iteration_results.correspondences_results.mean_point_to_plane_dists_before_optimization,
      iteration_results.correspondences_results.mean_point_to_plane_dists_after_optimization,
      iteration_results.correspondences_results.std_point_to_plane_dists_before_optimization,
      iteration_results.correspondences_results.std_point_to_plane_dists_after_optimization);
}