#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include "src/lib/io_utils.h"
#include "src/lib/pt_cloud.h"
#include "src/prog/3rdparty/cxxopts/cxxopts.hpp"

struct Params
{
  std::string pc_in;
  std::string pc_out;
  std::string transform;
  bool suppress_logging;
};

Params ParseUserInputs(int argc, char** argv);

int main(int argc, char** argv)
{
  try
  {
    spdlog::set_pattern("%v");

    Params params = ParseUserInputs(argc, argv);

    spdlog::stopwatch sw;
    spdlog::info("Start of \"gbpcm-transform\"");

    spdlog::info("Create point cloud object");
    auto X = ImportFileToMatrix(params.pc_in, false, false);
    auto pc_mov{PtCloud(X.leftCols(3))};
    spdlog::info("Point cloud has {:d} points", pc_mov.NumPts());

    spdlog::info("Import x/y/z translation grids");
    pc_mov.ImportTranslationGrids(params.transform);

    spdlog::info("Transform point cloud");
    pc_mov.InitMatricesForUpdateXt();
    pc_mov.UpdateXt();
    X.leftCols(3) = pc_mov.Xt();

    spdlog::info("Write transformed point cloud to file");
    SaveMatrixToFile(X, params.pc_in, params.pc_out);

    spdlog::info("Finished \"gbpcm-transform\" in {:.3}s!", sw);
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
  cxxopts::Options options(
      "gbpcm-transform",
      "Transformation of point cloud using transform file generated by the executable \"gbpcm\"");

  // clang-format off
  options.add_options()
  ("i,pc_in",
   "Path to input point cloud",
   cxxopts::value<std::string>())
  ("o,pc_out",
   "Path to output point cloud, i.e. transformed point cloud",
   cxxopts::value<std::string>())
  ("t,transform",
   "Path to transform file generated by the executable \"gbpcm\"",
   cxxopts::value<std::string>())
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
  params.pc_in = result["pc_in"].as<std::string>();
  params.pc_out = result["pc_out"].as<std::string>();
  params.transform = result["transform"].as<std::string>();
  params.suppress_logging = result["suppress_logging"].as<bool>();

  // Check parameter inputs
  if (result["suppress_logging"].as<bool>())
  {
    spdlog::set_level(spdlog::level::off);
  }

  return params;
}
