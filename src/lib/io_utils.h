#ifndef IO_UTILS_H
#define IO_UTILS_H

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <pdal/Options.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/BufferReader.hpp>

Eigen::MatrixXd ImportFileToMatrix(const std::string& path);

// Matrix with x,y,z or x,y,z,nx,ny,nz
Eigen::MatrixXd ExtractMatrix(const pdal::PointViewPtr view, const bool& with_normals);

// Return string with fields in pointcloud
std::string PointcloudFieldsToString(const pdal::PointViewPtr view);

void SaveMatrixToFile(const Eigen::MatrixXd& A,
                      const std::string& path_in,
                      const std::string& path_out);

std::string CreatePDALReaderType(const std::string& extension);

std::string CreatePDALWriterType(const std::string& extension);

pdal::Options CreatePDALWriterOptions(const std::string& extension);

void UpdateTransformedPointcloud(const pdal::PointViewPtr view, const Eigen::MatrixXd& x_updated);

void CheckNumColsOfMatrix(const Eigen::MatrixXd& M,
                          const int& num_cols,
                          const std::string& matrix_name);

#endif  // IO_UTILS_H
