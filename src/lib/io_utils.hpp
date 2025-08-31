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
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>

#include "src/lib/named_column_matrix.hpp"

NamedColumnMatrix<Eigen::MatrixXd> ImportFileToMatrix(const std::string& path,
                                                      const bool& with_normals,
                                                      const bool& with_correspondence_id);

// Matrix with x,y,z or x,y,z,nx,ny,nz
NamedColumnMatrix<Eigen::MatrixXd> ExtractMatrix(const pdal::PointViewPtr view,
                                                 const bool& with_normals,
                                                 const bool& with_correspondence_id);

// Return string with fields in pointcloud
std::string PointcloudFieldsToString(const pdal::PointViewPtr view);

// Save pointcloud to file
void SaveMatrixToFile(const NamedColumnMatrix<Eigen::MatrixXd>& A, const std::string& path_in,
                      const std::string& path_out);

// Create PDAL reader signature from a file extension
std::string CreatePDALReaderType(const std::string& extension);

// Create PDAL writer signature from a file extension
std::string CreatePDALWriterType(const std::string& extension);

// Create PDAL writer options from a file extension
pdal::Options CreatePDALWriterOptions(const std::string& extension);

// Overwrite the transformed x,y,z coordinates to the original pointcloud
void UpdateTransformedPointcloud(const pdal::PointViewPtr view,
                                 const NamedColumnMatrix<Eigen::MatrixXd>& x_updated);

#endif  // IO_UTILS_H
