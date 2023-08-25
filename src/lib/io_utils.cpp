#include "io_utils.h"

Eigen::MatrixXd ImportFileToMatrix(const std::string& path)
{
  // Extract extension
  std::string extension = std::filesystem::path(path).extension().string();

  // Check if pdal supports the file format
  pdal::StageFactory factory;

  // Create pdal reader type based on suffix
  std::string pdal_reader_type;
  if (extension == ".las" || extension == ".laz")
  {
    pdal_reader_type = "readers.las";
  }
  else if (extension == ".ply")
  {
    pdal_reader_type = "readers.ply";
  }
  else if (extension == ".xyz" || extension == ".txt")
  {
    pdal_reader_type = "readers.text";
  }
  else
  {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }

  // Check if a reader type is available
  pdal::Stage* reader = factory.createStage(pdal_reader_type);

  // Initialize PDAL reader
  pdal::PointTable table;
  pdal::Options options;
  options.add("filename", path);
  reader->setOptions(options);
  reader->prepare(table);

  // Execute the reader to read the point cloud
  pdal::PointViewSet view_set = reader->execute(table);

  // Take first view from view set (contains the point cloud data)
  pdal::PointViewPtr view = *view_set.begin();

  if (view->empty())
  {
    std::string error_string = "Point cloud is empty!";
    throw std::runtime_error(error_string);
  }
  else
  {
    std::cout << "Read in pointcloud with " << view->size() << " points." << std::endl;
  }

  // Return 6D eigen matrix
  return ExtractMatrix(view, true);
}

Eigen::MatrixXd ExtractMatrix(const pdal::PointViewPtr view, const bool& with_normals)
{
  if (!with_normals)
  {
    if (!view->hasDim(pdal::Dimension::Id::X) || !view->hasDim(pdal::Dimension::Id::Y) ||
        !view->hasDim(pdal::Dimension::Id::Z))
    {
      std::string error_string;
      error_string += "Point cloud does not have all fields for 3D matrix!\n";
      error_string += "Fields required: X, Y, Z\n";
      error_string += PointcloudFieldsToString(view);
      throw std::runtime_error(error_string);
    }
  }
  else
  {
    if (!view->hasDim(pdal::Dimension::Id::X) || !view->hasDim(pdal::Dimension::Id::Y) ||
        !view->hasDim(pdal::Dimension::Id::Z) || !view->hasDim(pdal::Dimension::Id::NormalX) ||
        !view->hasDim(pdal::Dimension::Id::NormalY) || !view->hasDim(pdal::Dimension::Id::NormalZ))
    {
      std::string error_string;
      error_string += "Point cloud does not have all fields for 6D matrix!\n";
      error_string += "Fields required: X, Y, Z, NormalX, NormalY, NormalZ\n";
      error_string += PointcloudFieldsToString(view);
      throw std::runtime_error(error_string);
    }
  }

  int matrix_cols = with_normals ? 6 : 3;
  Eigen::MatrixXd x(view->size(), matrix_cols);
  for (pdal::PointId idx = 0; idx < view->size(); idx++)
  {
    x(idx, 0) = view->getFieldAs<double>(pdal::Dimension::Id::X, idx);
    x(idx, 1) = view->getFieldAs<double>(pdal::Dimension::Id::Y, idx);
    x(idx, 2) = view->getFieldAs<double>(pdal::Dimension::Id::Z, idx);
    if (with_normals)
    {
      x(idx, 3) = view->getFieldAs<double>(pdal::Dimension::Id::NormalX, idx);
      x(idx, 4) = view->getFieldAs<double>(pdal::Dimension::Id::NormalY, idx);
      x(idx, 5) = view->getFieldAs<double>(pdal::Dimension::Id::NormalZ, idx);
    }
  }

  return x;
}

void UpdateTransformedPointcloud(const pdal::PointViewPtr view, const Eigen::MatrixXd& x_updated)
{
  if (!view->hasDim(pdal::Dimension::Id::X) || !view->hasDim(pdal::Dimension::Id::Y) ||
      !view->hasDim(pdal::Dimension::Id::Z))
  {
    std::string error_string;
    error_string += "Point cloud does not have all required fields!\n";
    error_string += "Fields required: X, Y, Z\n";
    error_string += PointcloudFieldsToString(view);
    throw std::runtime_error(error_string);
  }

  for (pdal::PointId idx = 0; idx < view->size(); idx++)
  {
    view->setField(pdal::Dimension::Id::X, idx, x_updated(idx, 0));
    view->setField(pdal::Dimension::Id::Y, idx, x_updated(idx, 1));
    view->setField(pdal::Dimension::Id::Z, idx, x_updated(idx, 2));
  }
}

std::string PointcloudFieldsToString(const pdal::PointViewPtr view)
{
  std::stringstream string_stream;
  string_stream << "Fields in pointcloud: ";
  for (auto const& dim : view->dims())
  {
    string_stream << view->dimName(dim);
    if (dim != view->dims().back())
    {
      string_stream << ", ";
    }
  }
  string_stream << std::endl;
  return string_stream.str();
}

void SaveMatrixToFile(const Eigen::MatrixXd& x_updated,
                      const std::string& path_in,
                      const std::string& path_out)
{
  // Extract extension
  std::string extension_in = std::filesystem::path(path_in).extension().string();
  std::string extension_out = std::filesystem::path(path_out).extension().string();

  // Create pdal reader/writer type based on suffix
  std::string pdal_reader_type = CreatePDALReaderType(extension_in);
  std::string pdal_writer_type = CreatePDALWriterType(extension_out);

  // Check if input and output file formats are the same
  if (extension_in != extension_out)
  {
    std::string error_string;
    error_string += "Input and output file formats are not the same!\n";
    error_string += "Input file format: " + extension_in + "\n";
    error_string += "Output file format: " + extension_out + "\n";
    throw std::runtime_error(error_string);
  }

  // Create stage factory
  pdal::StageFactory factory;

  // Check if a reader type is available
  pdal::Stage* reader = factory.createStage(pdal_reader_type);

  // Initialize PDAL reader
  pdal::PointTable table;
  pdal::Options options;
  options.add("filename", path_in);
  reader->setOptions(options);
  reader->prepare(table);

  // Execute the reader to read the point cloud
  pdal::PointViewSet view_set = reader->execute(table);

  // Take first view from view set (contains the point cloud data)
  pdal::PointViewPtr view = *view_set.begin();
  if (view->empty())
  {
    std::string error_string = "Point cloud is empty!";
    throw std::runtime_error(error_string);
  }
  else
  {
    std::cout << "Read in pointcloud with " << view->size() << " points." << std::endl;
  }

  // Update x,y,z fields of pointcloud with transformed values
  UpdateTransformedPointcloud(view, x_updated);

  // PDAL Writer
  std::cout << "Write pointcloud to file: " << path_out << std::endl;
  pdal::Stage* writer = factory.createStage(pdal_writer_type);
  pdal::Options writer_options = CreatePDALWriterOptions(extension_out);
  writer_options.add("filename", path_out);
  writer->setOptions(writer_options);

  // Link the reader to the writer with a buffer reader
  pdal::BufferReader buffer_reader;
  buffer_reader.addView(view);
  writer->setInput(buffer_reader);

  // Prepare the writer
  writer->prepare(table);

  // Execute the writer to write the point cloud
  writer->execute(table);
}

std::string CreatePDALReaderType(const std::string& extension)
{
  std::string pdal_reader_type;
  if (extension == ".las" || extension == ".laz")
  {
    pdal_reader_type = "readers.las";
  }
  else if (extension == ".ply")
  {
    pdal_reader_type = "readers.ply";
  }
  else if (extension == ".xyz" || extension == ".txt")
  {
    pdal_reader_type = "readers.text";
  }
  else
  {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_reader_type;
}

std::string CreatePDALWriterType(const std::string& extension)
{
  std::string pdal_writer_type;
  if (extension == ".las" || extension == ".laz")
  {
    pdal_writer_type = "writers.las";
  }
  else if (extension == ".ply")
  {
    pdal_writer_type = "writers.ply";
  }
  else if (extension == ".xyz" || extension == ".txt")
  {
    pdal_writer_type = "writers.text";
  }
  else
  {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_writer_type;
}

pdal::Options CreatePDALWriterOptions(const std::string& extension)
{
  pdal::Options pdal_writer_options;
  if (extension == ".las" || extension == ".laz")
  {
    pdal_writer_options.add("forward", "all");
    pdal_writer_options.add("extra_dims", "all");
  }
  else if (extension == ".ply")
  {
  }
  else if (extension == ".xyz" || extension == ".txt")
  {
  }
  else
  {
    std::string error_string;
    error_string += "File format '" + extension + "' is not supported.\n";
    throw std::runtime_error(error_string);
  }
  return pdal_writer_options;
}