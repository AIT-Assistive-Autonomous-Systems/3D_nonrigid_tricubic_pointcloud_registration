#include <fstream>

#include "ioutils.h"

Eigen::MatrixXd ImportFileToMatrix(const std::string &path) {
  std::ifstream data(path);
  if (data.is_open()) {
    // Read data from file
    std::vector<std::vector<std::string>> parsedData;
    std::string line;
    while (getline(data, line)) {
      std::stringstream lineStream(line);
      std::string cell;  // single value
      std::vector<std::string> parsedRow;
      while (getline(lineStream, cell, ' ')) {
        parsedRow.push_back(cell);
      }
      parsedData.push_back(parsedRow);
    }

    if (parsedData.size() == 0) {
      std::cerr << "File \"" << path << "\" seems to be empty!" << std::endl;
      exit(1);
    }

    // Check if each line contains the same number of elements
    uint64_t num_cols_first_line;
    uint64_t num_cols_current_line;
    for (int i = 0; i < parsedData.size(); i++) {
      if (i == 0) {
        num_cols_first_line = parsedData[i].size();
      } else {
        num_cols_current_line = parsedData[i].size();
        if (num_cols_current_line != num_cols_first_line) {
          std::cerr << "File \"" << path << "\": first line contains " << num_cols_first_line
                    << " elements. However, line " << i + 1 << " contains " << parsedData[i].size()
                    << " values!" << std::endl;
          exit(1);
        }
      }
    }

    // Create eigen array
    Eigen::MatrixXd X(parsedData.size(), num_cols_first_line);
    for (int i = 0; i < parsedData.size(); i++) {
      for (int j = 0; j < parsedData[i].size(); j++) {
        try {
          X(i, j) = stod(parsedData[i][j]);
        } catch (std::exception &e) {
          std::cerr << "File \"" << path << "\": conversion of " << parsedData[i][j]
                    << " on row/column=" << i << "/" << j << " is not possible!" << std::endl;
          exit(1);
        }
      }
    }

    return X;

  } else {
    std::cerr << "Error opening file \"" << path << "\"!" << std::endl;
    exit(1);
  }
}

void SaveMatrixToFile(const Eigen::MatrixXd &A, const std::string &path) {
  // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");
  std::ofstream file(path);
  if (file.is_open()) {
    file << A.format(CSVFormat);
    file.close();
  }
}

// TODO Change to "... must have exactly 6 columns: x y z nx ny nz."
void CheckNumColsOfMatrix(const Eigen::MatrixXd &M,
                          const int &num_cols,
                          const std::string &matrix_name) {
  if (M.cols() != num_cols) {
    std::cout << "Point cloud \"" << matrix_name << "\" does not have exactly " << num_cols
              << " columns!" << std::endl;
    exit(1);
  }
}