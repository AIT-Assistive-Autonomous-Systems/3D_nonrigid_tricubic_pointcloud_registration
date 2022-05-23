#ifndef IOUTILS_H
#define IOUTILS_H

#include <Eigen/Dense>
#include <iostream>

Eigen::MatrixXd ImportFileToMatrix(const std::string& path);

void SaveMatrixToFile(const Eigen::MatrixXd& A, const std::string& path);

void CheckNumColsOfMatrix(const Eigen::MatrixXd& M,
                          const int& num_cols,
                          const std::string& matrix_name);

#endif  // IOUTILS_H
