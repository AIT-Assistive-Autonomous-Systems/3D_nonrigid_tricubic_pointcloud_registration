#pragma once

#include <Eigen/Dense>
#include <map>
#include <stdexcept>

template <typename T>
class NamedColumnMatrix : public T {
 public:
  // Inherit Eigen::Matrix constructors
  using T::T;

  // Constructor for NamedColumnMatrix
  NamedColumnMatrix(NamedColumnMatrix const& other) : T(other), col_names_(other.col_names_) {}
  NamedColumnMatrix(const T& matrix, const std::vector<std::string>& col_names) : T(matrix) {
    CheckNamedColsSize(col_names);
    AddColNames(col_names);
  }

  // Overload = operator
  NamedColumnMatrix<T>& operator=(const NamedColumnMatrix<T>& other) {
    if (this != &other) {
      this->T::operator=(other);
      col_names_ = other.col_names_;
    }
    return *this;
  }

  // Get column by name
  inline typename T::ColXpr namedCol(const std::string& col_name) {
    return this->T::col(namedColIndex(col_name));
  }
  const inline typename T::ColXpr namedCol(const std::string& col_name) const {
    return this->T::col(namedColIndex(col_name));
  }

  // Access matrix element by row index and column name
  inline typename T::Scalar& namedColElem(int row_idx, const std::string& col_name) {
    return this->coeffRef(row_idx, namedColIndex(col_name));
  }
  const inline typename T::Scalar& namedColElem(int row_idx, const std::string& col_name) const {
    return this->coeffRef(row_idx, namedColIndex(col_name));
  }

  // Return the column index by name
  inline Eigen::Index namedColIndex(const std::string& col_name) const {
    return col_names_.at(col_name);
  }

 private:
  // Check if named columns match matrix columns size
  inline void CheckNamedColsSize(const std::vector<std::string>& col_names) {
    if (col_names.size() != static_cast<size_t>(this->cols())) {
      throw std::invalid_argument("Column name size does not match matrix column size");
    }
  }

  // Add column names to map
  inline void AddColNames(const std::vector<std::string>& col_names) {
    // Add column names to map
    for (size_t col_idx = 0; col_idx < col_names.size(); col_idx++) {
      col_names_[col_names[col_idx]] = col_idx;
    }
  }

  std::map<std::string, Eigen::Index> col_names_;
};