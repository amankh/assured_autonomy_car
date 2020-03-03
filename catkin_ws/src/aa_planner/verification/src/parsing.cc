#include <vector>
#include <numeric>
#include <algorithm>
#include "dreal/dreal.h"
#include "nlohmann/json.hpp"

//TODO: All of this with appropriate classes.

using json = nlohmann::json;

namespace dreal {
  using std::vector;
  
  vector<Expression> vec_expr_of_vec(vector<double> v) {
    vector<Expression> out(v.size());
    std::transform(v.begin(), v.end(), out.begin(),
		   [](double d) { Expression e{d}; return e; });
    return out;
  }

  vector<vector<Expression>> mat2d_expr_of_mat2d(vector<vector<double>> m) {
    vector<vector<Expression>> out(m.size());
    std::transform(m.begin(), m.end(), out.begin(), vec_expr_of_vec);
    return out;
  }

  vector<vector<vector<Expression>>> tensor3d_expr_of_tensor3d(vector<vector<vector<double>>> m) {
    vector<vector<vector<Expression>>> out(m.size());
    std::transform(m.begin(), m.end(), out.begin(), mat2d_expr_of_mat2d);
    return out;
  }

  /* Assumes already in "Connected layers" */
  vector<Expression> get_biases(json j) {
        return vec_expr_of_vec(j["biases"]);
  }

  /* Assumes already in "Connected layers" */
  vector<vector<Expression>> get_weights(json j) {
    return mat2d_expr_of_mat2d(j["weights"]);
  }

  
  vector<vector<Expression>> get_biases3d(json j) {
        return mat2d_expr_of_mat2d(j["biases"]);
  }

  vector<vector<vector<Expression>>> get_weights3d(json j) {
    return tensor3d_expr_of_tensor3d(j["weights"]);
  }
  
  /* Assumes already in "Connected layers" -> "Convolutional kernels" */
  vector<vector<vector<Expression>>> get_kernels(json j) { 
    return tensor3d_expr_of_tensor3d(j["kernels"]);
  }
} // namespace dreal
