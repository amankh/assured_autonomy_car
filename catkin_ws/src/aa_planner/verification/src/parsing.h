/* Parse json representation of network into dReal4 solver format
 * @author Michael A. Warren <mawarren@hrl.com>
 */
#include <vector>
#include "dreal/dreal.h"
#include "nlohmann/json.hpp"

// TODO: With appropriate classes.

using json = nlohmann::json;

namespace dreal {

  std::vector<Expression> vec_expr_of_vec(vector<double> v);

  std::vector<std::vector<Expression>> mat2d_expr_of_mat2d(vector<vector<double>> m);
  
  /* Assumes already in "Connected layers" */
  std::vector<Expression> get_biases(nlohmann::json j);

  /* Assumes already in "Connected layers" */
  std::vector<std::vector<Expression>> get_weights(nlohmann::json j);

  /* Assumes already in "Connected layers" -> "Convolutional kernels" */
  std::vector<std::vector<std::vector<Expression>>> get_kernels(nlohmann::json j);

  vector<vector<Expression>> get_biases3d(nlohmann::json j);

  vector<vector<vector<Expression>>> get_weights3d(nlohmann::json j);

}
