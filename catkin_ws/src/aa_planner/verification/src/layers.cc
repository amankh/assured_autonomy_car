/* Lightweight PCP Net analysis
 * @author Michael A. Warren <mawarren@hrl.com>
 */
#include <regex>
#include "layers.h"

using json = nlohmann::json;

namespace dreal {
  using std::cout;
  using std::endl;
  using std::vector;

  Formula is_close(vector<Expression> es1, vector<Expression> es2, Expression epsilon) {
    return (expr_dist(es1,es2) < epsilon);
  }
    
  optional<Box> CheckSat(const Formula& f, const Config& config) {
    Context context(config);
    for (const Variable& v : f.GetFreeVariables()) {
      // Straight variables
      if (v.get_name() == "y") {
	// context.DeclareVariable(v, -0.25, 0.25);
	context.DeclareVariable(v, -1.0, 1.0);
      } else if (v.get_name() == "yaw") {
	context.DeclareVariable(v, -1.0471975511965976, 1.0471975511965976);
      } else if (v.get_name() == "x_dot") {
	// context.DeclareVariable(v, 0, 1.3);
	context.DeclareVariable(v, 0, 8.0);
      } else if (v.get_name() == "y_dot") {
	context.DeclareVariable(v, -0.6, 0.6);
      } else if (v.get_name() == "yaw_dot") {
	// context.DeclareVariable(v, -2.0, 2.0);
	context.DeclareVariable(v, -0.3, 0.3);
      } else if (v.get_name() == "dx") {
      // Circle variables
	context.DeclareVariable(v, -0.25, 0.25);
      } else if (v.get_name() == "theta") {
	context.DeclareVariable(v, -0.5235, 0.5235);
      } else if (v.get_name() == "dx_dot") {
	context.DeclareVariable(v, -0.525, 0.525);
      } else if (v.get_name() == "theta_dot") {
	context.DeclareVariable(v, -0.7, 0.7);
      } else {
	context.DeclareVariable(v, -1.0, 1.0);
      }
    }
    
    context.Assert(f);
    
    return context.CheckSat();
  }
     
  vector<Expression> dense(vector<Expression> input,
			   vector<vector<Expression>> W,
			   vector<Expression> b) {
    vector<Expression> out = expr_mat_vec_mul(trans_mat(W), input);
    out = expr_vec_add(out, b);
    out = expr_vec_relu(out);
    return out;
  }
  
  vector<Expression> linear(vector<Expression> input,
			    vector<vector<Expression>> W,
			    vector<Expression> b) {
    vector<Expression> out = expr_mat_vec_mul(trans_mat(W), input);
    out = expr_vec_add(out, b);
    return out;
  }
  
  int get_depth(json j) {
    int count = 0;
    std::regex re("kernel[0-9]*");
    std::smatch m;
    for (json::iterator it = j.begin(); it != j.end(); ++it) {
      std::string s = it.key();
      if (std::regex_match(s, m, re)) {
	++count;
      }
    }
    return count;
  }
  
  vector<Expression> get_policy(vector<Expression> input, std::string fn) {
    std::ifstream ic(fn);
    auto j = json::parse(ic);
    ic.close();
    
    vector<vector<vector<Expression>>> weights;
    vector<vector<Expression>> biases;
    
    int depth = get_depth(j);
    for (int k = 0; k < depth; ++k) {
      std::string w = "kernel" + std::to_string(k);
      std::string b = "bias" + std::to_string(k);
      weights.push_back(mat2d_expr_of_mat2d(j[w]));
      biases.push_back(vec_expr_of_vec(j[b]));
    }
    std::cout << "depth is " << depth << std::endl;
    vector<Expression> out = linear(input, weights[0], biases[0]);
    out = expr_vec_tanh(out);
    out = linear(out, weights[1], biases[1]);
    out = expr_vec_tanh(out);
    out = linear(out, weights[2], biases[2]);
    return out;
  }
    
} // namespace dreal
