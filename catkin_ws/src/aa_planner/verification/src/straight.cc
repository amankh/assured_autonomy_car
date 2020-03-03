/* Analysis of RL agent
 * @author Michael A. Warren <mawarren@hrl.com>
 */
#include <ctime>
#include <vector>
#include "nlohmann/json.hpp"
#include "layers.h"

using json = nlohmann::json;


namespace dreal {
  namespace {
    using std::cout;
    using std::endl;
    //using std::experimental::optional;
    using std::vector;

    
    void sat_checker_straight(const double delta,
			      // const double yaw_bound,
			      // const double yaw_dot_bound,
			      std::string weights_fn,
			      const int num_workers) {
      Variable y("y");
      Variable yaw("yaw");
      Variable x_dot("x_dot");
      Variable y_dot("y_dot");
      Variable yaw_dot("yaw_dot");

      vector<Expression> inputs({y, yaw, x_dot, y_dot, yaw_dot});

      vector<Expression> policy = get_policy(inputs, weights_fn);
      
      // Basic distance version
      // Formula constraints =
      // 	-yaw_bound < yaw && yaw < yaw_bound
      // 				  && -yaw_dot_bound < yaw_dot
      // 						      && yaw_dot < yaw_dot_bound;
				  
      // Formula form_r = y > 0.2 && policy[1] > 0.2;// agent selects right turn
      // Formula form_l = y < -0.2 && policy[1] < -0.2;
      // Formula form = //constraints &&
      // (form_r || form_l);

      Expression sign_y = if_then_else(y > 0.01, 1., -1.);
      Expression sign_yaw = if_then_else(yaw > 0.01, 1., -1.);
      Expression driving_away = policy[1] * (sign_y + sign_yaw);
      Formula form = driving_away > 0.1;
      
      cout << "Delta is: " << delta << endl;

      Config config;
      config.mutable_precision() = delta;
      config.mutable_number_of_jobs() = num_workers;
      optional<Box> result = CheckSat(form, config);


      cout << "Agent selects extreme steering angle: ";
      if (result) {
	Box box = *result;
      	cout << "delta-SAT:\n" << box << endl;

	Expression y_val = box[y].mid();
	Expression yaw_val = box[yaw].mid();
	Expression x_dot_val = box[x_dot].mid();
	Expression y_dot_val = box[y_dot].mid();
	Expression yaw_dot_val = box[yaw_dot].mid();
	vector<Expression> eval_point({y_val, yaw_val, x_dot_val, y_dot_val, yaw_dot_val});
	vector<Expression> policy_out = get_policy(eval_point, weights_fn);
	cout << "  policy = [";
	for (uint i = 0; i < policy_out.size(); ++i) {
	  cout << " " << policy_out[i] << " ";
	}
	cout << "]" << endl;
	
      } else {
      	cout << "UNSAT" << endl;
      }
    }
    
  } // namespace
  
} // namespace dreal


int main(int argc, const char* argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " weights.json delta num_workers" << std::endl;
    exit(1);
  }
  std::string weights_fn = argv[1];
  double delta = std::atof(argv[2]);
  int num_workers = std::atoi(argv[3]);
  // double yaw_bound = std::atof(argv[3]);
  // double yaw_dot_bound = std::atof(argv[4]);
  dreal::sat_checker_straight(delta,
			      // yaw_bound, yaw_dot_bound,
			      weights_fn,
			      num_workers);
}
