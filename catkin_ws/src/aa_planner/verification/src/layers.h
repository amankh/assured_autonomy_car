#include <ctime>
#include <vector>
#include <list>
#include <numeric>
#include <algorithm>
#include <ostream>
#include <fstream>
#include "dreal/dreal.h"
#include "units.h"
#include "vectorize.h"
#include "parsing.h"

namespace dreal {
  using std::vector;
  using std::list;

  Formula is_close(vector<Expression> es1, vector<Expression> es2, Expression epsilon);
        
  optional<Box> CheckSat(const Formula& f, const Config& config);
        
  vector<Expression> dense(vector<Expression> input,
			   vector<vector<Expression>> W,
			   vector<Expression> b);
  
  vector<Expression> linear(vector<Expression> input,
			    vector<vector<Expression>> W,
			    vector<Expression> b);
  
  vector<Expression> get_policy(vector<Expression> input, std::string fn);
  
}
