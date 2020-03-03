#include <vector>
#include <set>
#include <numeric>
#include <algorithm>
#include "dreal/dreal.h"
#include "units.h"
#include "dreal/symbolic/symbolic_expression.h"

namespace dreal {
  using std::vector;
  using std::set;

  vector<Expression> expr_vec_scale(vector<Expression> v, Expression e) {
    vector<Expression> out(v.size());
    std::transform(v.begin(), v.end(), out.begin(), [=] (Expression f) { return (e * f); });
    return out;
  }
  
  vector<Expression> expr_vec_add(vector<Expression> es1, vector<Expression> es2) {
    vector<Expression> out(es1.size());
    std::transform(es1.begin(), es1.end(), es2.begin(), out.begin(),
		   std::plus<Expression>());
    return out;
  }

  vector<Expression> expr_vec_otimes(vector<Expression> es1, vector<Expression> es2) {
    vector<Expression> out(es1.size());
    std::transform(es1.begin(), es1.end(), es2.begin(), out.begin(),
		   std::multiplies<Expression>());
    return out;
  }

  Expression expr_vec_sum(vector<Expression> es) {
    Expression z = 0;
    return std::accumulate(es.begin(), es.end(), z, std::plus<Expression>());
  }

  Expression expr_inner_prod(vector<Expression> cs, vector<Expression> es) {
    return expr_vec_sum(expr_vec_otimes(cs, es));
  }

  Expression expr_norm(vector<Expression> v) {
    return (sqrt(expr_inner_prod(v,v)));
  }

  Expression expr_dist(vector<Expression> v, vector<Expression> w) {
    return (expr_norm(expr_vec_add(v,expr_vec_scale(w,-1.0))));
  }

  vector<vector<Expression>> trans_mat(vector<vector<Expression>> m) {
    vector<vector<Expression>> out(m[0].size(), vector<Expression>(m.size()));
    for (uint i = 0; i < out.size(); ++i) {
      for (uint j = 0; j < m.size(); ++j) {
	out[i][j] = m[j][i];
      }
    }
    return out;
  }

  /* helper function to clean up mat mul code */
  vector<Expression> mat_mul_help(vector<vector<Expression>> m, vector<Expression> v) {
    
    vector<Expression> out(m.size());
    
    std::transform(m.begin(), m.end(), out.begin(),
		   [&] (std::vector<Expression> w) { return expr_inner_prod(v,w); });
    
    return out;
    
  }
  
  /* TODO add size check asserts */
  vector<vector<Expression>> expr_mat_mul(vector<vector<Expression>> m1,
					  vector<vector<Expression>> m2) {
    vector<vector<Expression>> m3 = trans_mat(m2);
    vector<vector<Expression>> out(m1.size(), vector<Expression>(m2[0].size()));
    
    std::transform(m1.begin(), m1.end(), out.begin(),
		   [&] (vector<Expression> es1) { return mat_mul_help(m3,es1); });
    return out;
  }

  vector<Expression> expr_mat_vec_mul(vector<vector<Expression>> m,
				      vector<Expression> v) {
    vector<Expression> out(m.size());
    Expression e = 0;
    for (uint row = 0; row < m.size(); ++row) {
      out[row] = expr_inner_prod(m[row], v);
    }
    return out;
  }
  
  vector<vector<Expression>> expr_zero_pad(vector<vector<Expression>> m,
					   int row_pad,
					   int col_pad) {
    vector<vector<Expression>> out(m.size() + 2 * row_pad,
				   vector<Expression>(m[0].size() + 2 * col_pad));
    for (uint i = 0; i < m.size(); i++) {
      for (uint j = 0; j < m[0].size(); j++) {
	out[i+1][j+1] = m[i][j];
      }
    }    
    return out;
  }

  vector<vector<Expression>> expr_zero_pad(vector<vector<Expression>> m,
					   int row_pad_before, int row_pad_after,
					   int col_pad_before, int col_pad_after) {
    vector<vector<Expression>> out(m.size() + row_pad_before + row_pad_after,
				   vector<Expression>(m[0].size() + col_pad_before + col_pad_after));
    for (uint i = 0; i < m.size(); i++) {
      for (uint j = 0; j < m[0].size(); j++) {
	out[i+row_pad_before][j+col_pad_before] = m[i][j];
      }
    }    
    return out;
  }

  vector<vector<Expression>> expr_simple_zero_pad(vector<vector<Expression>> m,
					   int row_pad,
					   int col_pad) {
    return (expr_zero_pad(m, row_pad, row_pad, col_pad, col_pad));
  }
 
  set<Expression> get_bases(const Expression& e) {
    assert(is_addition(e));
    set<Expression> s;
    std::map<Expression, double> m = get_expr_to_coeff_map_in_addition(e);
    for (std::map<Expression, double>::iterator it = m.begin(); it != m.end(); ++it) {
      s.insert(it->first);
    }
    return s;
  }

  vector<Expression> expr_vec_sigmoid(vector<Expression> v) {
    vector<Expression> out(v.size());
    std::transform(v.begin(), v.end(), out.begin(), sigmoid);
    return out;
  }

  vector<Expression> expr_vec_relu(vector<Expression> v) {
    vector<Expression> out(v.size());
    std::transform(v.begin(), v.end(), out.begin(), relu);
    return out;
  }

  vector<Expression> expr_vec_tanh(vector<Expression> v) {
    vector<Expression> out(v.size());
    std::transform(v.begin(), v.end(), out.begin(), [](const Expression& e){ return tanh(e); });
    return out;
  }

  
  Formula vec_conjunction(vector<Formula> v) {
    return std::accumulate(v.begin(), v.end(), v[0],
			   [](Formula f1, Formula f2) { return (f1 && f2); }); 
  }

  Formula vec_disjunction(vector<Formula> v) {
    return std::accumulate(v.begin(), v.end(), v[0],
			   [](Formula f1, Formula f2) { return (f1 || f2); }); 
  }
}// dreal
