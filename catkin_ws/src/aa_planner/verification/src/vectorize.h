/* Vectorizing input to the dReal4 solver
 * @author Michael A. Warren <mawarren@hrl.com>
 */
#include <vector>
#include "dreal/dreal.h"


// TODO: Instead of std::vector<Expression>, etc., make this a class.


namespace dreal {
  using std::vector;

  vector<Expression> flatten(vector<vector<Expression>> m);

  vector<Expression> expr_vec_scale(vector<Expression> v, Expression e);
  
  /* vector addition */
  vector<Expression> expr_vec_add(vector<Expression> es1, vector<Expression> es2);
  
  /* pointwise multiplication of vectors */
  vector<Expression> expr_vec_otimes(vector<Expression> es1, vector<Expression> es2);  

  /* sum of all entries in a vector */  
  Expression expr_vec_sum(vector<Expression> es);

  /* inner product */
  Expression expr_inner_prod(vector<Expression> cs, vector<Expression> es);

  /* norm */
  Expression expr_norm(vector<Expression> v);

  /* distance */
  Expression expr_dist(vector<Expression> v, vector<Expression> w);

  /* matrix transpose */
  vector<vector<Expression>> trans_mat(vector<vector<Expression>> m);
  
  /* matrix multiplication */
  vector<vector<Expression>> expr_mat_mul(vector<vector<Expression>> m1,
						    vector<vector<Expression>> m2);

  /* matrix-vector multiplication */
  vector<Expression> expr_mat_vec_mul(vector<vector<Expression>> m,
				      vector<Expression> v);
  
  vector<vector<Expression>> expr_zero_pad(vector<vector<Expression>> m,
					   int row_pad_before,
					   int row_pad_after,
					   int col_pad_before,
					   int col_pad_after);

  vector<vector<Expression>> expr_simple_zero_pad(vector<vector<Expression>> m, int row_pad, int col_pad);

  
  Expression expr_vec_max(vector<Expression> v);

  Expression expr_mat_max(vector<vector<Expression>> m);

  Expression expr_vec_max_xo(vector<Expression> v);

  Expression expr_mat_max_xo(vector<vector<Expression>> m);

  Expression expr_mat_max_special(vector<vector<Expression>> m);
  
  vector<Expression> expr_vec_sigmoid(vector<Expression> v);

  vector<Expression> expr_vec_relu(vector<Expression> v);

  vector<Expression> expr_vec_tanh(vector<Expression> v);
  
  Formula vec_conjunction(vector<Formula> v);

  Formula vec_disjunction(vector<Formula> v);
}
