/* Basic neural network units.
 * @author Michael A. Warren <mawarren@hrl.com>
 */

#include "dreal/dreal.h"

namespace dreal {

  Expression relu(const Expression& e);
  
  Expression sigmoid(const Expression& e);

  Formula bound(const Expression& e1, const Expression& e2, const Expression& e3);
  
}
