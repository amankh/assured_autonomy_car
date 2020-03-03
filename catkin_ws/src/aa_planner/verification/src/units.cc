#include "dreal/dreal.h"

namespace dreal {
  Expression sigmoid(const Expression& e) {
    return (1 / (1 + exp(-e)));
  }

  Expression relu(const Expression& e) {
    return (max(0,e));
  }
  
  Formula bound(const Expression& e1, const Expression& e2, const Expression& e3) {
    return (e2 <= e1 && e1 <= e3);
  }
}
