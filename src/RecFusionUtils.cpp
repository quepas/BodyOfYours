#include "RecFusionUtils.h"

using RecFusion::Mat4;

Mat4 IdentityMat4() {
  Mat4 mat;
  for (unsigned i = 0; i < 4; ++i) {
    for (unsigned j = 0; j < 4; ++j) {
      mat(i, j) = (i == j) ? 1 : 0;
    }
  }
  return mat;
}
