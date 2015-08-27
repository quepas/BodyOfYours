#include "RecFusionUtils.h"

using std::string;
using std::to_string;
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

string Mat4ToString(Mat4 matrix)
{
  string str;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      str += to_string(matrix(i, j));
      if (i < 4) {
        str += " ,";
      }
    }
    str += "\n";
  }
  return str;
}
