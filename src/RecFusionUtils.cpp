#include "RecFusionUtils.h"

using std::string;
using std::to_string;
using RecFusion::Mat4;

Mat4 identityMat4() {
  Mat4 mat;
  for (unsigned i = 0; i < 4; ++i) {
    for (unsigned j = 0; j < 4; ++j) {
      mat(i, j) = (i == j) ? 1 : 0;
    }
  }
  return mat;
}

QString mat4ToString(Mat4 matrix)
{
  QString str;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      str += matrix(row, col);
      if (row < 4) {
        str += " ,";
      }
    }
    str += "\n";
  }
  return str;
}
