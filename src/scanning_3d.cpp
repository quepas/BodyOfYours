#include "scanning_3d.h"

Scanning3D::Scanning3D(Scanner3D* scanner3d)
  : scanner_(scanner3d)
{

}

void Scanning3D::run()
{
  scanner_->GrabCamera();
}
