#include "OGLDummy.h"

float ver[8][3] =
{
  { -1.0, -1.0, 1.0 },
  { -1.0, 1.0, 1.0 },
  { 1.0, 1.0, 1.0 },
  { 1.0, -1.0, 1.0 },
  { -1.0, -1.0, -1.0 },
  { -1.0, 1.0, -1.0 },
  { 1.0, 1.0, -1.0 },
  { 1.0, -1.0, -1.0 },
};

void quad(int a, int b, int c, int d)
{
  glBegin(GL_QUADS);
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3fv(ver[a]);

  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3fv(ver[b]);

  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3fv(ver[c]);

  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3fv(ver[d]);
  glEnd();
}

void drawDummy(float x /*= 0.0f*/, float y /*= 0.0f*/, float z /*= 0.0f*/, float scale /*= 1.0f*/)
{
  glTranslatef(x, y, z);
  glScalef(scale, scale, scale);
  quad(0, 3, 2, 1);
  quad(2, 3, 7, 6);
  quad(0, 4, 7, 3);
  quad(1, 2, 6, 5);
  quad(4, 5, 6, 7);
  quad(0, 1, 5, 4);
}
