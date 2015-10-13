#include <QGLViewer/qglviewer.h>
#include "RecFusion.h"
#include <QVector>

class Viewer : public QGLViewer
{
public:
  bool addMesh(RecFusion::Mesh* mesh);
  bool removeMesh(RecFusion::Mesh* mesh);
protected :
  virtual void draw();
  virtual void init();
private:
  QVector<RecFusion::Mesh*> meshes_;
};
