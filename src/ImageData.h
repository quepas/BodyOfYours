#pragma once

#include <QImage>

struct ImageData
{
  ImageData(QImage img, int w, int h)
    : image(img), width(w), height(h) {}
  QImage image;
  int width, height;
};
