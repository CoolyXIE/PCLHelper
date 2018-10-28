#include "Camera.h"

namespace cooly
{
    PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy, int width, int height)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), width_(width_), height_(height_) {};

}

