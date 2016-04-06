#ifndef OBJLOADER_HPP
#define OBJLOADER_HPP

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>

using std::cout;
using std::endl;


std::ifstream objecttext;
std::string gline;

int diffx;
int diffz;

float scale;

bool point, line, triangle, normal;

// Camera variables
float rotFactor;
float zoomStep;
float cdist;
float eyex;
float eyey;
float eyez;
float azimuthAngle;
float altitudeAngle;

// Event handle variables
bool mouseClickDown;
int mx0;
int my0;

#endif // OBJ_HPP