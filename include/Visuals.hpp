#ifndef VISUALS_HPP
#define VISUALS_HPP

#include "Open3DOR.hpp"
#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <ctime>

#define PI 3.1415927

// Drawing Functions
void drawMesh(Mesh&);
void drawAxis();

// OpenGL Handling Functions
void renderScene(void);
void resize(int, int);

// Event Handling Functions
void mouseClick(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboardDown(unsigned char key, int x, int y);

#endif // VISUALS_HPP
