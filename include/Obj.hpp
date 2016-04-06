#ifndef OBJ_HPP
#define OBJ_HPP

#include <GL/glut.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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

#define PI 3.1415927

class Vertex {
public:
	Vertex();
	Vertex(float, float, float);
	Vertex(Vertex&&);
	Vertex(const Vertex&);
	Vertex& operator=(Vertex&&);
	Vertex& operator=(const Vertex&);
	virtual ~Vertex();

  	float x;
	float y;
	float z;
};

class Triangle
{
public:
	Triangle();
	Triangle(int, int, int);
	Triangle(Triangle&&);
	Triangle(const Triangle&);
	Triangle& operator=(Triangle&&);
	Triangle& operator=(const Triangle&);
	virtual ~Triangle();

	int v1;
	int v2;
	int v3;
};

class Pair
{
public:
	Pair(int, int);
	~Pair();

	int v;
	int n;
};

class Color {
public:
	Color();
	Color(float, float, float);
	Color(Color&&);
	Color(const Color&);
	Color& operator=(Color&&);
	Color& operator=(const Color&);
	virtual ~Color();

  	float r;
	float g;
	float b;
};

Vertex::Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
Vertex::Vertex() : x(0), y(0), z(0) {}
Vertex::Vertex(Vertex&& v) = default;
Vertex::Vertex(const Vertex& v) = default;
Vertex& Vertex::operator=(Vertex&& v) = default;
Vertex& Vertex::operator=(const Vertex& v) = default;
Vertex::~Vertex() {}

Triangle::Triangle() : v1(0), v2(0), v3(0) {}
Triangle::Triangle(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {}
Triangle::Triangle(Triangle&& t) = default;
Triangle::Triangle(const Triangle& t) = default;
Triangle& Triangle::operator=(Triangle&& t) = default;
Triangle& Triangle::operator=(const Triangle& t) = default;
Triangle::~Triangle() {}

Color::Color(float x, float y, float z) : r(x), g(y), b(z) {}
Color::Color() : r(0), g(0), b(0) {}
Color::Color(Color&& c) = default;
Color::Color(const Color& c) = default;
Color& Color::operator=(Color&& c) = default;
Color& Color::operator=(const Color& c) = default;
Color::~Color() {}

Pair::Pair(int x, int y) : v(x), n(y) {}
Pair::~Pair() {}

std::vector<Vertex> calc_normals(const std::vector<Vertex> &verts);

std::ifstream objecttext;
std::string gline;

std::vector<Vertex> vertices;
std::vector<Vertex> normals;
std::vector<Triangle> triangles;
std::vector<Color> colors;
std::vector<Pair> pairs;
std::vector<Pair> pairs1;

std::vector<Vertex> norms;
std::vector<Vertex> vert_filt;

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