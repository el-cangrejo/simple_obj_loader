#ifndef OBJ_HPP
#define OBJ_HPP

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

class Vertex {
public:
	Vertex ();
	Vertex (float, float, float);
	Vertex (const Vertex&);
	virtual ~Vertex ();

  	float x;
	float y;
	float z;
};

class Triangle
{
public:
	Triangle ();
	Triangle (Vertex, Vertex, Vertex);
	Triangle (const Triangle&);
	virtual ~Triangle ();

	Vertex v1;
	Vertex v2;
	Vertex v3;
};

class Pair
{
public:
	Pair (int, int);
	~Pair ();

	int v;
	int n;
};


Vertex::Vertex (float x, float y, float z) : x(x), y(y), z(z) {}
Vertex::Vertex () : x(0), y(0), z(0) {}
Vertex::Vertex (const Vertex& v) : x(v.x), y(v.y), z(v.z) {}
Vertex::~Vertex () {}

Triangle::Triangle () : v1(Vertex(0, 0, 0)), v2(Vertex(0, 0, 0)), v3(Vertex(0, 0, 0)) {}
Triangle::Triangle (Vertex v1, Vertex v2, Vertex v3) : v1(v1), v2(v2), v3(v3) {}
Triangle::Triangle (const Triangle& t) : v1( t.v1 ), v2( t.v2 ), v3( t.v3 ) {}
Triangle::~Triangle () {}

Pair::Pair (int x, int y) : v(x), n(y) {}
Pair::~Pair () {}

std::ifstream objecttext;
std::string gline;

std::vector<Vertex> vertices;
std::vector<Vertex> normals;
std::vector<Triangle> triangles;
std::vector<Pair> pairs;
std::vector<Pair> pairs1;

int diffx;
int diffz;

bool point, line, triangle, normal;

#endif // OBJ_HPP