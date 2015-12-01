#ifndef OBJ_HPP
#define OBJ_HPP

#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>

using std::cout;
using std::endl;

class Vertex
{
public:
	float x;
	float y;
	float z;
	Vertex();
	Vertex(float x, float y, float z);
	Vertex(const Vertex& v);
	virtual ~Vertex();
};

class Edge
{
	Vertex vert1;	
	Vertex vert2;
public:
	Edge(Vertex v1, Vertex v2);
	Edge();
	virtual ~Edge();
};

class Triangle
{
public:
	Vertex v1;
	Vertex v2;
	Vertex v3;
	Triangle();
	Triangle(Vertex v1, Vertex v2, Vertex v3);
	Triangle(const Triangle& t);
	virtual ~Triangle();
};

Vertex::Vertex( float x , float y, float z ) : x(x), y(y), z(z) {}
Vertex::Vertex() : x(0), y(0), z(0) {}
Vertex::Vertex( const Vertex& v ) : x(v.x), y(v.y), z(v.z) {}
Vertex::~Vertex() {}

Edge::Edge( Vertex v1 , Vertex v2 ) : vert1(v1), vert2(v2) {}
Edge::Edge( ) : vert1(), vert2() {}
Edge::~Edge() {}

Triangle::Triangle() : v1(Vertex(0, 0, 0)), v2(Vertex(0, 0, 0)), v3(Vertex(0, 0, 0)) {}
Triangle::Triangle(Vertex v1, Vertex v2, Vertex v3) : v1(v1), v2(v2), v3(v3) {}
Triangle::Triangle(const Triangle& t) : v1( t.v1 ), v2( t.v2 ), v3( t.v3 ) {}
Triangle::~Triangle() {}

std::ifstream objecttext;
std::string gline;

std::vector<Vertex> vertices;
std::vector<Vertex> normals;
std::vector<Triangle> triangles;


int diffx;
int diffz;

bool point, line, triangle, normal;

#endif // OBJ_HPP