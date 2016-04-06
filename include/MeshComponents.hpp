#ifndef OPEN3DOR_HPP
#define OPEN3DOR_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <ctime>


class Vertex {
public:
	Vertex(void);
	Vertex(float, float, float);
	Vertex(Vertex&&);
	Vertex(const Vertex&);
	
	float L2Norm(void) const;
	float L2Norm(const Vertex&) const;
	float Dot(const Vertex&);
	Vertex Cross(const Vertex&);
	Vertex Normalize();
	float Angle(const Vertex&);
	
	bool operator==(const Vertex&) const;
	Vertex& operator=(const Vertex&);
	Vertex& operator=(Vertex&&);
	Vertex operator-(const Vertex&);
	Vertex operator+(const Vertex&);
	Vertex operator*(float);
	Vertex operator/(float);
	
	virtual ~Vertex();

	float x;
	float y;
	float z;
};

class Edge {
public:
	Edge(void);
	Edge(int, int);
	Edge(Edge&&);
	Edge(const Edge&);
	
	Edge& operator=(Edge&&);
	Edge& operator=(const Edge&);
	bool operator==(const Edge&) const;
	
	virtual ~Edge();

	int v1;
	int v2;
};

class Triangle {
public:
	Triangle(void);
	Triangle(int, int, int);
	Triangle(Triangle&&);
	Triangle(const Triangle&);
	
	bool areNeighbors(const Triangle&);
	float Area(std::vector<Vertex>&);
	
	Triangle& operator=(Triangle&&);
	Triangle& operator=(const Triangle&);

	virtual ~Triangle();

	int v1;
	int v2;
	int v3;
};



void read_mesh(const std::string, Mesh&);
void preprocess_mesh(Mesh&);

#endif // OPEN3DOR_HPP