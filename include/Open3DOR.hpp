#ifndef OPEN3DOR_HPP
#define OPEN3DOR_HPP

#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <armadillo>

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

class Mesh {
public:	
	Mesh(void);
	
	void computeDualVertices(void);
	void computeDualEdges(void);
	void computeAdjacency(void);
	void computeDualAdjacency(void);
	void findNeighbors(void);
	std::vector<int> findNearestNeighbors(int, float);
	void computeNormals(void);
	void computeFPFH(void);
	void computeFV(void);
	void fittoUnitSphere(void);
	void movetoCenter(void);
	Mesh gridFilter(float, float, float);

	~Mesh();

	Vertex centroid;
	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;
	std::vector<Vertex> normals;
	std::vector<std::vector<float>> fpfhist;
	std::vector<std::vector<float>> fisher_vector;
	
	std::vector<Vertex> dvertices;
	std::vector<Vertex> trinormals;
	std::vector<Edge> edges;
	std::vector<Edge> dedges;
	std::vector<std::vector<int>> neighbors;
};

void read_mesh(const std::string, Mesh&);
void preprocess_mesh(Mesh&);
float local_distance(const Mesh&, const Mesh&);
float dist_L1(const std::vector<float>& , const std::vector<float>&);
int find_type(const std::string); 
void save_descriptors(const std::string , const Mesh&);
void load_descriptors(const std::string , Mesh&);
void preprocess_database();

#endif // OPEN3DOR_HPP