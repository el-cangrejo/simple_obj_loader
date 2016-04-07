#ifndef MESH_HPP
#define MESH_HPP

#include "MeshComponents.hpp"

#include <vector>
#include <string>

class Mesh {
public:	
	Mesh(void);
	
	void computeDualVertices(void);
	void computeDualEdges(void);
	void findNeighbors(void);
	std::vector<int> findNearestNeighbors(int, float);
	void computeNormals(void);
	void fittoUnitSphere(void);
	void movetoCenter(void);
	void print(void);

	virtual ~Mesh(void);

	Vertex centroid;

	std::vector<Vertex> vertices;
	std::vector<Triangle> triangles;
	std::vector<Edge> edges;

	std::vector<Vertex> dvertices;
	std::vector<Edge> dedges;

	std::vector<Vertex> normals;	
	std::vector<Vertex> trinormals;
	
	std::vector<std::vector<int>> neighbors;
};

#endif // MESH_HPP