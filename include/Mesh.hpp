class Mesh {
public:	
	Mesh(void);
	
	void computeDualVertices(void);
	void computeDualEdges(void);
	void computeDualAdjacency(void);
	void findNeighbors(void);
	std::vector<int> findNearestNeighbors(int, float);
	void computeNormals(void);
	void fittoUnitSphere(void);
	void movetoCenter(void);

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