#include "MeshComponents.hpp"
#include "Mesh.hpp"

Mesh::Mesh(void) {}

Mesh::~Mesh(void) {}

void Mesh::computeDualVertices(void) {
  if (triangles.size() == 0 || vertices.size() == 0) {
    std::cout << "Unitialized mesh\n";
  }

  for (const auto& t : triangles) {
    Vertex dv((vertices[t.v1].x + vertices[t.v2].x + vertices[t.v3].x)/3,
              (vertices[t.v1].y + vertices[t.v2].y + vertices[t.v3].y)/3,
              (vertices[t.v1].z + vertices[t.v2].z + vertices[t.v3].z)/3);
    dvertices.push_back(dv);
  }
}

void Mesh::computeDualEdges(void) {
  if (triangles.size() == 0 || vertices.size() == 0 || dvertices.size() == 0) {
    std::cout << "Unitialized mesh\n";
  }

  for (int i = 0; i < triangles.size(); ++i) {
    int count = 0;
    for (int j = 0; j < triangles.size(); ++j) {
      if (triangles[i].areNeighbors(triangles[j])) {
        Edge de(i, j);
        ++count;
        auto result =std::find(dedges.begin(), dedges.end(), de);
        if (result == dedges.end()) {
          dedges.push_back(de);
        }
      }
      if (count == 3) break;
    }
  }
}

void Mesh::findNeighbors(void) {
  std::vector<std::vector<int>> temp_vec(vertices.size());
  for (const auto &e : edges) {
    temp_vec[e.v1].push_back(e.v2);
    temp_vec[e.v2].push_back(e.v1);
  }
  neighbors = temp_vec;
}

std::vector<int> Mesh::findNearestNeighbors(int queryIdx, float radius) {
  std::vector<int> nneighbors;
  Vertex query = vertices[queryIdx];
  for (int i = 0; i < vertices.size(); ++i) {
    Vertex test = vertices[i];
    Vertex dif = query - test;
    if (dif.L2Norm() >= radius) continue;

    nneighbors.push_back(i);
  }
  return nneighbors;
}

void Mesh::computeNormals(void) {

  if (normals.size() == vertices.size()) {
    std::cout << "Normals already exist! : " << normals.size() << "\n";
    return;
  }

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();
  std::cout << "Calculating normals begin\n";

  std::vector<Vertex> norms(vertices.size());

  for (auto &t : triangles) {
    Vertex pa, pb, pc;
    Vertex diff1, diff2;
    Vertex trinorm;

    pa = vertices[t.v1];
    pb = vertices[t.v2];
    pc = vertices[t.v3];

    diff1 = pb - pa;
    diff2 = pc - pa;
    trinorm = diff1.Cross(diff2);
    trinorm = trinorm.Normalize();
    trinormals.push_back(trinorm);

    float theta1 = (pb-pa).Angle(pc-pa);
    float theta2 = (pa-pb).Angle(pc-pb);
    float theta3 = (pb-pc).Angle(pa-pc);

    norms[t.v1] = norms[t.v1] + trinorm * t.Area(vertices) * theta1;
    norms[t.v2] = norms[t.v2] + trinorm * t.Area(vertices) * theta2;
    norms[t.v3] = norms[t.v3] + trinorm * t.Area(vertices) * theta3;
  }

  for (auto &n : norms) {
    Vertex v = n.Normalize();
    normals.push_back(v);
  }

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Calculating normals end : elapsed time: " << elapsed_secs << "\n";
}

void Mesh::fittoUnitSphere(void) {
	float max_dist(0.0);
	for (const auto &v : vertices)
		if (max_dist < v.L2Norm()) max_dist = v.L2Norm();

	for (auto &v : vertices) {
		v.x /= max_dist;
		v.y /= max_dist;
		v.z /= max_dist;
	}
}

void Mesh::movetoCenter(void) {
  for (auto &v : vertices) {
  	v = v - centroid;
  }
}

void Mesh::print(void) {
  // Prints Information about the mesh
  std::cout << "Object size : \n"
  << this->vertices.size() << " vertices \n"
  << this->triangles.size() << " triangles \n"
  << this->edges.size() << " edges \n"
  << this->normals.size() << " normals \n"
  << this->dvertices.size() << " dvertices \n"
  << this->dedges.size() << " dedges \n"
  << this->trinormals.size() << " trinormals \n";
}