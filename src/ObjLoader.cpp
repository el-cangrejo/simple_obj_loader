#include "ObjLoader.hpp"

int loadObj(const std::string filepath, Mesh& mesh) {
  std::ifstream objfile;
  std::string line;
  int type(0);
  float maxdist(0.0);

  std::vector<Vertex> std_normals;
  // Reads .obj File
  objfile.open(filepath);
  if (objfile.is_open()) {
    std::cout << "Started reading " << filepath << "\n";
    while(getline(objfile, line)) {
      std::istringstream in(line);
      std::string element;
      in >> element;

      if (element == "v") {
        float x, y, z;
        in >> x >> y >> z;
        mesh.vertices.push_back(Vertex(x, y, z));

        mesh.centroid.x += x;
        mesh.centroid.y += y;
        mesh.centroid.z += z;

        float dist = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
        if ( dist > maxdist) maxdist = dist;
      } else if (element == "vn" || element == "n") {
        float nx, ny, nz;
        in >> nx >> ny >> nz;
        std_normals.push_back(Vertex(nx, ny, nz));
      } else if (element == "f") {
        if (type == 0 && !std_normals.empty()) mesh.normals.resize(mesh.vertices.size());

        if (type == 0) type = find_type(line);
        int v1, v2, v3;
        int vt1, vn1;
        int vt2, vn2;
        int vt3, vn3;
        
        switch (type) {
          case 1 : {
            in >> v1 >> v2 >> v3;
            break;
          }
          case 2 : {
            char c;
            in >> v1 >> c >> vt1;
            in >> v2 >> c >> vt2;
            in >> v3 >> c >> vt3;
            break;
          }
          case 3 : {
            char c;
            in >> v1 >> c >> c >> vn1;
            in >> v2 >> c >> c >> vn2;
            in >> v3 >> c >> c >> vn3;
            break;
          }
          case 4 : {
            char c;
            in >> v1 >> c >> vt1 >> c >> vn1;
            in >> v2 >> c >> vt2 >> c >> vn2;
            in >> v3 >> c >> vt3 >> c >> vn3;
            break;
          }
          default : {
            std::cout << "No such type! \n";
            exit(0);
            break;
          }
        }
        if ((type == 3 || type == 4) && !std_normals.empty()) {
          mesh.normals[v1 - 1] = std_normals[vn1 - 1];
          mesh.normals[v2 - 1] = std_normals[vn2 - 1];
          mesh.normals[v3 - 1] = std_normals[vn3 - 1];
        } else if ((type == 1 || type == 2) && !std_normals.empty()) {
          if (mesh.normals.size() != std_normals.size()) std::cout << "fuck\n";
          mesh.normals[v1 - 1] = std_normals[v1 - 1];
          mesh.normals[v2 - 1] = std_normals[v2 - 1];
          mesh.normals[v3 - 1] = std_normals[v3 - 1];
        }

        mesh.triangles.push_back(Triangle(v1 - 1, v2 - 1, v3 -1));
      }
    }
    objfile.close();
  } else {
    std::cout << "Unable to open file! \n";
  }

  mesh.centroid.x /= mesh.vertices.size();
  mesh.centroid.y /= mesh.vertices.size();
  mesh.centroid.z /= mesh.vertices.size();

  // Prints Information about the mesh
  std::cout << "Object size = " << mesh.vertices.size() << " vertices \n";
  std::cout << "Object size = " << mesh.triangles.size() << " triangles \n";
  std::cout << "Object size = " << mesh.normals.size() << " edges \n";
}

int findType(const std::string line) {
  int count(0);

  for (int i = 0; i < line.size(); ++i) {
    if (line[i] == '/') ++count;
  }
  
  if (count == 0) return 1;
  
  if (count == 3) return 2;

  for (int i = 0; i < line.size()-1; ++i)
    if ((line[i] == '/') && (line[i+1]) == '/') return 3;

  return 4;
}