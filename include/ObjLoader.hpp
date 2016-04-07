#ifndef OBJLOADER_HPP
#define OBJLOADER_HPP

#include "Mesh.hpp"

#include <string>
#include <vector>

int loadObj(const std::string filepath, Mesh& mesh);

int findType(const std::string line);

#endif // OBJLOADER_HPP