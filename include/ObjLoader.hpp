#ifndef OBJLOADER_HPP
#define OBJLOADER_HPP

int loadObj(const std::string filepath, Mesh& mesh);

int findType(const std::string line);

#endif // OBJLOADER_HPP