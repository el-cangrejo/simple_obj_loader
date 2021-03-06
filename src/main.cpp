#include "Mesh.hpp"
#include "ObjLoader.hpp"
#include "Visuals.hpp"

#include <GL/glut.h>

extern Mesh mesh;

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "usage: ./ObjLoader.out <Target Object Path>\n";
    exit(1);
  }

  if (!loadObj(std::string(argv[1]), mesh))
    std::cout << "Could not load obj\n";

  mesh.print();
  mesh.movetoCenter();
  mesh.fittoUnitSphere();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowPosition(50, 25);
  glutInitWindowSize(900, 620); // w h
  glutCreateWindow("OpenGL Window");
  glClearColor(.9, .9, .9, 0.0);

  initializeGL();

  // GLUT Display Function
  glutDisplayFunc(renderScene);
  glutReshapeFunc(resize);

  // Keyboard handlers
  glutKeyboardFunc(keyboardDown);

  // Mouse handlers
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseMotion);

  // GLUT Main Lopp
  glutMainLoop();

  return 0;
}
