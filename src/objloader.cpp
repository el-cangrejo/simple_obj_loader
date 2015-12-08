#include "Obj.hpp"

void RenderScene (void) {
  glClear(GL_COLOR_BUFFER_BIT );
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0.2*sin(diffx), 0, 0.2*cos(diffz),
            0, 0, 0,
            0, 1, 0);
  glColor3f(1.0, 0.0, 0.0);
     
  if (point == true) {
    glBegin(GL_POINTS);      
    for (const auto& vertex : vertices) 
      glVertex3f( vertex.x/scale, vertex.y/scale, vertex.z/scale);
    glEnd();
  } else if (triangle == true) {
    glBegin(GL_TRIANGLES);
    for (const auto& triangle : triangles) {
      glVertex3f(triangle.v1.x/scale, triangle.v1.y/scale, triangle.v1.z/scale);
      glVertex3f(triangle.v2.x/scale, triangle.v2.y/scale, triangle.v2.z/scale);
      glVertex3f(triangle.v3.x/scale, triangle.v3.y/scale, triangle.v3.z/scale);    
    }
    glEnd();
    glColor3f(0.0, 1.0, 0.0);    
    glBegin(GL_LINES);
    for (const auto& triangle : triangles) {  
      glVertex3f(triangle.v1.x/scale, triangle.v1.y/scale, triangle.v1.z/scale);
      glVertex3f(triangle.v2.x/scale, triangle.v2.y/scale, triangle.v2.z/scale);
      glVertex3f(triangle.v2.x/scale, triangle.v2.y/scale, triangle.v2.z/scale);
      glVertex3f(triangle.v3.x/scale, triangle.v3.y/scale, triangle.v3.z/scale);
      glVertex3f(triangle.v3.x/scale, triangle.v3.y/scale, triangle.v3.z/scale);
      glVertex3f(triangle.v1.x/scale, triangle.v1.y/scale, triangle.v1.z/scale);
    }
    glEnd();
  } else if (line == true) {
    glBegin(GL_LINES);
    for (const auto& triangle : triangles) {
      glVertex3f(triangle.v1.x/scale, triangle.v1.y/scale, triangle.v1.z/scale);
      glVertex3f(triangle.v2.x/scale, triangle.v2.y/scale, triangle.v2.z/scale);
      
      glVertex3f(triangle.v2.x/scale, triangle.v2.y/scale, triangle.v2.z/scale);
      glVertex3f(triangle.v3.x/scale, triangle.v3.y/scale, triangle.v3.z/scale);
      
      glVertex3f(triangle.v3.x/scale, triangle.v3.y/scale, triangle.v3.z/scale);
      glVertex3f(triangle.v1.x/scale, triangle.v1.y/scale, triangle.v1.z/scale);
    }
    glEnd();
  }

  if (normal == true) {
    glBegin(GL_LINES);
    for (const auto& pair : pairs1) {
      Vertex vert = vertices[pair.v];
      Vertex norm = normals[pair.n];
      glColor3f(1.0, 1.0, 0.0);    
      glVertex3f(vert.x/scale, vert.y/scale, vert.z/scale);
      glVertex3f(vert.x/scale + norm.x/scale, vert.y/scale + norm.y/scale, vert.z/scale + norm.z/scale);
    }
    glEnd();
  }
  glFlush();
}

void KeyEvent (unsigned char key, int x, int y) {
  switch (key) {
    case 'q' :
    exit(0);

    case 'p' : {
      point = true; 
      line = false; 
      triangle = false;
      break;
    }
    case 'l' : {
      line = true; 
      point = false; 
      triangle = false;
      break;
    }
    case 't' : {
      triangle = true; 
      line = false; 
      point = false;
      break;
    }
    case 'n' : {
      if (!normals.empty()) {
        normal = true;
      } else {
        cout << "There are no normals!" << endl; 
      }
      break;
    }
    case 'm' : {
      normal = false;
      break;
    }
    default : {
      triangle = false; 
      line = false; 
      point = false;
      break;
    }
  }
  glutPostRedisplay();
}

void MouseEvent (int button, int state, int x, int y) {
}

void MouseMotion (int x, int y) {
  diffx = 0.5 * x;
  diffz = 0.5 * x;
  glutPostRedisplay();
}

int main (int argc, char **argv) {
  if (argc != 2) {
    printf("usage: Obj.out <Object Path>\n");
    return -1;
  }
    
  objecttext.open(argv[1]);  

  if (objecttext.is_open()) {
    while (getline(objecttext,gline)) {
      std::istringstream in(gline);      //make a stream for the line itself
      std::string type;
      in >> type; 

      if (type == "v") {
        float x, y, z;
        in >> x >> y >> z;

        Vertex v(x, y, z);
        vertices.push_back(v);
      } else if (type == "vn") {
        float x, y, z;
        in >> x >> y >> z;

        Vertex vn(x, y, z);
        normals.push_back(vn);
      } else if (type == "f") {
        int v1, v2, v3, n1, n2, n3;
        char s1, s2;
        in >> v1 >> s1 >> s2 >> n1;
        in >> v2 >> s1 >> s2 >> n2; 
        in >> v3 >> s1 >> s2 >> n3;
        //in >> v1 >> v2 >> v3; // For b66_L2.obj, icosahedron.obj file 
        Pair p1(v1 - 1, n1 - 1);
        Pair p2(v2 - 1, n2 - 1);
        Pair p3(v3 - 1, n3 - 1);

        pairs.push_back(p1);
        pairs.push_back(p2);
        pairs.push_back(p3);
        
        Triangle t(vertices[v1 - 1], vertices[v2 - 1], vertices[v3 - 1]);        
        triangles.push_back(t);
      }
    }
    objecttext.close(); 
  }
  
  std::sort(pairs.begin(), pairs.end(), 
    [](const Pair& p1, const Pair& p2) -> bool { return p1.v > p2.v; });
  std::unique_copy(pairs.begin(), pairs.end(), std::back_inserter(pairs1), 
    [](const Pair& p1, const Pair& p2){ return p1.v == p2.v; });

  std::cout << "Object size = " << vertices.size() << " vertices \n";
  std::cout << "Object size = " << triangles.size() << " triangles \n";
  std::cout << "Object size = " << pairs1.size() << " normals \n";
  
  diffz = 1;
  diffx = 1;  
  scale = 60;
  point = true;
  normal = false;

  glutInit(&argc, argv);  
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB );
  glutInitWindowPosition(50,25);
  glutInitWindowSize(1024,720); //w h
  glutCreateWindow("Dafuq");
  glClearColor(0.0,0.0,0.0,0.0); 

  glutDisplayFunc(RenderScene);
  glutKeyboardFunc(KeyEvent);
  glutMouseFunc(MouseEvent);
  glutPassiveMotionFunc(MouseMotion);

  glutMainLoop();

  return 0;
}