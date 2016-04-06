#include "graphics.hpp"

Mesh qmesh, tmesh;
extern const std::vector<int> vidx;

// Display variables
static bool showVerts = true;
static bool showLines = false;
static bool showTriangles = false;
static bool showNormals = false;
static bool showTriNormals = false;
static bool showDVerts = false;
static bool showEdges = false;
static bool showDEdges = false;
static bool sphere = false;
static bool query_object = false;
static bool target_object1 = false;
static bool target_object2 = false;
static bool showGrid = false;

// Camera variables
static float rotFactor = 0.01;
static float zoomStep = 0.01;
static float cdist = 2;
static float eyex = 0.0;
static float eyey = 0.0;
static float eyez = cdist;
static float azimuthAngle = 0.0;
static float altitudeAngle = PI/2.;
Vertex up_vector(0, 1, 0);
// Event handle variables
static bool mouseClickDown = false;
static int mx0 = 0;
static int my0 = 0;

//
static float angle_x = 0;
static float rot_step =   5;

void drawMesh(Mesh &mesh) {
  if (showVerts == true) {
    glColor3f(.9, 0.0, 0.0);
    glBegin(GL_POINTS);
    for (const auto& v : mesh.vertices)
      glVertex3f(v.x, v.y, v.z);
    glEnd();
  }
  if (showTriangles == true) {
    glEnable(GL_DEPTH_TEST);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glShadeModel(GL_SMOOTH);
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_TRIANGLES);
    //glPolygonMode(GL_FRONT, GL_FILL);
    for (const auto &t : mesh.triangles) {
      //glColor3f(0.9, 0.1, 0.1);
      glNormal3f(mesh.normals[t.v1].x,
                 mesh.normals[t.v1].y,
                 mesh.normals[t.v1].z);
      glVertex3f(mesh.vertices[t.v1].x,
                 mesh.vertices[t.v1].y,
                 mesh.vertices[t.v1].z);
      //glColor3f(0.1, 0.9, 0.1);
      glNormal3f(mesh.normals[t.v2].x,
                 mesh.normals[t.v2].y,
                 mesh.normals[t.v2].z);
      glVertex3f(mesh.vertices[t.v2].x,
                 mesh.vertices[t.v2].y,
                 mesh.vertices[t.v2].z);
      //glColor3f(0.1, 0.1, 0.9);
      glNormal3f(mesh.normals[t.v3].x,
                 mesh.normals[t.v3].y,
                 mesh.normals[t.v3].z);
      glVertex3f(mesh.vertices[t.v3].x,
                 mesh.vertices[t.v3].y,
                 mesh.vertices[t.v3].z);
    }
    glEnd();
    //glColor3f(1.0, 0.0, 1.0);
    //glBegin(GL_LINES);
    // for (const auto &t : mesh.triangles) {
    //   glVertex3f(mesh.vertices[t.v1].x,
    //             mesh.vertices[t.v1].y,
    //             mesh.vertices[t.v1].z);
    //   glVertex3f(mesh.vertices[t.v2].x,
    //             mesh.vertices[t.v2].y,
    //             mesh.vertices[t.v2].z);

    //   glVertex3f(mesh.vertices[t.v2].x,
    //             mesh.vertices[t.v2].y,
    //             mesh.vertices[t.v2].z);
    //   glVertex3f(mesh.vertices[t.v3].x,
    //             mesh.vertices[t.v3].y,
    //             mesh.vertices[t.v3].z);

    //   glVertex3f(mesh.vertices[t.v3].x,
    //             mesh.vertices[t.v3].y,
    //             mesh.vertices[t.v3].z);
    //   glVertex3f(mesh.vertices[t.v1].x,
    //             mesh.vertices[t.v1].y,
    //             mesh.vertices[t.v1].z);
    // }
    //glEnd();
  }
  if (showLines == true) {
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_LINES);
    for (const auto &t : mesh.triangles) {
      glVertex3f(mesh.vertices[t.v1].x,
                mesh.vertices[t.v1].y,
                mesh.vertices[t.v1].z);
      glVertex3f(mesh.vertices[t.v2].x,
                mesh.vertices[t.v2].y,
                mesh.vertices[t.v2].z);

      glVertex3f(mesh.vertices[t.v2].x,
                mesh.vertices[t.v2].y,
                mesh.vertices[t.v2].z);
      glVertex3f(mesh.vertices[t.v3].x,
                mesh.vertices[t.v3].y,
                mesh.vertices[t.v3].z);

      glVertex3f(mesh.vertices[t.v3].x,
                mesh.vertices[t.v3].y,
                mesh.vertices[t.v3].z);
      glVertex3f(mesh.vertices[t.v1].x,
                mesh.vertices[t.v1].y,
                mesh.vertices[t.v1].z);
    }
    glEnd();
  }
  if (showNormals == true) {
    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (int i = 0; i < mesh.normals.size(); ++i) {
      glVertex3f((mesh.vertices[i].x + 0.1 * mesh.normals[i].x),
                 (mesh.vertices[i].y + 0.1 * mesh.normals[i].y),
                 (mesh.vertices[i].z + 0.1 * mesh.normals[i].z));

      glVertex3f(mesh.vertices[i].x, mesh.vertices[i].y, mesh.vertices[i].z);
    }
    glEnd();
  }
  if (showTriNormals == true) {
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (int i = 0; i < mesh.trinormals.size(); ++i) {
      glVertex3f((mesh.dvertices[i].x + 0.1 * mesh.trinormals[i].x),
                 (mesh.dvertices[i].y + 0.1 * mesh.trinormals[i].y),
                 (mesh.dvertices[i].z + 0.1 * mesh.trinormals[i].z));

      glVertex3f(mesh.dvertices[i].x, mesh.dvertices[i].y, mesh.dvertices[i].z);
    }
    glEnd();
  }
  if (showDVerts == true) {
    glColor3f(1.0, 1.0, 0.0);
    glBegin(GL_POINTS);
    for (const auto& dv : mesh.dvertices) {
      glVertex3f(dv.x, dv.y, dv.z);
    }
    glEnd();
  }
  if (showEdges == true) {
    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_LINES);
    for (const auto &e : mesh.edges) {
      glVertex3f(mesh.vertices[e.v1].x,
                mesh.vertices[e.v1].y,
                mesh.vertices[e.v1].z);
      glVertex3f(mesh.vertices[e.v2].x,
                mesh.vertices[e.v2].y,
                mesh.vertices[e.v2].z);
    }
    glEnd();
  }
  if (showDEdges == true) {
    glColor3f(0.0, 1.0, 1.0);
    glBegin(GL_LINES);
    for (const auto &de : mesh.dedges) {
      glVertex3f(mesh.dvertices[de.v1].x,
                mesh.dvertices[de.v1].y,
                mesh.dvertices[de.v1].z);
      glVertex3f(mesh.dvertices[de.v2].x,
                mesh.dvertices[de.v2].y,
                mesh.dvertices[de.v2].z);
    }
    glEnd();
  }

  //debug_nn(mesh);

  //glutSwapBuffers();
  glFlush();
}

void drawGrid(float step_x, float step_y, float step_z) {

  Vertex max(0., 0., 0.);
  Vertex min(1., 1., 1.);

  for (const auto& v : qmesh.vertices) {
    if (v.x >= max.x) max.x = v.x;
    if (v.y >= max.y) max.y = v.y;
    if (v.z >= max.z) max.z = v.z;
    if (v.x <= min.x) min.x = v.x;
    if (v.y <= min.y) min.y = v.y;
    if (v.z <= min.z) min.z = v.z;
  }

  /*
  glPushMatrix();
    glTranslatef(max.x, max.y, max.z);
    glColor3f(1.0, 0.0, 0.0);
    glutWireSphere(0.1, 10, 10);
  glPopMatrix();

  glPushMatrix();
    glTranslatef(min.x, min.y, min.z);
    glColor3f(1.0, 0.0, 0.0);
    glutWireSphere(0.1, 10, 10);
  glPopMatrix();
  //*/

  float max_dist = std::max({fabs(max.x - min.x),
                             fabs(max.y - min.y),
                             fabs(max.z - min.z)});

  int dim_x = ceil(fabs(max.x - min.x) / step_x);
  int dim_y = ceil(fabs(max.y - min.y) / step_y);
  int dim_z = ceil(fabs(max.z - min.z) / step_z);

  //std::cout << "Grid size = " << dim_x << " * " << dim_y << " * " << dim_z << "\n";

  float displacement_x = min.x + step_x / 2;
  float displacement_y = min.y + step_y / 2;
  float displacement_z = min.z + step_z / 2;

  glTranslatef(displacement_x, displacement_y, displacement_z);

  //*
  auto v_idx = vidx.begin();
  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {
        if (*v_idx) {
          glPushMatrix();
            glColor3f((float)3  / grid_x, 
                      (float)3  / grid_y, 
                      (float)3  / grid_z);
            glTranslatef(grid_x * step_x, grid_y * step_y, grid_z * step_z);
            glutWireCube(step_x);
          glPopMatrix();
        }
        v_idx++;
      }
    }
  }
  //*/

  /*
  auto v_idx = vidx.begin();

  float radius = sqrt(pow(step_x / 2., 2) +
                      pow(step_y / 2., 2) +
                      pow(step_z / 2., 2));

  for (int grid_x = 0; grid_x < dim_x; ++grid_x) {
    for (int grid_y = 0; grid_y < dim_y; ++grid_y) {
      for (int grid_z = 0; grid_z < dim_z; ++grid_z) {
        if (*v_idx) {
          glPushMatrix();
            glColor3f(1.0, 1.0, 0.0);
            //glColor3f(grid_x / dim_x, grid_y / dim_y, grid_z / dim_z);
            //std::cout << "Color is " << grid_x / (float)dim_x << " "
                      //<< grid_y / (float)dim_y << " "
                      //<< grid_z / (float)dim_z << "\n";
            glTranslatef(grid_x * step_x, grid_y * step_y, grid_z * step_z);
            glutWireSphere(radius, 15, 5);
          glPopMatrix();
        }
        v_idx++;
      }
    }
  }
  std::cout << "grid :: " << dim_x * dim_y * dim_z << "\n";
  //*/
}

void drawAxis() {

    glPushMatrix();
        glColor3f(0.0, 0.0, 0.0);
        glutSolidSphere(0.05, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.5, 0.0, 0.0);
    glEnd();

    glPushMatrix();
        glTranslatef(0.5, 0.0, 0.0);
        glRotatef(90.0, 0.0, 1.0, 0.0);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.5, 0.0);
    glEnd();

    glPushMatrix();
        glTranslatef(0.0, 0.5, 0.0);
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();

    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.5);
    glEnd();

    glPushMatrix();
        glTranslatef(0.0, 0.0, 0.5);
        glutSolidCone(0.05, 0.1, 10, 10);
    glPopMatrix();
}

void renderScene(void) {
  
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(eyex, eyey, eyez,  // camera position
            0,  0,  0,         // target position
            up_vector.x,
            up_vector.y,
            up_vector.z);
  
  float grid_size(0.05);

  //*/
  glPushMatrix();
    //glTranslatef(0.4, 0, 0);
    //glScalef(1/3., 1/3., 1/3.);
    //glRotatef(angle_x, 0, 1, 0);
    // if (query_object) {
    //   drawMesh(qmesh);
    //   //drawGrid(grid_size, grid_size, grid_size);
    // }
    drawMesh(qmesh);

    if (showGrid) drawGrid(grid_size, grid_size, grid_size);

  glPopMatrix();
  //*/

  //*
  glPushMatrix();
    glTranslatef(-0.4, 0, 0);
    glScalef(1/3., 1/3., 1/3.);
    glRotatef(angle_x, 0, 1, 0);
    //if (target_object1)
    //drawMesh(tmesh);
  glPopMatrix();
  //*/
  glPushMatrix();
    glTranslatef(-0.5, -0.5, -0.5);
    glScalef(1/3., 1/3., 1/3.);
    drawAxis();
  glPopMatrix();

  if (sphere) {
    glColor3f(1.0, 1.0, 0.0);
    glutWireSphere(1.0,50,50);
  }
}

void mouseClick(int button, int state, int x, int y) {
  if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
      mouseClickDown = true;
      mx0 = x;
      my0 = y;
  } else {
      mouseClickDown = false;
  }
}

void keyboardDown(unsigned char key, int x, int y) {
  switch (key) {
  case 'i' : {
    cdist -= zoomStep;
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    break;
    }
  case 'o' : {
    cdist += zoomStep;
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    break;
    }
  case 'q' : {
    std::cout << "User evoked exiting..\n";
    exit(0);
    break;
    }
  case 'v' : {
    std::cout << "Showing Vertices \n";
    showVerts = !showVerts;
    break;
    }
  case 'l' : {
    std::cout << "Showing Lines \n";
    showLines = !showLines;
    break;
    }
  case 't' : {
    std::cout << "Showing Triangles \n";
    showTriangles = !showTriangles;
    break;
    }
  case 'd' : {
    showDVerts = !showDVerts;
    break;
    }
  case 'e' : {
    showEdges = !showEdges;
    break;
    }
  case 'k' : {
    showDEdges = !showDEdges;
    break;
    }
  case 'n' : {
    std::cout << "Showing Normals \n";
    showNormals = !showNormals;
    break;
    }
  case 'z' : {
    sphere = !sphere;
    break;
    }
  case 'b' : {
    showTriNormals = !showTriNormals;
    break;
    }
  case 'Q' : {
    query_object = !query_object;
    break;
    }
  case 'T' : {
    target_object1 = !target_object1;
    break;
    }
  case 'Y' : {
    target_object2 = !target_object2;
    break;
    }
  case 'G' : {
    showGrid = !showGrid;
    break;
    }
  default : {
    showVerts = true;
    showLines = false;
    showTriangles = false;
    showNormals = false;
    showDVerts = false;
    showEdges = false;
    query_object = false;
    target_object1 = false;
    target_object2 = false;
    break;
    }
  }
  glutPostRedisplay();
}

void mouseMotion(int x, int y) {
  if (mouseClickDown) {
    // Calculate angles
    azimuthAngle  -= (x-mx0)*rotFactor;
    altitudeAngle -= (y-my0)*rotFactor;
    // Set new camrea position
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);

    if ((altitudeAngle > 2 * PI) || (altitudeAngle < -2 * PI)) altitudeAngle = 0.0;

    if (altitudeAngle > PI) up_vector.y = -1;
    if (altitudeAngle < PI) up_vector.y = 1;
    if (altitudeAngle < 0) up_vector.y = -1;
    if (altitudeAngle < -PI) up_vector.y = 1;
    // Keep mouse x,y for next call
    mx0 = x;
    my0 = y;

    glutPostRedisplay();
  }
}

void idle(int x) {
  angle_x += rot_step;
  glutPostRedisplay();
  glutTimerFunc(10, idle, 0);
}

void resize(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)w/h, 0.01, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void initializeGL() {
  glClearColor(0.9, 0.9, 0.9, 0.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
}