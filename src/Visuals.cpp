#include "Visuals.hpp"

Mesh mesh;

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
static float zoomStep = 0.1;
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

  glFlush();
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
  
  drawMesh (mesh);
  drawAxis ();
}

void initializeGL(void) {
  glClearColor(0.9, 0.9, 0.9, 0.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
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
    showVerts = !showVerts;
    break;
    }
  case 'l' : {
    showLines = !showLines;
    break;
    }
  case 't' : {
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
    showNormals = !showNormals;
    break;
    }
  case 'b' : {
    showTriNormals = !showTriNormals;
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

void resize(int w, int h) {
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (float)w/h, 0.01, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
