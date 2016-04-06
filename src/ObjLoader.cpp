#include "Obj.hpp"

void RenderScene (void) {
  glClear(GL_COLOR_BUFFER_BIT );
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glScalef(1/cdist, 1/cdist, 1/cdist);
  gluLookAt(eyex, eyey, eyez,  // camera position
            0,  0,  0,         // target position
            0,  1,  0);        // up vector

  glColor3f(1.0, 0.0, 0.0);
  
  int counter(0);
  
  if (point == true) {
    glBegin(GL_POINTS);      
    //for (const auto& vertex : vertices) 
      //glVertex3f(vertex.x/scale, vertex.y/scale, vertex.z/scale);
    glColor3f(1.0, 0.0, 1.0);
    for (int i = 0; i < vertices.size(); ++i) {
      //glColor3f(colors[i].r, colors[i].g, colors[i].b);
      glVertex3f(vertices[i].x/scale, vertices[i].y/scale, vertices[i].z/scale);
    }
    glEnd();
    glFlush();
  } else if (triangle == true) {
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < triangles.size(); ++i) {
      //cout << "Round = " << i << "\n";
      //cout << "Triangle = " << t.v1 << " " << t.v2 << " " << t.v3 << "\n";    
      glColor3f(colors[triangles[i].v1].r, colors[triangles[i].v1].g, colors[triangles[i].v1].b);
      glVertex3f(vertices[triangles[i].v1].x/scale, 
                 vertices[triangles[i].v1].y/scale, 
                 vertices[triangles[i].v1].z/scale);
      glVertex3f(vertices[triangles[i].v2].x/scale, 
                 vertices[triangles[i].v2].y/scale, 
                 vertices[triangles[i].v2].z/scale);
      glVertex3f(vertices[triangles[i].v3].x/scale, 
                 vertices[triangles[i].v3].y/scale, 
                 vertices[triangles[i].v3].z/scale);
      ++counter;
    }
    glEnd();
  } else if (line == true) {
    glBegin(GL_LINES);
    for (const auto& t : triangles) {
      glColor3f(colors[t.v1].r, colors[t.v1].g, colors[t.v1].b);
      glVertex3f(vertices[t.v1].x/scale, 
                vertices[t.v1].y/scale, 
                vertices[t.v1].z/scale);
      glVertex3f(vertices[t.v2].x/scale, 
                vertices[t.v2].y/scale, 
                vertices[t.v2].z/scale);
      
      glVertex3f(vertices[t.v2].x/scale, 
                vertices[t.v2].y/scale, 
                vertices[t.v2].z/scale);
      glVertex3f(vertices[t.v3].x/scale, 
                vertices[t.v3].y/scale, 
                vertices[t.v3].z/scale);
      
      glVertex3f(vertices[t.v3].x/scale, 
                vertices[t.v3].y/scale, 
                vertices[t.v3].z/scale);
      glVertex3f(vertices[t.v1].x/scale, 
                vertices[t.v1].y/scale, 
                vertices[t.v1].z/scale);
    }
    glEnd();
  }

  if (normal == true) {
    glBegin(GL_LINES);
    for (int i = 0, data_size = norms.size(); i < data_size; i += 10) {
      cout << "norma " << i << " " << norms[i].x << " " << norms[i].y << " " << norms[i].z << "\n";
      glColor3f(1.0, 1.0, 0.0);    
      glVertex3f(vertices[i].x/scale, vertices[i].y/scale, vertices[i].z/scale);
      glVertex3f(vertices[i].x/scale + norms[i].x/10, 
                 vertices[i].y/scale + norms[i].y/10, 
                 vertices[i].z/scale + norms[i].z/10);
    }
    glEnd();
  }
  glFlush();
}

void mouseClick(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON) {
        mouseClickDown = true;
        mx0 = x;
        my0 = y;
    } else {
        mouseClickDown = false;
    }
}

void keyboardDown(unsigned char key, int x, int y)
{
  switch(key)
  {
  case 'o' : {
    cdist -= zoomStep;
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    break;
    }
  case 'i' : {
    cdist += zoomStep;
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    break;
    }
  case 'q' : {
    exit(0);
    break;
    }
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
    normal = true;
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

void mouseMotion(int x, int y)
{
  if (mouseClickDown)
  {
    // Calculate angles
    azimuthAngle  -= (x-mx0)*rotFactor;
    altitudeAngle -= (y-my0)*rotFactor;
    // Set new camrea position
    eyex = cdist*sin(altitudeAngle)*sin(azimuthAngle);
    eyey = cdist*cos(altitudeAngle);
    eyez = cdist*sin(altitudeAngle)*cos(azimuthAngle);
    // Keep mouse x,y for next call
    mx0 = x;
    my0 = y;

    glutPostRedisplay();
  }
}

std::vector<Vertex> calc_normals(const std::vector<Vertex> &verts) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  //... read, pass in or create a point cloud ...
  cloud->points.resize(verts.size());
  for (int i = 0; i < verts.size(); ++i) {
      cloud->points[i].x = verts[i].x;
      cloud->points[i].y = verts[i].y;
      cloud->points[i].z = verts[i].z;
  }

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (2.0);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
  std::vector<Vertex> normalssss;

  for (int i = 0; i < cloud_normals->size(); ++i) {
    Vertex n;
    n.x = cloud_normals->points[i].normal[0];
    n.y = cloud_normals->points[i].normal[1]; 
    n.z = cloud_normals->points[i].normal[2]; 
    normalssss.push_back(n);
  }

  return normalssss;
}

std::vector<Vertex> filter_cloud(const std::vector<Vertex> &verts) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  //... read, pass in or create a point cloud ...
  cloud->points.resize(verts.size());
  cloud_filtered->points.resize(verts.size());
  for (int i = 0; i < verts.size(); ++i) {
      cloud->points[i].x = verts[i].x;
      cloud->points[i].y = verts[i].y;
      cloud->points[i].z = verts[i].z;
  }

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*cloud_filtered);

  std::vector<Vertex> filtered;

  for (int i = 0; i < cloud_filtered->points.size(); ++i) {
    Vertex v;
    v.x = cloud_filtered->points[i].x;  
    v.y = cloud_filtered->points[i].y;  
    v.z = cloud_filtered->points[i].z;  
    filtered.push_back(v);
  }

  return filtered;
}


int main(int argc, char **argv) {
  
  cout << "dafuq" << "\n";

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
        // in >> v1 >> s1 >> s2 >> n1;
        // in >> v2 >> s1 >> s2 >> n2; 
        // in >> v3 >> s1 >> s2 >> n3;
        in >> v1 >> v2 >> v3; // For b66_L2.obj, icosahedron.obj file 
        // Pair p1(v1 - 1, n1 - 1);
        // Pair p2(v2 - 1, n2 - 1);
        // Pair p3(v3 - 1, n3 - 1);

        // pairs.push_back(p1);
        // pairs.push_back(p2);
        // pairs.push_back(p3);
        
        Triangle t(v1 - 1, v2 - 1, v3 - 1);        
        triangles.push_back(t);
      } else if (type == "c") {
        float r, g, b;
        in >> b >> g >> r;
        Color c(r, g, b);
        colors.push_back(c);
      }
    }
    objecttext.close(); 
  }
  

  std::cout << "Object size = " << vertices.size() << " vertices \n";
  std::cout << "Object size = " << triangles.size() << " triangles \n";
  std::cout << "Object size = " << normals.size() << " normals \n";
  
  scale = 25;
  point = true;
  normal = false;
  
  
  //vert_filt = filter_cloud(vertices);
  norms = calc_normals(vertices);
  
  // Camera parameters
  rotFactor = 0.01;
  zoomStep  = 0.01;
  cdist = 0.5;
  eyex = 0.0;
  eyey = 0.0;
  eyez = cdist;
  azimuthAngle  = 0.0;
  altitudeAngle = PI/2.;
  mouseClickDown = false;
  
  glutInit(&argc, argv);  
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB );
  glutInitWindowPosition(50,25);
  glutInitWindowSize(900,620); //w h
  glutCreateWindow("Dafuq");
  glClearColor(0.0,0.0,0.0,0.0); 

  glutDisplayFunc(RenderScene);

  // Keyboard handlers
  glutKeyboardFunc(keyboardDown);

  // Mouse handlers
  glutMouseFunc(mouseClick);
  glutMotionFunc(mouseMotion);

  glutMainLoop();

  return 0;
}

