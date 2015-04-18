#ifndef OBJ_GEOMETRY
#define OBJ_GEOMETRY
// This header file defines a tractable objects geometry representation.
// Since we only consider in-plane pushing, so it's sufficient to only
// focus on contact surface geometry modelling.

// Here, we sort all the vertices counter-clockwise and form edges along the way.
// All vertices and edges are represented in the local object reference frame.

#include <vector>
#include <matVec/matVec.h>
#include <assert.h>
#include <cmath>
#include <fstream>
#include <iostream>

struct Edge {
  Edge(const Vec& lp, const Vec& rp) {
    left_end = lp;
    right_end = rp;
    edge_vec = right_end - left_end;
    double axis_z[3] = {0,0,1};
    Vec z(axis_z,3);
    normal_dir = edge_vec ^ z;
    // Normalize to unit length.
    normal_dir = normal_dir / (sqrt(normal_dir * normal_dir));
  }
  // Get a sample point on the edge segment. 
  // 0 means the left_end, 1 means the right_end.
  Vec GetSample(double r) {
    assert( r >=0 && r<=1 );
    Vec sample_pt = left_end + edge_vec * r;
    return sample_pt;
  }
  Vec left_end;
  Vec right_end;
  Vec edge_vec;
  Vec normal_dir;
};

class ObjectGeometry {
 public:
  ObjectGeometry(){};
  void Serialize(std::ostream& fout) {
    assert(vertices.size() == edges.size());
    int numV = vertices.size();
    // Number of vertices.
    fout << numV << std::endl;
    // All vertices.
    for (int i = 0; i < numV; i++) {
      SerializeVector(vertices[i], fout);
    }
    // All edges.
    for (int i = 0; i < numV; i++) {
      // Just serialize left and right end is sufficient.
      SerializeVector(edges[i].left_end, fout);
      SerializeVector(edges[i].right_end, fout);
    }
  }

  void Deserialize(std::istream& fin) {
    int numV;
    // Get #vertices.
    fin >> numV; 
    vertices.clear();
    edges.clear();
    // Get all vertices.
    for (int i = 0; i < numV; i++) {
      Vec v = DeserializeVector(fin);
      vertices.push_back(v);
    }
    // Get all edges.
    for (int i = 0; i < numV; ++i) {
      Vec le = DeserializeVector(fin);
      Vec re = DeserializeVector(fin);
      Edge e(le,re);
      edges.push_back(e);
    }
    
  }
  std::vector<Vec> vertices;
  std::vector<Edge> edges;
 private:
  void SerializeVector(const Vec& v, std::ostream& fout) {
    fout << v[0] << " " << v[1] << " " << v[2] << std::endl;
  }
  Vec DeserializeVector(std::istream& fin) {
    Vec v(3);
    fin >> v[0] >> v[1] >> v[2];
    return v;
  }
};

#endif
