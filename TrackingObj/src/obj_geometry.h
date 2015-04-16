#ifndef OBJ_GEOMETRY
#define OBJ_GEOMETRY
// This header file defines a tractable objects geometry representation.
// Since we only consider in-plane pushing, so it's sufficient to only
// focus on contact surface geometry modelling.

// Here, we sort all the vertices counter-clockwise and form edges along the way.
// All vertices and edges are represented in the local object reference frame.

#include <vector>
#include <matVec/matVec.h>

struct Edge {
  Edge(const Vec& lp, const Vec& rp) {
    left_end = lp;
    right_end = rp;
    edge_vec = right_end - left_end;
    Vec z({0,0,1},3);
    normal_dir = edge_vec ^ z;
    // Normalize to unit length.
    normal_dir = normal_dir / (sqrt(normal_dir * normal_dir));
  }
  // Get a sample point on the edge segment. 
  // 0 means the left_end, 1 means the right_end.
  Vec GetSample(double r) {
    assert( r >=0 && r<=1 );
    Vec sample_pt = left_end + r * (edge_vec);
    return sample_pt;
  }
  Vec left_end;
  Vec right_end;
  Vec edge_vec;
  Vec normal_dir;
};

class ObjectGeometry {
 public:
  ObjectGeometry();
  std::vector<Vec> vertices;
  std::vector<Edge> edges;
};

#endif
