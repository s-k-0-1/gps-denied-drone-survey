#ifndef WHYCODE__TYPES_H
#define WHYCODE__TYPES_H


namespace whycode {

struct Parameters {
  // static params
  int id_bits;
  int id_samples = 360;
  int hamming_dist = 1;
  
  // dynamic params
  bool draw_coords = true;
  bool draw_segments = true;
  bool use_gui = true;
  bool identify = true;
  int coords_method;
  int num_markers = 1;
  int min_size = 100;           // minimum total size (outer + inner)
  int max_size = 1000000;       // maximum total size (outer + inner)
  int min_size_outer = 50;      // minimum outer segment size
  int min_size_inner = 30;      // minimum inner segment size
  double circle_diameter = 0.122;
  double calib_dist_x = 1.0;
  double calib_dist_y = 1.0;
  double initial_circularity_tolerance = 100.0;
  double final_circularity_tolerance = 2.0;
  double area_ratio_tolerance = 40.0;
  double center_distance_tolerance_ratio = 10.0;
  double center_distance_tolerance_abs = 5.0;

  bool operator==(const Parameters&) const = default;
};

struct SDecoded {
  float angle;    // axis rotation angle
  int id;         // marker decoded ID
  int edgeIndex;  // idx of starting edge
};

// this structure contains information related to image coordinates and dimensions of the detected pattern
struct SSegment {
  float x;                    // center in image coordinates
  float y;                    // center in image coordinates
  float angle, horizontal;    // orientation (not really used in this case, see the SwarmCon version of this software)
  int size;                   // number of pixels
  int maxy, maxx, miny, minx; // bounding box dimensions
  int mean;                   // mean brightness
  int type;                   // black or white ?
  float roundness;            // result of the first roundness test, see Eq. 2 of paper [1]
  float bwRatio;              // ratio of white to black pixels, see Algorithm 2 of paper [1]
  bool round;                 // segment passed the initial roundness test
  bool valid;                 // marker passed all tests and will be passed to the transformation phase
  float m0, m1;               // eigenvalues of the pattern's covariance matrix, see Section 3.3 of [1]
  float v0, v1;               // eigenvectors of the pattern's covariance matrix, see Section 3.3 of [1]
  float r0, r1;               // ratio of inner vs outer ellipse dimensions (used to establish ID, see the SwarmCon version of this class)
  int ID;                     // pattern ID
};

// which transform to use
typedef enum {
  TRANSFORM_NONE = 0,     // camera-centric
  TRANSFORM_2D = 1,       // 3D->2D homography
  TRANSFORM_3D = 2,       // 3D user-defined - linear combination of four translation/rotation transforms
  TRANSFORM_4D = 3,       // 3D user-defined - full 4x3 matrix
  TRANSFORM_INV = 4,      // for testing purposes
  TRANSFORM_NUMBER = 5
} ETransformType;

struct STrackedObject {
  float u, v;                 // real center in the image coords
  float x, y, z, d;           // position and distance in the camera coords
  float roll, pitch, yaw;     // fixed axis angles
  float angle;                // axis angle around marker's surface normal
  float n0, n1, n2;           // marker surface normal pointing from the camera
  float qx, qy, qz, qw;       // quaternion
};

struct SEllipseCenters {
  float u[2];     // retransformed x coords
  float v[2];     // retransformed y coords
  float n[2][3];  // both solutions of marker's surface normal
  float t[2][3];  // both solutions of position vector
};

// rotation/translation model of the 3D transformation                                                                                                                  
struct S3DTransform {
  float orig[3];      // translation {x, y, z}
  float simlar[9];    // rotation description, similarity transformation matrix
};

struct SMarker {
  bool valid;
  SSegment seg;
  STrackedObject obj;
  int outer_size;             // outer segment size before summing
  int inner_size;             // inner segment size
};

struct CalibrationConfig {
  float grid_dim_x_;              // x unit dimention of 2D coordinate
  float grid_dim_y_;              // y unit dimention of 2D coordinate
  float hom_[9];                  // transformation description for 2D
  S3DTransform D3transform_[4];   // transformation description for 3D
};

}  // namespace whycode

#endif  // WHYCODE__TYPES_H
