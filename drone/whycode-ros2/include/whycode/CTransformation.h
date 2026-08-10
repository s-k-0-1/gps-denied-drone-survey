#ifndef WHYCODE__CTRANSFORMATION_H
#define WHYCODE__CTRANSFORMATION_H

#include <string>
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

#include "whycode/types.h"


namespace whycode {

class CTransformation {
public:
  /* circle diameter of the pattern */
  CTransformation(float circle_diam);

  /* update of intrinsic and distortion camera params */
  void updateCameraParams(const std::array<double, 9> &intri, const std::vector<double> &dist);

  /* calculate marker 3D or 2D coordinates in user-defined coordinate system from the segment description provided by the CCircleDetector class, see 4.1-4.4 of [1] */
  SEllipseCenters calcSolutions(const SSegment segment);

  /* transform coordinates into desired system */
  void transformCoordinates(STrackedObject &obj);
  
  /* calculate orientation of marker */
  void calcOrientation(STrackedObject &obj);

  /* get/set transformation type */
  ETransformType getTransformType();

  void setTransformType(int trans);

  void setTransformType(ETransformType trans);

  /* update circle diameter */
  void setCircleDiameter(const float circle_diam);

  /* establish the user-defined coordinate system from four calibration patterns - see 4.4 of [1] */
  void calibrate2D(const STrackedObject *in, const float g_dim_x, const float g_dim_y, const float robot_radius = 0.0, const float robot_height = 0.0, const float camera_height = 1.0);

  void calibrate3D(const STrackedObject *o, const float g_dim_x, const float g_dim_y);

  void setCalibrationConfig(const CalibrationConfig &config);

  CalibrationConfig getCalibrationConfig();

  bool is_calibrated(){ return calibrated_; };

private:
  /* transform into planar coordinates */
  void transform2D(STrackedObject &o);

  /* transform into custom 3D coordinates */
  void transform3D(STrackedObject &o, const int num = 4);

  /* supporting method for partial calibrating custom 3D coordinates */
  S3DTransform calibrate3D(const STrackedObject &o0, const STrackedObject &o1, const STrackedObject &o2, const float g_dim_x, const float g_dim_y);

  /* image to canonical coords (unbarrel + focal center and length) */
  void transformXY(float &x,float &y);

  /* get back image coords from canonical coords */
  void reTransformXY(float &x, float &y, float &z);

  /* calculate quaternion from axis angle */
  void calcQuaternion(STrackedObject &obj);

  /* calculate RPY from quaternion */
  void calcEulerFromQuat(STrackedObject &obj);

  void normalize_quaternion(float &qx1, float &qy1, float &qz1, float &qw1);
  float quaternion_norm(float qx1, float qy1, float qz1, float qw1);
  void conjugate_quaternion(float qx1, float qy1, float qz1, float qw1, float &qx2, float &qy2, float &qz2, float &qw2);
  void hamilton_product(float qx1, float qy1, float qz1, float qw1, float qx2, float qy2, float qz2, float qw2, float &qx3, float &qy3, float &qz3, float &qw3);
  /* calculate the pattern 3D position from its ellipse characteristic equation, see 4.3 of [1] */
  SEllipseCenters calcEigen(const float *data);

  bool calibrated_ = false;           // whether the user defined coordinate system is avaiable
  float grid_dim_x_;          // x unit dimention of 2D coordinate
  float grid_dim_y_;          // y unit dimention of 2D coordinate
  float circle_diameter_;     // outer circle diameter [m]
  
  float hom_[9];                  // transformation description for 2D
  S3DTransform D3transform_[4];   // transformation description for 3D
  ETransformType transform_type_; // current transformation

  cv::Mat intrinsic_mat_;         // camera intrinsic matrix
  cv::Mat distortion_coeffs_;     // camera distortion parameters
};

} // namespace whycode

#endif  // WHYCODE__CTRANSFORMATION_H
