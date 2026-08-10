#ifndef WHYCODE__CWHYCON_H
#define WHYCODE__CWHYCON_H

#include <vector>
#include <deque>
#include <map>
#include <array>
#include <stdlib.h>
#include <string>
#include <cmath>
#include <memory>

#include <opencv2/opencv.hpp>

// WhyCode libs
#include "whycode/CCircleDetect.h"
#include "whycode/CTransformation.h"
#include "whycode/CNecklace.h"
#include "whycode/CRawImage.h"

namespace whycode {

class CWhycode {
public:
  // marker detection variables
  int num_found_ = 0;         // num of markers detected in the last step
  int num_static_ = 0;        // num of non-moving markers

  // marker identification
  int id_bits_;       // num of ID bits
  int id_samples_;    // num of samples to identify ID
  int hamming_dist_;  // hamming distance of ID code

  ~CWhycode();
  
  void init(float circle_diam, int id_b, int id_s, int ham_dist);

  void setDrawing(bool draw_coords, bool draw_segments);

  void autocalibration();

  void manualcalibration();

  void selectMarker(float x, float y);

  void updateCameraInfo(const std::array<double, 9> &intrinsic_mat, const std::vector<double> &distortion_coeffs);

  std::vector<SMarker> processImage(CRawImage &image);

  void set_parameters(Parameters &p);

  Parameters get_parameters();

  bool is_calibrated();

  void setCalibrationConfig(const CalibrationConfig &config);

  CalibrationConfig getCalibrationConfig();

  void saveCalibration(const std::string &str);

  void loadCalibration(const std::string &str);

private:
  Parameters params_{};

  // GUI-related stuff
  int eval_time_;         // time required to detect the patterns

  CTransformation *trans_;    // transformation instance
  CNecklace *decoder_;        // instance to decode marker's ID

  std::vector<SMarker> current_marker_array_;      // array of currently detected markers
  std::vector<SMarker> last_marker_array_;         // array of previously detected markers
  std::vector<std::unique_ptr<CCircleDetect>> detector_array_;     // array of detector instances for each marker
  // std::vector<CCircleDetect*> detector_array_;

  bool calibrated_coords_ = false;


  std::array<std::map<int, int>, 4> indices_autocalib;
  int index_autocalib[4];

  // variables related to (auto) calibration
  const int calibration_steps_ = 20;              // how many measurements to average to estimate calibration pattern position (manual calib)
  const int auto_calibration_steps_ = 30;         // how many measurements to average to estimate calibration pattern position (automatic calib)  
  const int auto_calibration_pre_steps_ = 10;     // how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
  int calib_num_ = 5;                             // number of objects acquired for calibration (5 means calibration winished inactive)
  STrackedObject calib_[4];                       // array to store calibration patterns positions
  std::vector<STrackedObject> calib_tmp_;         // array to store several measurements of a given calibration pattern
  int calib_step_ = calibration_steps_ + 2;       // actual calibration step (num of measurements of the actual pattern)
  
  bool autocalibrate_ = false;                    // is the autocalibration in progress ?
  bool mancalibrate_ = false;

  ETransformType last_transform_type_ = TRANSFORM_2D;     // pre-calibration transform (used to preserve pre-calibation transform type)
  int was_markers_ = 1;                                   // pre-calibration number of makrers to track (used to preserve pre-calibation number of markers to track)

  /*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
  void manualCalib();

  /*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
  void autoCalib();
};

}  // namespace whycode

#endif  // WHYCODE__CWHYCON_H
