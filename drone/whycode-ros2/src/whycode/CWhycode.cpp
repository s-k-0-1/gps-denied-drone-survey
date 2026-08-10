#include <stdexcept>
#include <chrono>
#include <stdio.h>

#include "whycode/CWhycode.h"

namespace whycode {

void CWhycode::autocalibration() {
  if (num_found_ < 4) {
    throw std::runtime_error("Autocalibration not possible. Cannot locate 4 markers in the scene.");
  } else {
    calib_step_ = 0;
    was_markers_ = params_.num_markers;

    last_transform_type_ = trans_->getTransformType();
    trans_->setTransformType(TRANSFORM_NONE);
    mancalibrate_ = false;
    autocalibrate_ = true;
  }
}

void CWhycode::manualcalibration() {
  if (num_found_ < 4) {
    throw std::runtime_error("Manual calibration not possible. Cannot locate 4 markers in the scene.");
  } else {
    calib_num_ = 0;
    was_markers_ = params_.num_markers;
    params_.num_markers = 1;

    last_transform_type_ = trans_->getTransformType();
    trans_->setTransformType(TRANSFORM_NONE);
    autocalibrate_ = false;
    mancalibrate_ = true;
  }
}

void CWhycode::selectMarker(float x, float y) {
  // if (mancalibrate_) {
    if (calib_num_ < 4 && calib_step_ > calibration_steps_) {
      calib_step_ = 0;
      trans_->setTransformType(TRANSFORM_NONE);
    }
    if (params_.num_markers > 0) {
      current_marker_array_[0].seg.x = x; 
      current_marker_array_[0].seg.y = y;
      current_marker_array_[0].seg.valid = true;
      detector_array_[0]->localSearch = true;
    }
  // } else {
  // }
}

/*manual calibration can be initiated by pressing 'r'
  and then clicking circles at four positions (0,0)(params_.calib_dist_x,0)...*/
void CWhycode::manualCalib() {
  if (current_marker_array_[0].valid) {
    STrackedObject o = current_marker_array_[0].obj;
    //moveOne = moveVal;

    //object found - add to a buffer
    if (calib_step_ < calibration_steps_) { calib_tmp_[calib_step_++] = o; }

    //does the buffer contain enough data to calculate the object position
    if (calib_step_ == calibration_steps_) {
      o.x = o.y = o.z = 0;

      for (int k = 0; k < calibration_steps_; ++k) {
        o.x += calib_tmp_[k].x;
        o.y += calib_tmp_[k].y;
        o.z += calib_tmp_[k].z;
      }
      o.x = o.x / calibration_steps_;
      o.y = o.y / calibration_steps_;
      o.z = o.z / calibration_steps_;

      if (calib_num_ < 4) { calib_[calib_num_++] = o; }

      //was it the last object needed to establish the transform ?
      if (calib_num_ == 4) {
        //calculate and save transforms
        trans_->calibrate2D(calib_, params_.calib_dist_x, params_.calib_dist_y);
        trans_->calibrate3D(calib_, params_.calib_dist_x, params_.calib_dist_y);
        calib_num_++;
        params_.num_markers = was_markers_;
        trans_->setTransformType(last_transform_type_);
        detector_array_[0]->localSearch = false;
        mancalibrate_ = false;
        printf("manualCalib done\n");
      }
      calib_step_++;
    }
  }
}

/*finds four outermost circles and uses them to set-up the coordinate system
  [0,0] is left-top, [0,params_.calib_dist_x] next in clockwise direction*/
/*void CWhycode::autoCalib() {
  int ok_last_tracks = 0;
  for (int i = 0; i < params_.num_markers; ++i) {
    if (detector_array_[i]->lastTrackOK) { ++ok_last_tracks; }
  }
  if (ok_last_tracks < 4) { return; }

  int index[] = {0, 0, 0, 0};
  int max_eval = 0;
  int eval = 0;
  int sX[] = {-1, +1, -1, +1};
  int sY[] = {+1, +1, -1, -1};
  for (int b = 0; b < 4; ++b) {
    max_eval = -10000000;
    for (int i = 0; i < params_.num_markers; ++i) {
      eval =  sX[b] * current_marker_array_[i].seg.x + sY[b] * current_marker_array_[i].seg.y;
      if (eval > max_eval) {
        max_eval = eval;
        index[b] = i;
      }
    }
  }
  printf("INDEX: %i %i %i %i\n", index[0], index[1], index[2], index[3]);

  for (int i = 0; i < 4; ++i) {
    if (calib_step_ <= auto_calibration_pre_steps_) {
      calib_[i].x = calib_[i].y = calib_[i].z = 0;
    }
    calib_[i].x += current_marker_array_[index[i]].obj.x;
    calib_[i].y += current_marker_array_[index[i]].obj.y;
    calib_[i].z += current_marker_array_[index[i]].obj.z;
  }
  calib_step_++;
  if (calib_step_ == auto_calibration_steps_) {
    for (int i = 0; i < 4; ++i) {
      calib_[i].x = calib_[i].x / (auto_calibration_steps_ - auto_calibration_pre_steps_);
      calib_[i].y = calib_[i].y / (auto_calibration_steps_ - auto_calibration_pre_steps_);
      calib_[i].z = calib_[i].z / (auto_calibration_steps_ - auto_calibration_pre_steps_);
    }
    trans_->calibrate2D(calib_, params_.calib_dist_x, params_.calib_dist_y);
    trans_->calibrate3D(calib_, params_.calib_dist_x, params_.calib_dist_y);
    calib_num_++;
    params_.num_markers = was_markers_;
    trans_->setTransformType(last_transform_type_);
    autocalibrate_ = false;
    printf("autoCalib done\n");
  }
}*/

void CWhycode::autoCalib() {
  printf("autocalib start point. step %d\n", calib_step_);
  int ok_last_tracks = 0;
  for (int i = 0; i < params_.num_markers; ++i) {
    if (detector_array_[i]->lastTrackOK) { ++ok_last_tracks; }
  }
  if (ok_last_tracks < 4) { return; }

  if (calib_step_ < auto_calibration_pre_steps_) {
    int max_eval = 0;
    int eval = 0;
    int sX[] = {-1, +1, -1, +1};
    int sY[] = {+1, +1, -1, -1};
    for (int b = 0; b < 4; ++b) {
      max_eval = -10000000;
      int best_index = 0;
      for (int i = 0; i < params_.num_markers; ++i) {
        if (current_marker_array_[i].valid) {
          eval =  sX[b] * current_marker_array_[i].seg.x + sY[b] * current_marker_array_[i].seg.y;
          if (eval > max_eval) {
            max_eval = eval;
            best_index = i;
            // fprintf(stderr, "indices tested b %d i %d eval %d max_eval %d\n", b, i, eval, max_eval);
          }
        }
      }
      ++indices_autocalib[b][best_index];
    }
  } else if (calib_step_ == auto_calibration_pre_steps_) {
    // printf("index extraction\n");
    for (int i = 0; i < 4; ++i) {
      // for(auto p : indices_autocalib[i])
      //     printf("%d ---- %d %d\n", i, p.first, p.second);
      index_autocalib[i] = (*std::max_element(indices_autocalib[i].begin(), indices_autocalib[i].end(),
        [] (const std::pair<int, int>& a, const std::pair<int, int>& b) { return a.second < b.second; } )).first;

      calib_[i].x = calib_[i].y = calib_[i].z = 0;
    }
    printf("INDEX: %i %i %i %i\n", index_autocalib[0], index_autocalib[1], index_autocalib[2], index_autocalib[3]);
  } else if (calib_step_ > auto_calibration_pre_steps_) {
    for (int i = 0; i < 4; ++i) {
      calib_[i].x += current_marker_array_[index_autocalib[i]].obj.x;
      calib_[i].y += current_marker_array_[index_autocalib[i]].obj.y;
      calib_[i].z += current_marker_array_[index_autocalib[i]].obj.z;
    }
    if (calib_step_ == auto_calibration_steps_) {
      for (int i = 0; i < 4; ++i) {
        calib_[i].x = calib_[i].x / (auto_calibration_steps_ - auto_calibration_pre_steps_);
        calib_[i].y = calib_[i].y / (auto_calibration_steps_ - auto_calibration_pre_steps_);
        calib_[i].z = calib_[i].z / (auto_calibration_steps_ - auto_calibration_pre_steps_);
        // printf("i %d -- %f %f %f\n", i, calib_[i].x, calib_[i].y, calib_[i].z);
      }
      trans_->calibrate2D(calib_, params_.calib_dist_x, params_.calib_dist_y);
      // trans_->calibrate2D(calib_, homo_square_pts_);
      trans_->calibrate3D(calib_, params_.calib_dist_x, params_.calib_dist_y);
      calib_num_++;
      // params_.num_markers = was_markers_;
      trans_->setTransformType(last_transform_type_);
      autocalibrate_ = false;
      printf("autoCalib done\n");
    }
  }
  ++calib_step_;
}

bool CWhycode::is_calibrated() {
  return trans_->is_calibrated();
};

std::vector<SMarker> CWhycode::processImage(CRawImage &image) {
  std::vector<SMarker> whycode_detections;
  num_found_ = num_static_ = 0;
  auto start = std::chrono::steady_clock::now();


  // track the markers found in the last attempt
  for (int i = 0; i < params_.num_markers; ++i) {
    if (current_marker_array_[i].valid) {
      last_marker_array_[i] = current_marker_array_[i];
      current_marker_array_[i] = detector_array_[i]->findSegment(image, last_marker_array_[i].seg);
    }
  }

  // search for untracked (not detected in the last frame) markers
  for (int i = 0; i < params_.num_markers; ++i) {
    if (current_marker_array_[i].valid == false) {
      last_marker_array_[i].valid = false;
      last_marker_array_[i].seg.valid = false;
      current_marker_array_[i] = detector_array_[i]->findSegment(image, last_marker_array_[i].seg);
    }
    if (current_marker_array_[i].seg.valid == false) { break; }  //does not make sense to search for more patterns if the last one was not found
  }

  for (int i = 0; i < params_.num_markers; ++i) {
    if (current_marker_array_[i].valid) {
      if (params_.identify && current_marker_array_[i].seg.ID <= -1) {
        current_marker_array_[i].seg.angle = last_marker_array_[i].seg.angle;
        current_marker_array_[i].seg.ID = last_marker_array_[i].seg.ID;
      }
      num_found_++;
      if (current_marker_array_[i].seg.x == last_marker_array_[i].seg.x) { num_static_++; }
    }
  }

  eval_time_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count();

  for (int i = 0; i < params_.num_markers; ++i) {
    if (current_marker_array_[i].valid) { whycode_detections.push_back(current_marker_array_[i]); }
  }

  // draw stuff on the GUI
  if (params_.use_gui) {
    image.drawTimeStats(eval_time_, num_found_);

    if (mancalibrate_) {
      image.drawGuideCalibration(calib_num_, params_.calib_dist_x, params_.calib_dist_y);
    }

    for (int i = 0; i < params_.num_markers && params_.draw_coords; ++i) {
      if (current_marker_array_[i].valid) {
        image.drawStats(current_marker_array_[i], trans_->getTransformType() == TRANSFORM_2D);
      }
    }
  }

  // establishing the coordinate system by manual or autocalibration
  if (autocalibrate_ && num_found_ > 3) { //num_found_ == params_.num_markers)
    autoCalib();
  }
  if (calib_num_ < 4) { manualCalib(); }

  return whycode_detections;
}

void CWhycode::setCalibrationConfig(const CalibrationConfig &config) {
  trans_->setCalibrationConfig(config);
}

CalibrationConfig CWhycode::getCalibrationConfig() {
  return trans_->getCalibrationConfig();
}

// cleaning up
CWhycode::~CWhycode() {
  delete trans_;
  delete decoder_;
}

void CWhycode::updateCameraInfo(const std::array<double, 9> &intrinsic_mat, const std::vector<double> &distortion_coeffs) {
  trans_->updateCameraParams(intrinsic_mat, distortion_coeffs);
}

void CWhycode::init(float circle_diam, int id_b, int id_s, int ham_dist) {
  id_bits_ = id_b;
  id_samples_ = id_s;
  hamming_dist_ = ham_dist;

  trans_ = new CTransformation(circle_diam);
  decoder_ = new CNecklace(id_bits_, id_samples_, hamming_dist_);
  calib_tmp_.resize(calibration_steps_);
}

void CWhycode::set_parameters(Parameters &p) {
  if (p == params_) { return; }

  params_ = p;

  trans_->setCircleDiameter(params_.circle_diameter);
  trans_->setTransformType(params_.coords_method);

  if (detector_array_.size() != params_.num_markers) {
    current_marker_array_.resize(params_.num_markers);
    last_marker_array_.resize(params_.num_markers);

    if (detector_array_.size() < params_.num_markers) {
      int new_markers = params_.num_markers - detector_array_.size();
      for (int i = 0; i < new_markers; ++i) {
        detector_array_.emplace_back(std::make_unique<CCircleDetect>(params_.identify, id_bits_, id_samples_, params_.draw_segments, trans_, decoder_));
      }
    } else {
      detector_array_.resize(params_.num_markers);
    }
  }

  for (auto & detector : detector_array_) {
    detector->reconfigure(params_.initial_circularity_tolerance, params_.final_circularity_tolerance, params_.area_ratio_tolerance,
                          params_.center_distance_tolerance_ratio, params_.center_distance_tolerance_abs, params_.identify,
                          params_.min_size, params_.max_size, params_.min_size_outer, params_.min_size_inner);
    detector->setDraw(params_.draw_segments);
  }
}

Parameters CWhycode::get_parameters() {
  return params_;
}

void CWhycode::loadCalibration(const std::string &str) {
  whycode::CalibrationConfig config;

  cv::FileStorage fs(str, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Could not open/load calibration file. " + str);
  }

  fs["dim_x"] >> config.grid_dim_x_;
  fs["dim_y"] >> config.grid_dim_y_;

  cv::Mat hom_tmp(3, 3, CV_32FC1);
  fs["hom"] >> hom_tmp;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      config.hom_[3 * i + j] = hom_tmp.at<float>(i, j);
    }
  }

  for (int k = 0; k < 4; ++k) {
    cv::Mat offset_tmp(3, 1, CV_32FC1);
    fs["offset_" + std::to_string(k)] >> offset_tmp;
    for (int i = 0; i < 3; ++i) {
      config.D3transform_[k].orig[i] = offset_tmp.at<float>(i);
    }

    cv::Mat simlar_tmp(3, 3, CV_32FC1);
    fs["simlar_" + std::to_string(k)] >> simlar_tmp;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        config.D3transform_[k].simlar[3 * i + j] = simlar_tmp.at<float>(i, j);
      }
    }
  }

  fs.release();

  setCalibrationConfig(config);
}

void CWhycode::saveCalibration(const std::string &str) {
  whycode::CalibrationConfig config = getCalibrationConfig();

  cv::FileStorage fs(str, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    throw std::runtime_error("Could not open/create calibration file. " + str);
  }

  fs.writeComment("Dimensions");
  fs << "dim_x" << config.grid_dim_x_;
  fs << "dim_y" << config.grid_dim_y_;
  fs.writeComment("2D calibration");
  fs << "hom" << cv::Mat(3, 3, CV_32FC1, config.hom_);
  fs.writeComment("3D calibration");

  for (int k = 0; k < 4; ++k) {
    fs.writeComment("D3transform " + std::to_string(k));
    fs << "offset_" + std::to_string(k) << cv::Mat(3, 1, CV_32FC1, config.D3transform_[k].orig);
    fs << "simlar_" + std::to_string(k) << cv::Mat(3, 3, CV_32FC1, config.D3transform_[k].simlar);
  }

  fs.release();
}

}  // namespace whycode
