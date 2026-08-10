#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "whycode/CRawImage.h"

namespace whycode {

CRawImage::CRawImage(int width, int height, int bpp)
: width_(width), height_(height), bpp_(bpp), size_(width * height * bpp)
{
  data_ = (unsigned char *) std::malloc(size_ * sizeof(unsigned char));
}

void CRawImage::updateImage(unsigned char* new_data, int width, int height, int bpp) {
  if (width_ != width || height_ != height || bpp_ != bpp) {
    width_ = width;
    height_ = height;
    bpp_ = bpp;
    size_ = width * height * bpp;
    data_ = (unsigned char *) std::realloc(data_, size_ * sizeof(unsigned char));
    std::printf("Readjusting image format to %ix%i %ibpp.\n", width_, height_, bpp_);
  }
  std::memcpy(data_, new_data, size_);
}

void CRawImage::drawTimeStats(int eval_time, int num_markers) {
  cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
  char text[100];
  std::sprintf(text, "Found %i markers in %.3f ms", num_markers, eval_time / 1000.0);

  int font_face = cv::FONT_HERSHEY_SIMPLEX;//DUPLEX;
  double font_scale = 0.5;
  int thickness = 1;
  int baseline = 0;

  cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

  cv::rectangle(img, cv::Point(0, text_size.height + 3), cv::Point(text_size.width, 0), cv::Scalar(0, 0, 0), cv::FILLED);

  cv::putText(img, text, cv::Point(0, text_size.height + 1), font_face, font_scale, cv::Scalar(255, 0, 0), thickness, cv::LINE_AA);
}

void CRawImage::drawStats(SMarker &marker, bool trans_2D) {
  cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
  char text[100];

  int font_face = cv::FONT_HERSHEY_SIMPLEX;//DUPLEX;
  double font_scale = 0.5;
  int thickness = 1;
  int baseline = 0;
  cv::Scalar color(255, 0, 0);

  if (trans_2D) {
    std::sprintf(text, "%03.0f %03.0f", 1000 * marker.obj.x, 1000 * marker.obj.y);
  } else {
    std::sprintf(text, "%.3f %.3f %.3f", marker.obj.x, marker.obj.y, marker.obj.z);
  }

  cv::Size text_size0 = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
  cv::Point text_pos0(marker.seg.minx - 30, marker.seg.maxy + text_size0.height);
  cv::putText(img, text, text_pos0, font_face, font_scale, color, thickness, cv::LINE_AA);

  if (trans_2D) {
      // std::sprintf(text, "%02i %03i", marker.seg.ID, (int)(marker.obj.yaw / M_PI * 180));
  } else {
    std::sprintf(text, "%02i %.3f %.3f %.3f", marker.seg.ID, marker.obj.roll, marker.obj.pitch, marker.obj.yaw);
    cv::Size text_size1 = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
    cv::Point text_pos1(marker.seg.minx - 30, marker.seg.maxy + 2 * text_size1.height + 5);
    cv::putText(img, text, text_pos1, font_face, font_scale, color, thickness, cv::LINE_AA);
  }
}

void CRawImage::drawGuideCalibration(int calib_num, float dim_x, float dim_y) {
  cv::Mat img(height_, width_, CV_8UC(bpp_), (void*)data_);
  char text[100];

  int font_face = cv::FONT_HERSHEY_SIMPLEX;
  double font_scale = 1;
  int thickness = 2;
  int baseline = 0;
  cv::Scalar text_color(0, 255, 0);
  cv::Scalar rect_color(0, 0, 0);

  switch (calib_num) {
    default:
    case 0: {
      std::sprintf(text, "Click at [0.000, 0.000].");
      break;
    }
    case 1: {
      std::sprintf(text, "Click at [%.3f, 0.000].", dim_x);
      break;
    }
    case 2: {
      std::sprintf(text, "Click at [0.000, %.3f].", dim_y);
      break;
    }
    case 3: {
      std::sprintf(text, "Click at [%.3f, %.3f].", dim_x, dim_y);
      break;
    }
  }

  cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
  cv::Point text_pos(width_ / 2 - 130, height_ / 2);
  cv::Point text_pos_opp(text_pos.x + text_size.width, text_pos.y - text_size.height);

  cv::rectangle(img, text_pos, text_pos_opp, rect_color, cv::FILLED);
  cv::putText(img, text, text_pos, font_face, font_scale, text_color, thickness);
}

}  // namespace whycode
