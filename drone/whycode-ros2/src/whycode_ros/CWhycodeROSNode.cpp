#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <limits>

#include <rclcpp/parameter_value.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include "whycode_ros/CWhycodeROSNode.h"

using std::placeholders::_1;
using std::placeholders::_2;

using IntegerRange = rcl_interfaces::msg::IntegerRange;
using FloatingPointRange = rcl_interfaces::msg::FloatingPointRange;
using ParameterDescriptor = rcl_interfaces::msg::ParameterDescriptor;

namespace whycode_ros2 {

void CWhycodeROSNode::setCalibMethodCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Request> req,
                                                  std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Response> res) {
  RCLCPP_INFO(this->get_logger(), "setCalibMethodCallback: %d", req->method);
  try {
    if (req->method == whycode_interfaces::srv::SetCalibMethod::Request::AUTO) {
      whycode_->autocalibration();
      res->success = true;
      res->msg = "OK";
    } else if (req->method == whycode_interfaces::srv::SetCalibMethod::Request::MANUAL) {
      whycode_->manualcalibration();
      res->success = true;
      res->msg = "OK";
    } else {
      res->success = false;
      res->msg = "ERROR in setting calibration method : unkown method '" + std::to_string(req->method) + "'";
    }
  } catch (const std::exception& e) {
    res->success = false;
    res->msg = e.what();
  }
}

void CWhycodeROSNode::setCalibPathCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Request> req,
                                                std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Response> res) {
  RCLCPP_INFO(this->get_logger(), "setCalibPathCallback: action %d path %s", req->action, req->path.c_str());
  try {
    if (req->action == whycode_interfaces::srv::SetCalibPath::Request::LOAD) {
      whycode_->loadCalibration(req->path);
      res->success = true;
      res->msg = "OK";
    } else if (req->action == whycode_interfaces::srv::SetCalibPath::Request::SAVE) {
      whycode_->saveCalibration(req->path);
      res->success = true;
      res->msg = "OK";
    } else {
      res->success = false;
      res->msg = "ERROR in setting calibration path : unkown action '" + std::to_string(req->action) + "'";
    }
  } catch (const std::exception& e) {
    res->success = false;
    res->msg = e.what();
  }
}

void CWhycodeROSNode::selectMarkerCallback(const std::shared_ptr<whycode_interfaces::srv::SelectMarker::Request> req,
                                                std::shared_ptr<whycode_interfaces::srv::SelectMarker::Response> res) {
  RCLCPP_INFO(this->get_logger(), "selectMarkerCallback: x %f y %f", req->point.x, req->point.y);
  whycode_->selectMarker(req->point.x, req->point.y);
}

void CWhycodeROSNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  if (msg->k[0] == 0) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "ERROR: Camera is not calibrated!");
    return;
  }
  whycode_->updateCameraInfo(msg->k, msg->d);
}

void CWhycodeROSNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  // image_.updateImage((unsigned char*)&msg->data[0], msg->height, msg->width, msg->step / msg->width);

  cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvShare(msg, "rgb8");

  image_.updateImage(cv_img->image.data, cv_img->image.cols, cv_img->image.rows, 3);

  std::vector<whycode::SMarker> whycode_detections = whycode_->processImage(image_);

  whycode_interfaces::msg::MarkerArray marker_array;
  marker_array.header = msg->header;
  for (const whycode::SMarker &detection : whycode_detections) {
    whycode_interfaces::msg::Marker marker;
    marker.id = detection.seg.ID;
    marker.size = detection.seg.size;
    marker.u = detection.seg.x;
    marker.v = detection.seg.y;
    marker.angle = detection.obj.angle;
    marker.position.position.x = detection.obj.x;
    marker.position.position.y = detection.obj.y;
    marker.position.position.z = detection.obj.z;
    marker.position.orientation.x = detection.obj.qx;
    marker.position.orientation.y = detection.obj.qy;
    marker.position.orientation.z = detection.obj.qz;
    marker.position.orientation.w = detection.obj.qw;
    marker.rotation.x = detection.obj.roll;
    marker.rotation.y = detection.obj.pitch;
    marker.rotation.z = detection.obj.yaw;
    marker.maxx = detection.seg.maxx;
    marker.maxy = detection.seg.maxy;
    marker.minx = detection.seg.minx;
    marker.miny = detection.seg.miny;
    marker.outer_size = detection.outer_size;
    marker.inner_size = detection.inner_size;
    marker_array.markers.push_back(marker);
  }
  markers_pub_->publish(marker_array);

  if (node_params_.use_gui && img_pub_.getNumSubscribers() > 0) {
    sensor_msgs::msg::Image out_msg;
    out_msg.header = msg->header;
    out_msg.height = msg->height;
    out_msg.width = msg->width;
    out_msg.encoding = msg->encoding;
    out_msg.step = msg->step;
    out_msg.data.resize(msg->step * msg->height);
    std::memcpy((void*)&out_msg.data[0], image_.data_, msg->step * msg->height);
    img_pub_.publish(out_msg);
  }
}

CWhycodeROSNode::CWhycodeROSNode()
: Node("whycode_node"), whycode_(std::make_unique<whycode::CWhycode>())
{
  process_node_parameters();
  auto why_params = process_lib_parameters();

  whycode_->init(why_params.circle_diameter, why_params.id_bits, why_params.id_samples, why_params.hamming_dist);
  if (node_params_.calib_path.size() > 0) {
    try {
      whycode_->loadCalibration(node_params_.calib_path);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Calibration file '%s' could not be loaded. Using camera centric coordinates.", node_params_.calib_path.c_str());
      why_params.coords_method = 0;
    }
  }
  whycode_->set_parameters(why_params);

  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CWhycodeROSNode::validate_upcoming_parameters_callback, this, _1));

  img_pub_ = image_transport::create_publisher(this, "~/debug_image");
  markers_pub_ = this->create_publisher<whycode_interfaces::msg::MarkerArray>("~/markers", 1);
  discovery_pub_ = this->create_publisher<whycode_interfaces::msg::Discovery>("~/discovery", 1);
  img_sub_ = image_transport::create_subscription(this, node_params_.img_base_topic, std::bind(&CWhycodeROSNode::imageCallback, this, _1),
                                                  node_params_.img_transport);  // image_transport::TransportHints for optionss
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(node_params_.info_topic, 1, std::bind(&CWhycodeROSNode::cameraInfoCallback, this, _1));

  calib_method_srv_ = this->create_service<whycode_interfaces::srv::SetCalibMethod>("~/set_calib_method", std::bind(&CWhycodeROSNode::setCalibMethodCallback, this, _1, _2));
  calib_path_srv_ = this->create_service<whycode_interfaces::srv::SetCalibPath>("~/set_calib_path", std::bind(&CWhycodeROSNode::setCalibPathCallback, this, _1, _2));
  select_marker_srv_ = this->create_service<whycode_interfaces::srv::SelectMarker>("~/select_marker", std::bind(&CWhycodeROSNode::selectMarkerCallback, this, _1, _2));
}

void CWhycodeROSNode::process_node_parameters() {
  node_params_.calib_path = this->declare_parameter("calib_file", std::string{},
    ParameterDescriptor().set__description("Coordinate calibration file").set__read_only(true));
  node_params_.img_transport = this->declare_parameter("img_transport", std::string("compressed"),
    ParameterDescriptor().set__description("Subscribed image topic transport type").set__read_only(true));
  node_params_.img_base_topic = this->declare_parameter("img_base_topic", std::string{},
    ParameterDescriptor().set__description("Subscribed image topic base name").set__read_only(true));
  node_params_.info_topic = this->declare_parameter("info_topic", std::string{},
    ParameterDescriptor().set__description("Camera info topic associated with the image topic").set__read_only(true));

  try {
    rclcpp::expand_topic_or_service_name(node_params_.img_base_topic, this->get_name(), this->get_namespace());
  } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'img_base_topic' is not set or contain invalid value. Exiting!");
    rclcpp::shutdown();
    throw e;
  }
  try {
    rclcpp::expand_topic_or_service_name(node_params_.info_topic, this->get_name(), this->get_namespace());
  } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'info_topic' is not set or contain invalid value. Exiting!");
    rclcpp::shutdown();
    throw e;
  }
}

whycode::Parameters CWhycodeROSNode::process_lib_parameters() {
  auto why_params = whycode_->get_parameters();

  // static parameters
  why_params.id_bits = this->declare_parameter("id_bits", why_params.id_bits,
    ParameterDescriptor().set__description("Number of encoded bits")
    .set__read_only(true).set__integer_range({IntegerRange().set__from_value(1).set__to_value(100)}));
  why_params.id_samples = this->declare_parameter("id_samples", why_params.id_samples,
    ParameterDescriptor().set__description("Number of samples along the perimeter")
    .set__read_only(true).set__integer_range({IntegerRange().set__from_value(1).set__to_value(3600)}));
  why_params.hamming_dist = this->declare_parameter("hamming_dist", why_params.hamming_dist,
    ParameterDescriptor().set__description("Encoded ID hamming distance")
    .set__read_only(true).set__integer_range({IntegerRange().set__from_value(1).set__to_value(10)}));

  // dynamic parameters
  why_params.draw_coords = this->declare_parameter("draw_coords", why_params.draw_coords,
    ParameterDescriptor().set__description("Enable drawing coordinates in the debug image"));
  why_params.draw_segments = this->declare_parameter("draw_segments", why_params.draw_segments,
    ParameterDescriptor().set__description("Enable highlighting found markers in the debug image"));
  why_params.use_gui = this->declare_parameter("use_gui", why_params.use_gui,
    ParameterDescriptor().set__description("Enable any drawings in the debug image"));
  node_params_.use_gui = why_params.use_gui;
  why_params.identify = this->declare_parameter("identify", why_params.identify,
    ParameterDescriptor().set__description("Enable WhyCode detection instead of ID-less WhyCon"));

  why_params.coords_method = this->declare_parameter("coords_method", why_params.coords_method,
    ParameterDescriptor().set__description("Coordinates transformation")
    .set__additional_constraints("[0] camera-centric [1] 2D homography [2] 3D linear combination of four transforms [3] 3D full 4x3 matrix")
    .set__integer_range({IntegerRange().set__from_value(0).set__to_value(3)}));
  why_params.num_markers = this->declare_parameter("num_markers", why_params.num_markers,
    ParameterDescriptor().set__description("Number of markers to detect")
    .set__integer_range({IntegerRange().set__from_value(0).set__to_value(10000)}));
  why_params.min_size = this->declare_parameter("min_size", why_params.min_size,
    ParameterDescriptor().set__description("Min total marker size [px] (outer + inner), -1 to disable")
    .set__integer_range({IntegerRange().set__from_value(-1).set__to_value(std::numeric_limits<std::int64_t>::max())}));
  why_params.max_size = this->declare_parameter("max_size", why_params.max_size,
    ParameterDescriptor().set__description("Max total marker size [px] (outer + inner), -1 to disable")
    .set__integer_range({IntegerRange().set__from_value(-1).set__to_value(std::numeric_limits<std::int64_t>::max())}));
  why_params.min_size_outer = this->declare_parameter("min_size_outer", why_params.min_size_outer,
    ParameterDescriptor().set__description("Min outer segment size [px], -1 to disable")
    .set__integer_range({IntegerRange().set__from_value(-1).set__to_value(std::numeric_limits<std::int64_t>::max())}));
  why_params.min_size_inner = this->declare_parameter("min_size_inner", why_params.min_size_inner,
    ParameterDescriptor().set__description("Min inner segment size [px], -1 to disable")
    .set__integer_range({IntegerRange().set__from_value(-1).set__to_value(std::numeric_limits<std::int64_t>::max())}));

  why_params.circle_diameter = this->declare_parameter("circle_diameter", why_params.circle_diameter,
    ParameterDescriptor().set__description("Marker's outer diameter [m]")
    .set__floating_point_range({FloatingPointRange().set__from_value(std::numeric_limits<double>::min()).set__to_value(std::numeric_limits<double>::max())}));
  why_params.calib_dist_x = this->declare_parameter("calib_dist_x", why_params.calib_dist_x,
    ParameterDescriptor().set__description("Distance of markers in x-axis in custom coordinate system")
    .set__floating_point_range({FloatingPointRange().set__from_value(std::numeric_limits<double>::min()).set__to_value(std::numeric_limits<double>::max())}));
  why_params.calib_dist_y = this->declare_parameter("calib_dist_y", why_params.calib_dist_y,
    ParameterDescriptor().set__description("Distance of markers in y-axis in custom coordinate system")
    .set__floating_point_range({FloatingPointRange().set__from_value(std::numeric_limits<double>::min()).set__to_value(std::numeric_limits<double>::max())}));
  why_params.initial_circularity_tolerance = this->declare_parameter("initial_circularity_tolerance", why_params.initial_circularity_tolerance,
    ParameterDescriptor().set__description("Initial circularity test tolerance [%]")
    .set__floating_point_range({FloatingPointRange().set__from_value(0).set__to_value(100)}));
  why_params.final_circularity_tolerance = this->declare_parameter("final_circularity_tolerance", why_params.final_circularity_tolerance,
    ParameterDescriptor().set__description("Final circularity test tolerance [%]")
    .set__floating_point_range({FloatingPointRange().set__from_value(0).set__to_value(100)}));
  why_params.area_ratio_tolerance = this->declare_parameter("area_ratio_tolerance", why_params.area_ratio_tolerance,
    ParameterDescriptor().set__description("Tolerance of black and white area ratios [%]")
    .set__floating_point_range({FloatingPointRange().set__from_value(0).set__to_value(200)}));
  why_params.center_distance_tolerance_ratio = this->declare_parameter("center_distance_tolerance_ratio", why_params.center_distance_tolerance_ratio,
    ParameterDescriptor().set__description("Concentricity test ratio [%]")
    .set__floating_point_range({FloatingPointRange().set__from_value(0).set__to_value(100)}));
  why_params.center_distance_tolerance_abs = this->declare_parameter("center_distance_tolerance_abs", why_params.center_distance_tolerance_abs,
    ParameterDescriptor().set__description("Concentricity test absolute [px]")
    .set__floating_point_range({FloatingPointRange().set__from_value(0).set__to_value(25)}));

  return why_params;
}

rcl_interfaces::msg::SetParametersResult CWhycodeROSNode::validate_upcoming_parameters_callback(const std::vector<rclcpp::Parameter> & parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "coords_method") {
      if (param.as_int() != 0 && !whycode_->is_calibrated()) {
        result.successful = false;
        result.reason = "The coordinates calibration is not available. Load or perform calibration.";
      }
    }
  }

  return result;
}

void CWhycodeROSNode::react_to_updated_parameters_callback(const std::vector<rclcpp::Parameter> & parameters) {
  auto why_params = whycode_->get_parameters();

  for (const auto & param : parameters) {
    RCLCPP_INFO(this->get_logger(), "received update parameter call '%s'", param.get_name().c_str());
    if (param.get_name() == "draw_coords") {
      why_params.draw_coords = param.as_bool();
    } else if (param.get_name() == "draw_segments") {
      why_params.draw_segments = param.as_bool();
    } else if (param.get_name() == "use_gui") {
      why_params.use_gui = param.as_bool();
      node_params_.use_gui = why_params.use_gui;
    } else if (param.get_name() == "identify") {
      why_params.identify = param.as_bool();
    } else if (param.get_name() == "coords_method") {
      why_params.coords_method = param.as_int();
    } else if (param.get_name() == "num_markers") {
      why_params.num_markers = param.as_int();
    } else if (param.get_name() == "min_size") {
      why_params.min_size = param.as_int();
    } else if (param.get_name() == "max_size") {
      why_params.max_size = param.as_int();
    } else if (param.get_name() == "min_size_outer") {
      why_params.min_size_outer = param.as_int();
    } else if (param.get_name() == "min_size_inner") {
      why_params.min_size_inner = param.as_int();
    } else if (param.get_name() == "circle_diameter") {
      why_params.circle_diameter = param.as_double();
    } else if (param.get_name() == "calib_dist_x") {
      why_params.calib_dist_x = param.as_double();
    } else if (param.get_name() == "calib_dist_y") {
      why_params.calib_dist_y = param.as_double();
    } else if (param.get_name() == "initial_circularity_tolerance") {
      why_params.initial_circularity_tolerance = param.as_double();
    } else if (param.get_name() == "final_circularity_tolerance") {
      why_params.final_circularity_tolerance = param.as_double();
    } else if (param.get_name() == "area_ratio_tolerance") {
      why_params.area_ratio_tolerance = param.as_double();
    } else if (param.get_name() == "center_distance_tolerance_ratio") {
      why_params.center_distance_tolerance_ratio = param.as_double();
    } else if (param.get_name() == "center_distance_tolerance_abs") {
      why_params.center_distance_tolerance_abs = param.as_double();
    }
  }

  whycode_->set_parameters(why_params);
}

}  // namespace whycode_ros2
