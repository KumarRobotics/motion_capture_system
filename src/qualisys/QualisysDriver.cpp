/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <motion_capture_system/qualisys/QualisysDriver.h>

using namespace std;
using namespace Eigen;

namespace mocap{

double QualisysDriver::deg2rad = M_PI / 180.0;

bool QualisysDriver::init() {
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  nh.param("server_address", server_address, string("192.168.254.1"));
  nh.param("server_base_port", base_port, 22222);
  nh.param("model", model, string(""));
  nh.param("frame_rate", frame_rate, 100);
  nh.param("max_accel", max_accel, 20.0);
  nh.param("publish_tf", publish_tf, false);
  nh.param("fixed_frame_id", fixed_frame_id, string("mocap"));

  frame_interval = 1.0 / static_cast<double>(frame_rate);
  double& dt = frame_interval;
  process_noise.topLeftCorner<6, 6>() =
    0.5*Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
  process_noise.bottomRightCorner<6, 6>() =
    Matrix<double, 6, 6>::Identity()*dt*5*max_accel;
  measurement_noise =
    Matrix<double, 6, 6>::Identity()*1e-5;

  // Connecting to the server
  ROS_INFO_STREAM("Connecting to the Qualisys at: "
      << server_address << ":" << base_port);

  if(!port_protocol.Connect((char *)server_address.data(), base_port, 0, 1, 7)) {
    ROS_FATAL_STREAM("Could not find the Qualisys at: "
        << server_address << ":" << base_port);
    return false;
  }
  ROS_INFO_STREAM("Connected to " << server_address << ":" << base_port);

  // Get 6DOF settings
  port_protocol.Read6DOFSettings();

  return true;
}

void QualisysDriver::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}

void QualisysDriver::run() {

  prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

  if(port_protocol.ReceiveRTPacket(e_type, true)) {

    switch(e_type) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "Error when streaming frames: "
            << port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handleFrame();
        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");
        break;
    }
  }

  return;
}

void QualisysDriver::handleFrame() {

  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFEulerBodyCount();

  if (model.empty()) {
    for (int i = 0; i< body_count; ++i)
        handleSubject(i);
  } else {
    // TODO: Convert the following loop to multi-threading
    for (int i = 0; i< body_count; ++i) {
      string subject_name(port_protocol.Get6DOFBodyName(i));
      if (subject_name == model)
        handleSubject(i);
    }
  }

  return;
}

void QualisysDriver::handleSubject(const int& sub_idx) {

  // Name of the subject
  string subject_name(port_protocol.Get6DOFBodyName(sub_idx));

  // Pose of the subject
  float x, y, z, roll, pitch, yaw;
  prt_packet->Get6DOFEulerBody(
      sub_idx, x, y, z, roll, pitch, yaw);

  // If the subject is lost
  if(isnan(x) || isnan(y) || isnan(z) ||
     isnan(roll) || isnan(pitch) || isnan(yaw)) {
    ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " <<
        subject_name << " not detected");
    if (subjects.find(subject_name) != subjects.end())
      subjects[subject_name]->disable();
    return;
  }

  // Qualisys sometimes flips 180 degrees around the x axis
  if(roll > 90)
    roll -= 180;
  else if(roll < -90)
    roll += 180;

  // Convert the msgs to Eigen type
  Eigen::Quaterniond m_att;
  tf::quaternionTFToEigen(
      tf::createQuaternionFromRPY(roll*deg2rad, pitch*deg2rad, yaw*deg2rad), m_att);
  Eigen::Vector3d m_pos(x, y, z);

  // Create a object if it has not been observed before
  if (subjects.find(subject_name) == subjects.end()) {
    subjects[subject_name] = Subject::SubjectPtr(
        new Subject(&nh, subject_name, fixed_frame_id));
    subjects[subject_name]->setParameters(
        process_noise, measurement_noise, frame_rate);
  } else {
    if (!subjects[subject_name]->isActive()) {
      ROS_WARN("Rigid body %s is re-detected", subject_name.c_str());
      subjects[subject_name]->enable();
    }
  }

  // Feed the new measurement to the subject
  double time = ros::Time::now().toSec();
  subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);

  // Publish tf if requred
  if (publish_tf) {
    tf::Quaternion att_tf;
    tf::Vector3 pos_tf;

    tf::quaternionEigenToTF(m_att, att_tf);
    tf::vectorEigenToTF(m_pos, pos_tf);

    tf::StampedTransform stamped_transform =
      tf::StampedTransform(tf::Transform(att_tf, pos_tf),
        ros::Time::now(), fixed_frame_id, subject_name);
    tf_publisher.sendTransform(stamped_transform);
  }

  return;
}
}

