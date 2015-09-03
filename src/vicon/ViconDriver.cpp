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

#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <motion_capture_system/vicon/ViconDriver.h>

using namespace std;
using namespace Eigen;
namespace ViconSDK = ViconDataStreamSDK::CPP;

namespace mocap {

bool ViconDriver::init() {

  nh.param("server_address", server_address, string("alkaline2"));
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

  timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;

  // Connect to the server
  ROS_INFO("Connecting to Vicon Datastream server at %s", server_address.c_str());
  bool is_connected = false;
  for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
    client->Connect(server_address);
    if(client->IsConnected().Connected) {
      is_connected = true;
      break;
    }
    else
      nanosleep(&ts_sleep, NULL);
  }

  // Report if cannot connect
  if (!is_connected) {
    ROS_WARN("Cannot Connect to Vicon server at %s", server_address.c_str());
    return false;
  }

  // Configure the connection
  ROS_INFO("Successfully Connect to Vicon server at %s", server_address.c_str());
  client->SetStreamMode(ViconSDK::StreamMode::ClientPull);
  client->SetAxisMapping(ViconSDK::Direction::Forward,
      ViconSDK::Direction::Left, ViconSDK::Direction::Up);
  client->EnableSegmentData();
  if(!client->IsSegmentDataEnabled().Enabled) {
    ROS_WARN("Segment data cannot be enabled.");
    return false;
  }
  ROS_INFO("Successfully configure Vicon server at %s", server_address.c_str());

  // Need to wait for some time after enabling data else you get junk frames
  //struct timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;
  nanosleep(&ts_sleep, NULL);

  return true;
}

void ViconDriver::run() {
  ViconSDK::Result::Enum result = client->GetFrame().Result;
  if (result != ViconSDK::Result::Success)
    return;
  handleFrame();
  return;
}

void ViconDriver::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server at "
      << server_address);
  client->Disconnect();
  return;
}

void ViconDriver::handleFrame() {
  int body_count = client->GetSubjectCount().SubjectCount;
  if (model.empty()) {
    for (int i = 0; i< body_count; ++i)
        handleSubject(i);
  } else {
    // TODO: Convert the following loop to multi-threading
    for (int i = 0; i< body_count; ++i) {
      string subject_name = client->GetSubjectName(i).SubjectName;
      if (subject_name == model)
        handleSubject(i);
    }
  }
  return;
}

void ViconDriver::handleSubject(const int& sub_idx) {

  // We assume each subject has only one segment
  string subject_name = client->GetSubjectName(sub_idx).SubjectName;
  string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;
  // Get the pose for the subject
  ViconSDK::Output_GetSegmentGlobalTranslation trans =
      client->GetSegmentGlobalTranslation(subject_name, segment_name);
  ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
      client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

  if(trans.Result != ViconSDK::Result::Success ||
     quat.Result != ViconSDK::Result::Success ||
     trans.Occluded || quat.Occluded) {
    subjects[subject_name]->disable();
    ROS_WARN("Rigid body %s cannot be detected", subject_name.c_str());
    return;
  }

  // Convert the msgs to Eigen type
  Eigen::Quaterniond m_att(quat.Rotation[3],
      quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
  Eigen::Vector3d m_pos(trans.Translation[0]/1000,
      trans.Translation[1]/1000, trans.Translation[2]/1000);

  // Create a object if it has not been observed before
  if (subjects.find(subject_name) == subjects.end()) {
    subjects[subject_name] = Subject::SubjectPtr(
        new Subject(&nh, subject_name, fixed_frame_id));
    subjects[subject_name]->setParameters(
        process_noise, measurement_noise, frame_rate);
  } else {
    if (!subjects[subject_name]->isActive()) {
      subjects[subject_name]->enable();
    }
  }

  // Feed the new measurement to the subject
  double time = ros::Time::now().toSec();
  //printf("time: %f\n", time);
  subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);

  // For debug only
  //printf("time: %f\n", time);
  //cout << Vector4d(m_att.w(), m_att.x(), m_att.y(), m_att.z()).transpose() << endl;
  //cout << m_pos.transpose() << endl;
  //Quaterniond f_att = subjects[subject_name]->getAttitude();
  //Vector3d f_pos = subjects[subject_name]->getPosition();
  //Vector3d f_ang_vel = subjects[subject_name]->getAngularVel();
  //Vector3d f_lin_vel = subjects[subject_name]->getLinearVel();
  //cout << Vector4d(f_att.w(), f_att.x(), f_att.y(), f_att.z()).transpose() << endl;
  //cout << f_pos.transpose() << endl;
  //cout << f_ang_vel.transpose() << endl;
  //cout << f_lin_vel.transpose() << endl;

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
