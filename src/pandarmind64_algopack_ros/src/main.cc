#include <ros/ros.h>
#include "p64st_algopack.hh"
#include <QUdpSocket>
#include <iostream>
#include <array>
#include <string>
#include "pandarmind64_algopack_ros/Algopack.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace hesai;
using namespace pandarmind64_algopack_ros;

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "pandarmind64_algopack");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  ros::Publisher publisher = node.advertise<Algopack>("pandarmind64_algopack", 24);
  ros::Publisher markerarray_publisher = node.advertise<visualization_msgs::MarkerArray>("pandarmind64_algopack_markerarray", 24);
  {
      int port;
      nh.getParam("port", port);
      QUdpSocket udp_socket;
      if (!udp_socket.bind(port)) {
          std::cerr << "unable to bind port: " << port << std::endl;
          return 1;
      }
      while(udp_socket.isValid()) {
          if (udp_socket.hasPendingDatagrams()) {
              std::array<char, 1500> data;
              uint8_t* pkt = (uint8_t *)data.data();
              int len = udp_socket.pendingDatagramSize();
              udp_socket.readDatagram((char *)pkt, len);
              if (!STAlgopack::isValid(pkt, len)) {
                  std::cout << "Invalid packet with size: " << len << std::endl;
                  continue;
              }
              Algopack algopack;
              auto header = (STAlgopackV2_0::Header*)pkt;
              algopack.minor_version = header->minor_version;
              algopack.major_version = header->major_version;
              algopack.return_code = header->return_code;
              algopack.info_type = header->info_type;
              algopack.data_length = header->data_length;
              algopack.packet_total_count = header->packet_total_count;
              algopack.packet_count = header->packet_count;
              algopack.detection_center_x = header->detection_center_x;
              algopack.detection_center_y = header->detection_center_y;
              std::memcpy(&algopack.translation_matrix, &header->translation_matrix, sizeof(algopack.translation_matrix));
              switch(((STAlgopack::Header*)pkt)->major_version) {
                  case 2:
                    switch(((STAlgopack::Header*)pkt)->minor_version) {
                        case 0:
                          {
                            auto begin = pkt + sizeof(STAlgopackV2_0::Header) + sizeof(uint32_t);
                            for (
                                auto ptr = begin;
                                ptr - begin < ((const STAlgopackV2_0::Header*)pkt)->data_length - sizeof(STAlgopackV2_0::ObjectMeta);

                            ) {
                                auto object_meta = (const STAlgopackV2_0::ObjectMeta*)ptr;
                                Algoobjs algoobjs;
                                algoobjs.type = object_meta->type;
                                algoobjs.count = object_meta->count;
                                algoobjs.size = object_meta->size;
                                assert(object_meta->size * 4 == sizeof(STAlgopackV2_0::Object));
                                ptr += sizeof(STAlgopackV2_0::ObjectMeta);
                                for (auto i = 0; i < object_meta->count; i++) {
                                    auto object = (const STAlgopackV2_0::Object*)(ptr + i * 4 * object_meta->size);
                                    Algoobj algoobj;
                                    algoobj.id = object->id;
                                    algoobj.timestamp = object->timestamp;
                                    std::memcpy(&algoobj.pos_quat, &object->pos_quat, sizeof(algoobj.pos_quat));
                                    std::memcpy(&algoobj.lwh, &object->lwh, sizeof(algoobj.lwh));
                                    std::memcpy(&algoobj.center, &object->center, sizeof(algoobj.center));
                                    std::memcpy(&algoobj.relative_velocity, &object->relative_velocity, sizeof(algoobj.relative_velocity));
                                    std::memcpy(&algoobj.absolute_velocity, &object->absolute_velocity, sizeof(algoobj.absolute_velocity));
                                    std::memcpy(&algoobj.acceleration, &object->acceleration, sizeof(algoobj.acceleration));
                                    std::memcpy(&algoobj.location_cov, &object->location_cov, sizeof(algoobj.location_cov));
                                    std::memcpy(&algoobj.velocity_cov, &object->velocity_cov, sizeof(algoobj.velocity_cov));
                                    std::memcpy(&algoobj.acceleration_cov, &object->acceleration_cov, sizeof(algoobj.acceleration_cov));
                                    algoobj.tracking_confidence = object->tracking_confidence;
                                    algoobj.is_detection = object->is_detection;
                                    algoobjs.objs.push_back(algoobj);
                                }
                                ptr += object_meta->count * 4 * object_meta->size;
                                algopack.objs.push_back(algoobjs);
                            }
                          }
                          break;
                        case 1:
                          {
                            auto begin = pkt + sizeof(STAlgopackV2_1::Header) + sizeof(uint32_t);
                            for (
                                auto ptr = begin;
                                ptr - begin < ((const STAlgopackV2_1::Header*)pkt)->data_length - sizeof(STAlgopackV2_1::ObjectMeta);

                                ) {
                                auto object_meta = (const STAlgopackV2_1::ObjectMeta*)ptr;
                                Algoobjs algoobjs;
                                algoobjs.type = object_meta->type;
                                algoobjs.count = object_meta->count;
                                algoobjs.size = object_meta->size;
                                assert(object_meta->size * 4 == sizeof(STAlgopackV2_1::Object));
                                ptr += sizeof(STAlgopackV2_1::ObjectMeta);
                                for (auto i = 0; i < object_meta->count; i++) {
                                    auto object = (const STAlgopackV2_1::Object*)(ptr + i * 4 * object_meta->size);
                                    Algoobj algoobj;
                                    algoobj.id = object->id;
                                    algoobj.timestamp = object->timestamp;
                                    std::memcpy(&algoobj.pos_quat, &object->pos_quat, sizeof(algoobj.pos_quat));
                                    std::memcpy(&algoobj.lwh, &object->lwh, sizeof(algoobj.lwh));
                                    std::memcpy(&algoobj.center, &object->center, sizeof(algoobj.center));
                                    std::memcpy(&algoobj.relative_velocity, &object->relative_velocity, sizeof(algoobj.relative_velocity));
                                    std::memcpy(&algoobj.absolute_velocity, &object->absolute_velocity, sizeof(algoobj.absolute_velocity));
                                    std::memcpy(&algoobj.acceleration, &object->acceleration, sizeof(algoobj.acceleration));
                                    std::memcpy(&algoobj.location_cov, &object->location_cov, sizeof(algoobj.location_cov));
                                    std::memcpy(&algoobj.velocity_cov, &object->velocity_cov, sizeof(algoobj.velocity_cov));
                                    std::memcpy(&algoobj.acceleration_cov, &object->acceleration_cov, sizeof(algoobj.acceleration_cov));
                                    algoobj.tracking_confidence = object->tracking_confidence;
                                    algoobj.is_detection = object->is_detection;
                                    algoobjs.objs.push_back(algoobj);
                                }
                                ptr += object_meta->count * 4 * object_meta->size;
                                algopack.objs.push_back(algoobjs);
                            }
                          }
                          break;
                        default:
                          break;
                    }
                    break;
                  default:
                    break;
              }
              publisher.publish(algopack);

              visualization_msgs::MarkerArray obj_boxes;
              int id = 0;
              for(auto& algoobjs: algopack.objs) {
                for(auto& algoobj: algoobjs.objs) {
                  visualization_msgs::Marker obj_box;
                  obj_box.header.frame_id = "Pandar64";
                  obj_box.header.stamp = ros::Time::now();
                  obj_box.ns = "pandarmind64_algopack";
                  obj_box.action = visualization_msgs::Marker::ADD;
                  obj_box.id = id++;
                  obj_box.type = visualization_msgs::Marker::LINE_STRIP;
                  obj_box.scale.x = 0.1;
                  switch (algoobjs.type)
                  {
                  case 0:
                    obj_box.color.r = 1.0;
                    obj_box.color.g = 0.0;
                    obj_box.color.b = 0.0;
                    obj_box.color.a = 1.0;
                    break;
                  case 1:
                    obj_box.color.r = 1.0;
                    obj_box.color.g = 1.0;
                    obj_box.color.b = 0.0;
                    obj_box.color.a = 1.0;
                    break;
                  case 2:
                    obj_box.color.r = 0.0;
                    obj_box.color.g = 1.0;
                    obj_box.color.b = 0.0;
                    obj_box.color.a = 1.0;
                    break;
                  case 3:
                    obj_box.color.r = 0.0;
                    obj_box.color.g = 0.0;
                    obj_box.color.b = 1.0;
                    obj_box.color.a = 1.0;
                    break;
                  case 4:
                    obj_box.color.r = 0.5;
                    obj_box.color.g = 0.0;
                    obj_box.color.b = 0.5;
                    obj_box.color.a = 1.0;
                    break;
                  default:
                    obj_box.color.r = 0.3;
                    obj_box.color.g = 0.3;
                    obj_box.color.b = 0.3;
                    obj_box.color.a = 1.0;
                    break;
                  }
                  auto R = Eigen::Quaternionf(algoobj.pos_quat.data()).toRotationMatrix();
                  Eigen::Vector3f front = R * Eigen::Vector3f{ 1, 0, 0 };
                  Eigen::Vector3f left = R * Eigen::Vector3f{ 0, 1, 0 };
                  Eigen::Vector3f up = R * Eigen::Vector3f{ 0, 0, 1 };
                  auto l = algoobj.lwh[0], w = algoobj.lwh[1], h = algoobj.lwh[2];
                  Eigen::Vector3f center(algoobj.center[0], algoobj.center[1], algoobj.center[2]);
                  auto toPoint = [](const Eigen::Vector3f& vec3) -> geometry_msgs::Point {
                    geometry_msgs::Point point;
                    point.x = vec3[0];
                    point.y = vec3[1];
                    point.z = vec3[2];
                    return point;
                  };
                  geometry_msgs::Point
                    centeer(toPoint(center)),
                    froont(toPoint(center + l * front)),
                    up_right_front(toPoint(center + l / 2 * front - w / 2 * left + h / 2 * up)),
                    up_left_front(toPoint(center + l / 2 * front + w / 2 * left + h / 2 * up)), 
                    up_left_back(toPoint(center - l / 2 * front + w / 2 * left + h / 2 * up)),
                    up_right_back(toPoint(center - l / 2 * front - w / 2 * left + h / 2 * up)),
                    down_right_front(toPoint(center + l / 2 * front - w / 2 * left - h / 2 * up)),
                    down_left_front(toPoint(center + l / 2 * front + w / 2 * left - h / 2 * up)),
                    down_left_back(toPoint(center - l / 2 * front + w / 2 * left - h / 2 * up)),
                    down_right_back(toPoint(center - l / 2 * front - w / 2 * left - h / 2 * up))
                  ;
                  obj_box.points.push_back(centeer);
                  obj_box.points.push_back(froont);

                  obj_box.points.push_back(up_right_front);
                  obj_box.points.push_back(up_left_front);
                  obj_box.points.push_back(up_left_front);
                  obj_box.points.push_back(up_left_back);
                  obj_box.points.push_back(up_left_back);
                  obj_box.points.push_back(up_right_back);
                  obj_box.points.push_back(up_right_back);
                  obj_box.points.push_back(up_right_front);

                  obj_box.points.push_back(down_right_front);
                  obj_box.points.push_back(down_left_front);
                  obj_box.points.push_back(down_left_front);
                  obj_box.points.push_back(down_left_back);
                  obj_box.points.push_back(down_left_back);
                  obj_box.points.push_back(down_right_back);
                  obj_box.points.push_back(down_right_back);
                  obj_box.points.push_back(down_right_front);

                  obj_box.points.push_back(up_right_front);
                  obj_box.points.push_back(down_right_front);
                  obj_box.points.push_back(up_left_front);
                  obj_box.points.push_back(down_left_front);
                  obj_box.points.push_back(up_left_back);
                  obj_box.points.push_back(down_left_back);
                  obj_box.points.push_back(up_right_back);
                  obj_box.points.push_back(down_right_back);
                  
                  obj_boxes.markers.push_back(obj_box);
                }
              }
              while(id < 256) {
                  visualization_msgs::Marker obj_box;
                  obj_box.header.frame_id = "Pandar64";
                  obj_box.header.stamp = ros::Time::now();
                  obj_box.ns = "pandarmind64_algopack";
                  obj_box.action = visualization_msgs::Marker::ADD;
                  obj_box.id = id++;
                  obj_box.type = visualization_msgs::Marker::LINE_STRIP;
                  obj_box.color.a = 0;
                  obj_box.pose.position.x = 0;
                  obj_box.pose.position.y = 0;
                  obj_box.pose.position.z = 0;
                  obj_box.scale.x = 0.0;
                  obj_box.scale.y = 0.0;
                  obj_box.scale.z = 0.0;
                  obj_boxes.markers.push_back(obj_box);
              }
              markerarray_publisher.publish(obj_boxes);
          }
      }
  }

  ros::spin();
  return 0;
}