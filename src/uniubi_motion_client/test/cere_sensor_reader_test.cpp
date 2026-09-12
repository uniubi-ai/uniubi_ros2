#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>
#include "dds/dds.h"
#include "BrainMotionState.h"
#include "uniubi_motion_client/cere_sensor_reader.hpp"
static void require(bool ok) {if (!ok) throw std::runtime_error("cere sensor reader assertion failed");}
int main(int argc, char **argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("cere_sensor_reader_test");
 int received=0;uniubi::msg::SensorObserved last;
 auto reader=std::make_unique<uniubi_motion_client::CereSensorReader>(node,"rt/cere/testSensor",[&](const auto& m){++received;last=m;});
 const auto domain=node->get_node_base_interface()->get_context()->get_domain_id();
 const auto participant=dds_create_participant(domain,nullptr,nullptr);require(participant>0);
 const auto topic=dds_create_topic(participant,&uniubi_dds__BrainMotionState_desc,"rt/cere/testSensor",nullptr,nullptr);require(topic>0);
 auto*q=dds_create_qos();dds_qset_reliability(q,DDS_RELIABILITY_BEST_EFFORT,0);
 const auto writer=dds_create_writer(participant,topic,q,nullptr);dds_delete_qos(q);require(writer>0);
 auto spin=[&](int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(2));}};
 for(int i=0;i<50 && dds_get_matched_subscriptions(writer,nullptr,0)==0;++i)spin(100);
 require(dds_get_matched_subscriptions(writer,nullptr,0)>0);
 uniubi_dds__BrainMotionState sample{};sample.timestamp=123456;
 require(dds_write(writer,&sample)==0);spin(100);require(received==0);
 sample.hasSensor=1;sample.gps.valid=1;sample.gps.point.lat=30.25;sample.gps.point.lng=120.5;
 sample.gps.speed=2.5;sample.gps.level=2;sample.gps.rssi=-30;
 sample.uwb.valid=1;sample.uwb.beaconId=1234;sample.uwb.pairState=2;sample.uwb.rssi=-40;
 sample.uwb.pitch=12;sample.uwb.azimuth=25;sample.uwb.distance=375;
 sample.odom.valid=1;sample.odom.epoch=7;sample.odom.yaw=.5;sample.odom.yawSpeed=.25;
 for(int i=0;i<3;++i){sample.odom.position[i]=i+1;sample.odom.velocity[i]=i+.5;}
 require(dds_write(writer,&sample)==0);spin(100);require(received==1);
 require(last.timestamp==123456000 && last.uwb.beacon_id==1234 && last.gps.valid==1);
 require(last.gps.point.lat==30.25 && last.gps.point.lng==120.5 && last.gps.speed==2.5 && last.gps.level==2 && last.gps.rssi==-30);
 require(last.uwb.valid==1 && last.uwb.pair_state==2 && last.uwb.rssi==-40 && last.uwb.pitch==12 && last.uwb.azimuth==25 && last.uwb.distance==375);
 require(last.odom.valid==1 && last.odom.epoch==7 && last.odom.yaw==.5 && last.odom.yaw_speed==.25);
 for(int i=0;i<3;++i)require(last.odom.position[i]==i+1 && last.odom.velocity[i]==i+.5);
 sample.gps.valid=0;sample.uwb.valid=0;sample.odom.valid=0;sample.hasMotion=1;
 require(dds_write(writer,&sample)==0);spin(100);require(received==2 && !last.gps.valid && !last.uwb.valid && !last.odom.valid);
 sample.timestamp=std::numeric_limits<uint64_t>::max();require(dds_write(writer,&sample)==0);spin(100);require(received==2);
 reader.reset();require(dds_write(writer,&sample)==0);spin(100);require(received==2);
 dds_delete(participant);node.reset();rclcpp::shutdown();
}
