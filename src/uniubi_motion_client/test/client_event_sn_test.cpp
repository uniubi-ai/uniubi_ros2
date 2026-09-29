#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/event_message.hpp"
#include "uniubi/srv/system.hpp"
#include "uniubi_motion_client/motion_high_level_client.hpp"

using namespace std::chrono_literals;
static void check(bool ok) {if (!ok) throw std::runtime_error("SN event filtering failed");}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<rclcpp::Node>("sn_event_server");
  auto client_node_a = std::make_shared<rclcpp::Node>("sn_event_client_a");
  auto client_node_b = std::make_shared<rclcpp::Node>("sn_event_client_b");
  auto client_node_c = std::make_shared<rclcpp::Node>("sn_event_client_board");
  auto service = server_node->create_service<uniubi::srv::System>(
    "/sn_event_test_robot", [](const uniubi::srv::System::Request::SharedPtr, uniubi::srv::System::Response::SharedPtr) {});
  auto publisher = server_node->create_publisher<uniubi::msg::EventMessage>(
    "/robotServer/Event", rclcpp::QoS(10));
  auto board_publisher = server_node->create_publisher<uniubi::msg::EventMessage>(
    "/robotCereServer/Event", rclcpp::QoS(10));
  rclcpp::executors::SingleThreadedExecutor server_executor, executor_a, executor_b, executor_c;
  server_executor.add_node(server_node);
  executor_a.add_node(client_node_a);
  executor_b.add_node(client_node_b);
  executor_c.add_node(client_node_c);
  std::thread server_thread([&]() {server_executor.spin();});
  std::thread thread_a([&]() {executor_a.spin();});
  std::thread thread_b([&]() {executor_b.spin();});
  std::thread thread_c([&]() {executor_c.spin();});
  std::atomic<int> count_a{0}, count_b{0}, count_c{0};
  {
    uniubi_motion_client::MotionHighLevelClient a(
      client_node_a, executor_a, "/sn_event_test_robot", "dog57");
    uniubi_motion_client::MotionHighLevelClient b(
      client_node_b, executor_b, "/sn_event_test_robot", "dog58");
    uniubi_motion_client::MotionHighLevelClient c(
      client_node_c, executor_c, "/sn_event_test_robot", "dog57",
      "/robotCereServer/Event", "", "", "cere_motion_state");
    c.setEventCallback([&](const std::string & topic, const std::string &) {
      if (topic == "marker") ++count_c;
    });
    a.setEventCallback([&](const std::string & topic, const std::string &) {
      if (topic == "marker") ++count_a;
    });
    b.setEventCallback([&](const std::string & topic, const std::string &) {
      if (topic == "marker") ++count_b;
    });
    check(a.connect() && b.connect() && c.connect());
    auto send = [&](const std::string & topic) {
      uniubi::msg::EventMessage m;
      m.magic = 0x53425645U;
      m.topic = topic;
      m.payload = R"({"event":"marker","detail":{"value":1}})";
      publisher->publish(m);
    };
    std::this_thread::sleep_for(400ms);
    for (int i = 0; i < 10; ++i) {
      send("dog57.robotServer.host.event");
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(100ms);
    check(count_a > 0 && count_b == 0);
    const int before_a = count_a;
    for (int i = 0; i < 10; ++i) {
      send("dog58.robotServer.host.event");
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(100ms);
    check(count_a == before_a && count_b > 0);
    const int before_b = count_b;
    send("robotServer.host.event");
    send("dog59.robotServer.host.event");
    std::this_thread::sleep_for(100ms);
    check(count_a == before_a && count_b == before_b);
    uniubi::msg::EventMessage board_event;
    board_event.magic = 0x53425645U;
    board_event.topic = "robotServer.host.event";
    board_event.payload = R"({"event":"marker","detail":{"value":1}})";
    for (int i = 0; i < 10; ++i) {
      board_publisher->publish(board_event);
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(100ms);
    check(count_a == before_a && count_b == before_b && count_c > 0);
  }
  server_executor.cancel(); executor_a.cancel(); executor_b.cancel(); executor_c.cancel();
  server_thread.join(); thread_a.join(); thread_b.join(); thread_c.join();
  service.reset(); publisher.reset(); board_publisher.reset();
  server_node.reset(); client_node_a.reset(); client_node_b.reset(); client_node_c.reset();
  rclcpp::shutdown();
}
