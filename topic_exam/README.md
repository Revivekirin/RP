# 2025 Robot Programming – Week 2 Assignments (ROS 2 Foxy, C++)

> Target: ROS 2 **Foxy** on Ubuntu 20.04, `rclcpp` (C++). Build with **colcon**.

---

## Week 2‑1: Topic (Publisher & Subscriber)

**Ref**: [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

### Learning Objectives

* ROS 2 토픽 통신 모델(발행/구독), QoS 이해
* 타이머/콜백을 이용하여 주기적 메시지 발행
* 실행 옵션을 통한 파라미터화(topic 이름, 주기, frame 등)

### Tasks

1. `cpp_talker_listener` 패키지 생성

   ```bash
   ros2 pkg create --build-type ament_cmake cpp_talker_listener --dependencies rclcpp std_msgs
   ```
2. **Publisher(Node: `talker`)**

   * 10 Hz로 `std_msgs::msg::String` 발행
   * 파라미터: `topic_name`(기본값: `/chatter`), `rate_hz`(기본값: 10)
   * 로그: 메시지 번호 포함 (예: `hello world 42`)
3. **Subscriber(Node: `listener`)**

   * 같은 QoS(Profile: `rclcpp::QoS(10).best_effort()` 과제 옵션)
   * 콜백에서 수신 문자열을 로그로 출력
4. **CLI 인자 실험**

   * `--ros-args -p rate_hz:=5 -p topic_name:=/demo_topic` 로 주기와 토픽을 변경하여 실행
5. **QoS 실험 보고**

   * `Reliable` vs `BestEffort`, 큐 사이즈 1/10 비교 (drop 여부, 지연)
6. **(도전)** 단일 실행 파일에 `talker`와 `listener` 동시에 띄우기(멀티스레드 executor)

### Minimal Code Sketches

**`src/talker.cpp`**

```cpp
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker"), count_(0) {
    topic_name_ = this->declare_parameter<std::string>("topic_name", "chatter");
    int rate_hz = this->declare_parameter<int>("rate_hz", 10);
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name_, rclcpp::QoS(10));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / std::max(1, rate_hz)),
      std::bind(&Talker::on_timer, this));
  }
private:
  void on_timer(){
    auto msg = std_msgs::msg::String();
    msg.data = "hello world " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    pub_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string topic_name_;
  int count_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
```

**`src/listener.cpp`**

```cpp
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node {
public:
  Listener() : Node("listener") {
    auto topic_name = this->declare_parameter<std::string>("topic_name", "chatter");
    sub_ = this->create_subscription<std_msgs::msg::String>(
      topic_name, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
```

### CMake & Package Setup

**`CMakeLists.txt` (핵심 부분)**

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS talker listener DESTINATION lib/${PROJECT_NAME})

ament_package()
```

**`package.xml` (핵심 의존성)**

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### Build & Run

```bash
colcon build --packages-select cpp_talker_listener
source install/setup.bash
ros2 run cpp_talker_listener talker --ros-args -p rate_hz:=5 -p topic_name:=/demo
ros2 run cpp_talker_listener listener --ros-args -p topic_name:=/demo
```

### Common Pitfalls

* `rclcpp::NodeHandle`(ROS1 스타일) 사용 금지. Foxy에서는 `rclcpp::Node`.
* `std_msgs/msg/String.hpp` (ROS2 경로) 포함 확인.
* 빌드 후 `source install/setup.bash` 잊지 않기.

---

## Week 2‑2: Service (Server & Client)

**Ref**: [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)

### Learning Objectives

* 요청/응답(Service) 통신과 비동기 클라이언트
* 메시지/서비스 인터페이스 사용법

### Tasks

1. `cpp_add_two_ints` 패키지 생성 (의존성: `rclcpp`, `example_interfaces`)

   ```bash
   ros2 pkg create --build-type ament_cmake cpp_add_two_ints --dependencies rclcpp example_interfaces
   ```
2. **Server(Node: `adder_server`)** – `example_interfaces/srv/AddTwoInts`

   * 요청 a, b를 더해 `sum` 반환
   * 요청/응답 내용을 로그로 출력
3. **Client(Node: `adder_client`)**

   * CLI 인자 또는 파라미터로 a, b 지정 (기본값 1, 2)
   * 서비스가 준비될 때까지 `wait_for_service`
   * 비동기 요청 후 결과 수신
4. **(확장)** `MultiplyTwoInts` 사용자 정의 srv를 추가로 만들어보기
5. **(테스트)** 잘못된 인자(오버플로, 음수 등) 케이스 로그/에러 처리

### Minimal Code Sketches

**`src/adder_server.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <memory>

class AdderServer : public rclcpp::Node {
public:
  AdderServer() : Node("adder_server") {
    srv_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
             std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response){
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "request: a=%ld b=%ld -> sum=%ld",
                    request->a, request->b, response->sum);
      });
  }
private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdderServer>());
  rclcpp::shutdown();
  return 0;
}
```

**`src/adder_client.cpp`**

```cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class AdderClient : public rclcpp::Node {
public:
  AdderClient() : Node("adder_client") {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    a_ = this->declare_parameter<int64_t>("a", 1);
    b_ = this->declare_parameter<int64_t>("b", 2);
  }

  void call(){
    while(!client_->wait_for_service(1s)){
      if(!rclcpp::ok()) return;
      RCLCPP_INFO(this->get_logger(), "waiting for service...");
    }
    auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    req->a = a_; req->b = b_;
    auto future = client_->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(shared_from_this(), future);
    if(ret == rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(this->get_logger(), "sum = %ld", future.get()->sum);
    } else {
      RCLCPP_ERROR(this->get_logger(), "service call failed");
    }
  }
private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  int64_t a_, b_;
};

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdderClient>();
  node->call();
  rclcpp::shutdown();
  return 0;
}
```

### CMake & Package Setup

**`CMakeLists.txt` (핵심 부분)**

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(adder_server src/adder_server.cpp)
ament_target_dependencies(adder_server rclcpp example_interfaces)

add_executable(adder_client src/adder_client.cpp)
ament_target_dependencies(adder_client rclcpp example_interfaces)

install(TARGETS adder_server adder_client DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### Build & Run

```bash
colcon build --packages-select cpp_add_two_ints
source install/setup.bash
ros2 run cpp_add_two_ints adder_server
# 다른 터미널
source install/setup.bash
ros2 run cpp_add_two_ints adder_client --ros-args -p a:=7 -p b:=35
```

---

## Submission

* **Repo 구조**

  ```
  repo/
    week2-1_cpp_talker_listener/
      src/ talker.cpp listener.cpp
      CMakeLists.txt package.xml
      README.md (실험 결과/QoS 비교 표 포함)
    week2-2_cpp_add_two_ints/
      src/ adder_server.cpp adder_client.cpp
      CMakeLists.txt package.xml
      README.md (동작 스크린샷/로깅 캡처)
  ```
* **실험 보고**: QoS 설정별 패킷 드롭/지연 관찰 결과 표, 파라미터 변경 스크린샷
* **데모 영상/GIF** 10–30초 (옵션)

## Grading Rubric (20점)

* (6) 기능 구현: talker/listener, adder server/client 정상 동작
* (4) 파라미터/CLI 처리, QoS 실험 수행
* (4) 코드 품질: 네이밍, 로그, 주석, CMake/패키지 구성
* (3) 보고서: 비교/분석의 명확성, 재현 가능성
* (3) 확장/도전 과제 수행(멀티스레드 executor, 커스텀 srv 등)

---

## Troubleshooting Quick Notes

* **빌드 실패**: 의존성 누락 → `package.xml`/`CMakeLists.txt` 재확인, `rosdep install -i --from-path src -y`
* **런타임 토픽 미일치**: talker/listener `topic_name` 파라미터 동일하게
* **QoS 불일치**: 신뢰성/큐 크기 다르면 메시지 드롭 가능
* **환경 설정**: 매 터미널 `source install/setup.bash` 필요
