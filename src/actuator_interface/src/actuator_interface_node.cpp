#include <JetsonGPIO.h>

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#define SERVO_PIN 15
#define ESC_PIN 33

// using std::placeholders::_1;

// class ActuatorInterfaceNode : public rclcpp::Node {
//    public:
//     ActuatorInterfaceNode() : Node("minimal_subscriber") {
//         subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "topic", 10,
//             std::bind(&ActuatorInterfaceNode::topicCallback, this, _1));

//         GPIO::setmode(GPIO::BOARD);
//         GPIO::NumberingModes mode = GPIO::getmode();
//         std::string info = GPIO::JETSON_INFO();
//         std::string model = GPIO::model();
//         std::string version = GPIO::VERSION;
//         // TODO Log this info
//         GPIO::setup(SERVO_PIN, GPIO::OUT, GPIO::HIGH);
//         GPIO::setup(ESC_PIN, GPIO::OUT, GPIO::HIGH);
//         servo_pwm_ = std::make_unique<GPIO::PWM>(SERVO_PIN, 50);
//         esc_pwm_ = std::make_unique<GPIO::PWM>(ESC_PIN, 1000);
//     }

//    private:
//     void armEsc() {
//         esc_pwm_->start(0);
//         RCLCPP_INFO(this->get_logger(),
//                     "Connect to power in the next 3 seconds...");
//         for (int i = 3; i > 0; i--) {
//             std::this_thread::sleep_for(std::chrono::seconds(1));
//             RCLCPP_INFO(this->get_logger(), "Connect to power in %d
//             seconds...",
//                         i);
//         }
//         esc_pwm_->ChangeDutyCycle(0);
//         std::this_thread::sleep_for(std::chrono::seconds(4));
//     }
//     void calibrateEsc() {
//         esc_pwm_->start(100);
//         std::string input;
//         std::cout << "Connect power and press Enter to continue... ";
//         std::getline(std::cin, input);

//         esc_pwm_->ChangeFrequency(100);
//         std::this_thread::sleep_for(std::chrono::seconds(2));
//         esc_pwm_->start(0);
//         std::this_thread::sleep_for(std::chrono::seconds(4));
//     }
//     void topicCallback(const std_msgs::msg::String::SharedPtr msg) const {
//         // Set steering angle
//         // Set drive velocity
//     }
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
//     std::unique_ptr<GPIO::PWM> servo_pwm_;
//     std::unique_ptr<GPIO::PWM> esc_pwm_;
//     const int MIN_WIDTH_ESC = 1000;    // us
//     const int MAX_WIDTH_ESC = 2000;    // us
//     const float TIRE_DIAMETER = 0.11;  // m
//     const int GEAR_RATIO = 42;
//     const float MAX_RAW_RPM = 10400;

//     const float RAD_TO_PWM = 900.0 / (M_PI / 2.0);
//     const int CENTER_PWM = 1550;
//     const float TURN_PHI_MAX = 0.558505361;
//     const float TURN_PHI_MIN = -0.820304748;
//     const float MAX_VEL =
//         ((MAX_RAW_RPM / GEAR_RATIO) / 60.0) * ((TIRE_DIAMETER * M_PI));
// };

int main(int argc, char* argv[]) {
    GPIO::setmode(GPIO::BOARD);
    GPIO::setup(SERVO_PIN, GPIO::OUT, GPIO::HIGH);
    GPIO::PWM s(SERVO_PIN, 333);
    s.start(50);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    s.stop();
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ActuatorInterfaceNode>());
    // rclcpp::shutdown();
    GPIO::cleanup();
    return 0;
}