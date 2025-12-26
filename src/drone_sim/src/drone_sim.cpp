#include <iostream>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class DroneSim : public rclcpp::Node {
public:
    DroneSim() : Node("drone_sim") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&DroneSim::joyCallback, this, std::placeholders::_1)
        );

        lastGravity = std::chrono::steady_clock::now();
        lastReturn  = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "Drone simulation node started.");
    }

    void update() {
        render();
        handleReturnToBase();
        handleGravity();
    }

private:

    double x = 0, y = 0, z = 0;
    double speed = 1.0;
    double returnSpeed = 5.0;

    bool hoverLock = false;
    bool returnToBase = false;


    std::chrono::steady_clock::time_point lastGravity;
    std::chrono::steady_clock::time_point lastReturn;

    int gravityInterval = 100;
    int returnInterval  = 50; 


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;


    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (returnToBase)
            return;

        double lx = msg->axes[0];
        double ly = msg->axes[1];
        double rz = msg->axes[4];

        x += ly * speed;
        y += lx * speed;
        z += rz * speed;

        if (z < 0) z = 0;

        if (msg->buttons[0]) {
            hoverLock = !hoverLock;
        }

        if (msg->buttons[1]) {
            returnToBase = true;
        }

        if (msg->buttons[4]) speed -= 0.1;
        if (msg->buttons[5]) speed += 0.1;

        if (speed < 1) speed = 1;
        if (speed > 5) speed = 5;
    }


    void handleGravity() {
        if (hoverLock || returnToBase)
            return;

        auto now = std::chrono::steady_clock::now();
        auto msPassed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastGravity).count();

        if (msPassed >= gravityInterval) {
            z -= 0.1;
            if (z < 0) z = 0;
            lastGravity = now;
        }
    }

    void handleReturnToBase() {
        if (!returnToBase)
            return;

        auto now = std::chrono::steady_clock::now();
        auto msPassed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReturn).count();

        if (msPassed >= returnInterval) {
            auto moveToward = [&](double &coord) {
                if (coord > returnSpeed) coord -= returnSpeed;
                else if (coord < -returnSpeed) coord += returnSpeed;
                else coord = 0;
            };

            moveToward(x);
            moveToward(y);
            moveToward(z);

            lastReturn = now;
        }

        if (x == 0 && y == 0 && z == 0) {
            returnToBase = false;
        }
    }


    void render() {
        std::cout << "\033[2J\033[1;1H";
        std::cout << "Drone Simulation\n";
        std::cout << "Right Stick Y: Altitude | Left Stick: Horizontal Movement\n";
        std::cout << "LB/RB: Decrease/Increase Speed | A: Toggle Hover Lock | B: Return to Base\n";
        std::cout << "-----------------------\n";
        std::cout << "Position: (" << x << ", " << y << ", " << z << ")\n";
        std::cout << "Speed: " << speed << "\n";
        std::cout << "Hover Lock: " << (hoverLock ? "ON" : "OFF") << "\n";
        std::cout << "Return Mode: " << (returnToBase ? "ONLINE" : "OFFLINE") << "\n";
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DroneSim>();
    rclcpp::Rate loop_rate(30); // 30 Hz

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->update();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
