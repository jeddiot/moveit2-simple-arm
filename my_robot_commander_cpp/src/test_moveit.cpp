#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper =
        moveit::planning_interface::MoveGroupInterface(node, "gripper");

    // Named goals
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pose_1");
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        arm.execute(plan1);
    }

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    bool success2 = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success2) {
        arm.execute(plan2);
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}