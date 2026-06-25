#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;  // 100ms, 1s 같은 표현 사용 가능
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoParkingNode : public rclcpp::Node
{
public:
    // =================================================
    // Python: def __init__(self):
    // C++:    생성자
    // =================================================
    AutoParkingNode() : Node("auto_parking_node")
    {
        // ----- 파라미터 선언 -----
        // Python: self.declare_parameter('goal_x', 0.0)
        // C++:    this->declare_parameter("goal_x", 0.0);
        this->declare_parameter("goal_x", 0.0);
        this->declare_parameter("goal_y", 0.0);
        this->declare_parameter("white_threshold", 2000);
        this->declare_parameter("us_stop_dist", 0.065);
        this->declare_parameter("line_kp", -0.0005);
        this->declare_parameter("line_angular_limit", 0.3);

        // Python: self.goal_x = self.get_parameter('goal_x').value
        // C++:    goal_x_ = this->get_parameter("goal_x").as_double();
        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        white_threshold_ = this->get_parameter("white_threshold").as_int();
        us_stop_dist_ = this->get_parameter("us_stop_dist").as_double();
        line_kp_ = this->get_parameter("line_kp").as_double();
        line_angular_limit_ = this->get_parameter("line_angular_limit").as_double();

        // ----- 퍼블리셔 -----
        // Python: self.cmd_pub = self.create_publisher(Twist, '/pinky1/cmd_vel', 10)
        // C++:    cmd_pub_ = this->create_publisher<타입>("토픽명", 10);
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/pinky1/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pinky1/parking/auto_done", 10);

        // ----- 액션 클라이언트 -----
        // Python: ActionClient(self, NavigateToPose, '/pinky1/navigate_to_pose')
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/pinky1/navigate_to_pose");

        // ----- 서브스크립션 -----
        // Python: self.create_subscription(PoseWithCovarianceStamped, '/pinky1/amcl_pose', ...)
        // C++:    this->create_subscription<타입>("토픽명", QoS, 콜백);
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pinky1/amcl_pose",
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AutoParkingNode::pose_callback, this, std::placeholders::_1));

        ir_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/pinky1/ir_sensor/range", 10,
            std::bind(&AutoParkingNode::ir_callback, this, std::placeholders::_1));

        us_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/pinky1/us_sensor/range", 10,
            std::bind(&AutoParkingNode::us_callback, this, std::placeholders::_1));

        start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/pinky1/parking/auto_start", 10,
            std::bind(&AutoParkingNode::start_callback, this, std::placeholders::_1));

        // ----- 타이머 -----
        // Python: self.create_timer(0.1, self._rotation_tick)
        // C++:    this->create_wall_timer(100ms, 콜백);
        rotation_timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoParkingNode::rotation_tick, this));

        nav_check_timer_ = this->create_wall_timer(
            1s, std::bind(&AutoParkingNode::check_nav_server, this));

        RCLCPP_INFO(this->get_logger(), "AutoParkingNode 시작. 목표=(%f, %f)", goal_x_, goal_y_);
    }

private:
    // =================================================
    // Phase 1: Nav2 이동 (구현 완료)
    // =================================================

    void check_nav_server()
    {
        if (nav_client_->action_server_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Nav2 서버 준비 완료. 이동 시작.");
            nav_check_timer_->cancel();
            started_ = true;
            go_home();
        }
    }

    void go_home()
    {
        auto goal = NavigateToPose::Goal();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = goal_x_;
        goal.pose.pose.position.y = goal_y_;

        RCLCPP_INFO(this->get_logger(), "목표 전송: (%f, %f)", goal_x_, goal_y_);

        // Python: future = self._nav_client.send_goal_async(goal, feedback_callback=...)
        //         future.add_done_callback(self._nav_goal_response)
        // C++:    SendGoalOptions에 콜백들을 묶어서 한번에 보냄
        auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        options.feedback_callback =
            std::bind(&AutoParkingNode::nav_feedback, this,
                      std::placeholders::_1, std::placeholders::_2);
        options.result_callback =
            std::bind(&AutoParkingNode::nav_result, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal, options);
    }

    // Python: def _nav_feedback(self, feedback_msg):
    //             dist = feedback_msg.feedback.distance_remaining
    void nav_feedback(
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "이동 중... 남은 거리: %.2f m", feedback->distance_remaining);
    }

    // Python: def _nav_result(self, future):
    //             status = future.result().status
    void nav_result(const GoalHandleNav::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "도착. IR 주차 모드 시작.");
            start_ir_parking();
        } else {
            RCLCPP_ERROR(this->get_logger(), "네비게이션 실패. 종료.");
            finish(false);
        }
    }

    // =================================================
    // Phase 2: IR 라인트레이싱  ← TODO: 같이 채울 부분
    // =================================================

    void start_ir_parking()
    {
        ir_active_ = true;
    }

    void ir_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr /*msg*/)
    {
        // TODO: Python의 _ir_callback 로직을 여기에 구현
    }

    // =================================================
    // Phase 3: 초음파 정밀 주차  ← TODO: 같이 채울 부분
    // =================================================

    void start_us_parking()
    {
        us_active_ = true;
    }

    void us_callback(const sensor_msgs::msg::Range::SharedPtr /*msg*/)
    {
        // TODO: Python의 _us_callback 로직을 여기에 구현
    }

    // =================================================
    // Phase 4: 180도 회전  ← TODO: 같이 채울 부분
    // =================================================

    void start_rotation()
    {
        rotating_ = true;
        target_yaw_set_ = false;
    }

    void rotation_tick()
    {
        // TODO: Python의 _rotation_tick 로직을 여기에 구현
    }

    // =================================================
    // 공통 콜백
    // =================================================

    // Python: def _pose_callback(self, msg):
    //             self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        auto & q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        yaw_valid_ = true;
    }

    void start_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !started_) {
            started_ = true;
            RCLCPP_INFO(this->get_logger(), "주차 진입 시작 신호 수신.");
            go_home();
        }
    }

    // Python: def _finish(self, success):
    //             self.status_pub.publish(Bool(data=success))
    void finish(bool success)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = success;
        status_pub_->publish(msg);
        done_ = true;
        rclcpp::shutdown();
    }

    // =================================================
    // 멤버 변수 (Python에서 self.xxx 였던 것들)
    // =================================================

    // 상수
    static constexpr double LINEAR_VEL = 0.05;
    static constexpr double ANGULAR_VEL = 0.5;
    static constexpr double ROTATION_YAW_TOLERANCE = 3.0 * M_PI / 180.0;

    // 파라미터
    double goal_x_{}, goal_y_{};
    int white_threshold_{};
    double us_stop_dist_{};
    double line_kp_{};
    double line_angular_limit_{};

    // 상태
    bool started_ = false;
    bool ir_active_ = false;
    bool us_active_ = false;
    bool rotating_ = false;
    bool done_ = false;
    bool yaw_valid_ = false;
    bool target_yaw_set_ = false;
    double current_yaw_ = 0.0;
    double target_yaw_ = 0.0;

    // ROS 인터페이스
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr ir_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
    rclcpp::TimerBase::SharedPtr rotation_timer_;
    rclcpp::TimerBase::SharedPtr nav_check_timer_;
};

// =================================================
// Python: def main(args=None):
//             rclpy.init(args=args)
//             node = AutoParkingNode()
//             rclpy.spin(node)
// =================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoParkingNode>());
    rclcpp::shutdown();
    return 0;
}
