#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Vector3d;

class ImuCovarianceNode : public rclcpp::Node
{
public:
    ImuCovarianceNode(int iterations, std::string file_name, std::string imu_topic)
    : Node("imu_covariance_node"), max_iterations_(iterations), current_iteration_(0), filename_(file_name)
    {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10,
            std::bind(&ImuCovarianceNode::imu_callback, this, std::placeholders::_1));
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (max_iterations_ == 0) return; // Avoid division by zero

        if (current_iteration_ >= max_iterations_) 
        {
            rclcpp::shutdown();
            return;
        }

        // Subscribe Orientation, Angular Velocity, Linear Acceleration
        // Convert Quaternion to Euler

        Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        orientation_data_.emplace_back(euler[0], euler[1], euler[2]);
        angular_velocity_data_.emplace_back(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        linear_acceleration_data_.emplace_back(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        current_iteration_++;

        // Print percentage of max_iterations_
        static int prePercentage = 0;
        int percentage = (current_iteration_ * 100) / max_iterations_;

        if (prePercentage != percentage) 
        {
            prePercentage = percentage;
            std::cout << "Reached " << percentage << "% of max_iterations_" << std::endl;
        }

        // File write and finish node when the number of iterations is reached
        if (current_iteration_ == max_iterations_) 
        {
            calculate_and_save(filename_);
            rclcpp::shutdown();
        }
    }

    void calculate_and_save(std::string filename)
    {
        Vector3d orientation_mean = calculate_mean(orientation_data_);
        Matrix3d orientation_cov = calculate_covariance(orientation_data_, orientation_mean);

        Vector3d angular_velocity_mean = calculate_mean(angular_velocity_data_);
        Matrix3d angular_velocity_cov = calculate_covariance(angular_velocity_data_, angular_velocity_mean);

        Vector3d linear_acceleration_mean = calculate_mean(linear_acceleration_data_);
        Matrix3d linear_acceleration_cov = calculate_covariance(linear_acceleration_data_, linear_acceleration_mean);

        std::ofstream file(filename);
        if (file.is_open()) 
        {
            file << "Orientation covariance matrix:\n";
            file << orientation_cov << "\n\n";

            file << "Angular velocity covariance matrix:\n";
            file << angular_velocity_cov << "\n\n";

            file << "Linear acceleration covariance matrix:\n";
            file << linear_acceleration_cov << "\n\n";

            file.close();
            std::string result = "Data saved to " + filename;
            RCLCPP_INFO(this->get_logger(), result.c_str());
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
        }
    }

    Vector3d calculate_mean(const std::vector<Vector3d>& data)
    {
        Vector3d mean = Vector3d::Zero();
        for (const auto& sample : data) 
        {
            mean += sample;
        }
        return mean / data.size();
    }

    Matrix3d calculate_covariance(const std::vector<Vector3d>& data, const Vector3d& mean)
    {
        Matrix3d covariance = Matrix3d::Zero();
        for (const auto& sample : data) 
        {
            Vector3d diff = sample - mean;
            covariance += diff * diff.transpose();
        }
        return covariance / (data.size() - 1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    int max_iterations_;
    int current_iteration_;
    std::string filename_;
    std::vector<Vector3d> orientation_data_;
    std::vector<Vector3d> angular_velocity_data_;
    std::vector<Vector3d> linear_acceleration_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    int iterations = 100;  // number of iteration 
    std::string filename = "covariance_matrix.txt";  // file name to save covariance matrix
    std::string imu_topic = "imu/data";
    if (argc > 1) 
    {
        iterations = std::stoi(argv[1]);
        if (argc > 2) filename = argv[2];
        if (argc > 3) imu_topic = argv[3];
    }
    rclcpp::spin(std::make_shared<ImuCovarianceNode>(iterations, filename, imu_topic));
    rclcpp::shutdown();
    return 0;
}
