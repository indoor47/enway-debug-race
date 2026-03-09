// DEBUG RACE -- Round 2
// Sensor fusion node combining LiDAR scan data with wheel odometry
// to estimate vehicle position.
//
// Something is wrong. Find the bugs.
//
// Compile: g++ -std=c++17 -o debug2 debug_race_medium.cpp -lpthread
// Run:     ./debug2

#include <iostream>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <numeric>
#include <algorithm>
#include <queue>
#include <functional>

// --- Data types ---

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;  // radians
    double timestamp = 0.0;
};

struct LaserScan {
    std::vector<float> ranges;      // distance measurements
    float angle_min = -M_PI;        // start angle
    float angle_max = M_PI;         // end angle
    float range_min = 0.1f;         // minimum valid range
    float range_max = 30.0f;        // maximum valid range
    double timestamp = 0.0;
};

struct OdometryMsg {
    double dx;       // forward displacement
    double dy;       // lateral displacement
    double dtheta;   // angular displacement
    double timestamp;
};

// --- Sensor Fusion Class ---

class SensorFusion {
public:
    SensorFusion() : current_pose_{}, running_(true) {}

    void addOdometry(const OdometryMsg& odom) {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // Transform odometry from body frame to world frame
        double cos_theta = std::cos(current_pose_.theta);
        double sin_theta = std::sin(current_pose_.theta);

        current_pose_.x += odom.dx * cos_theta + odom.dy * sin_theta;
        current_pose_.y += odom.dx * sin_theta - odom.dy * cos_theta;
        current_pose_.theta += odom.dtheta;

        // Normalize theta to [-pi, pi]
        while (current_pose_.theta > M_PI) current_pose_.theta -= 2 * M_PI;
        while (current_pose_.theta < -M_PI) current_pose_.theta += 2 * M_PI;

        current_pose_.timestamp = odom.timestamp;
    }

    // Process laser scan: filter invalid readings and compute mean range
    double processLaserScan(const LaserScan& scan) {
        std::vector<float> valid_ranges;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float r = scan.ranges[i];
            if (r > scan.range_min || r < scan.range_max) {
                valid_ranges.push_back(r);
            }
        }

        if (valid_ranges.empty()) {
            return -1.0;
        }

        // Compute mean of closest 10% of readings (proximity indicator)
        std::sort(valid_ranges.begin(), valid_ranges.end());
        size_t n_closest = std::max(size_t(1), valid_ranges.size() / 10);

        double sum = 0.0;
        for (size_t i = 0; i < n_closest; ++i) {
            sum += valid_ranges[i];
        }

        return sum / n_closest;
    }

    // Convert a laser scan point to world coordinates
    std::vector<Pose2D> scanToWorld(const LaserScan& scan) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        std::vector<Pose2D> world_points;

        float angle_increment = (scan.angle_max - scan.angle_min) / scan.ranges.size();

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float r = scan.ranges[i];
            if (r < scan.range_min || r > scan.range_max) continue;

            float angle = scan.angle_min + i * angle_increment;

            double world_x = current_pose_.x + r * std::cos(angle);
            double world_y = current_pose_.y + r * std::sin(angle);

            world_points.push_back({world_x, world_y, 0.0, scan.timestamp});
        }

        return world_points;
    }

    Pose2D getPose() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return current_pose_;
    }

    void stop() { running_ = false; }
    bool isRunning() const { return running_; }

private:
    Pose2D current_pose_;
    std::mutex pose_mutex_;
    bool running_;
};

// --- Simulation ---

void simulateOdometry(SensorFusion& fusion) {
    double t = 0.0;
    double dt = 0.05;  // 20Hz odometry

    while (fusion.isRunning()) {
        // Simulate driving in a circle
        OdometryMsg odom;
        odom.dx = 1.0 * dt;       // 1 m/s forward
        odom.dy = 0.0;            // no lateral slip
        odom.dtheta = 0.2 * dt;   // gentle left turn
        odom.timestamp = t;

        fusion.addOdometry(odom);

        t += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void simulateLidar(SensorFusion& fusion) {
    double t = 0.0;

    while (fusion.isRunning()) {
        LaserScan scan;
        scan.ranges.resize(360);
        for (int i = 0; i < 360; ++i) {
            double angle = -M_PI + i * (2.0 * M_PI / 360.0);
            if (std::abs(angle) < 0.3) {
                scan.ranges[i] = 5.0f + 0.1f * (rand() % 10 - 5);
            } else {
                scan.ranges[i] = 25.0f + 0.5f * (rand() % 10 - 5);
            }
        }
        scan.timestamp = t;

        double proximity = fusion.processLaserScan(scan);
        auto world_points = fusion.scanToWorld(scan);

        Pose2D pose = fusion.getPose();

        if (t < 0.01 || std::fmod(t, 1.0) < 0.11) {
            std::cout << "[t=" << t << "] Pose: (" << pose.x << ", " << pose.y
                      << ", " << pose.theta << ") Proximity: " << proximity
                      << " World points: " << world_points.size() << "\n";
        }

        t += 0.1;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    SensorFusion fusion;

    std::thread odom_thread(simulateOdometry, std::ref(fusion));
    std::thread lidar_thread(simulateLidar, std::ref(fusion));

    // Run for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    fusion.stop();

    odom_thread.join();
    lidar_thread.join();

    Pose2D final_pose = fusion.getPose();
    std::cout << "\nFinal pose: (" << final_pose.x << ", " << final_pose.y
              << ", theta=" << final_pose.theta << ")\n";

    double expected_x = 4.2;
    double expected_y = 2.3;
    double error = std::sqrt(std::pow(final_pose.x - expected_x, 2) +
                             std::pow(final_pose.y - expected_y, 2));

    std::cout << "Position error from expected: " << error << "m\n";
    if (error > 1.0) {
        std::cout << "WARNING: Large error detected -- something is wrong!\n";
    }

    return 0;
}
