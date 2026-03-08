// DEBUG RACE -- Exercise 2: MEDIUM
// Bug difficulty: Medium
// Domain: Sensor fusion -- combining LiDAR and odometry data
//
// INSTRUCTIONS:
// This program simulates a simple sensor fusion node that combines
// LiDAR scan data with wheel odometry to estimate vehicle position.
// It has 4 bugs of varying subtlety. Find them.
//
// Compile: g++ -std=c++17 -o debug2 debug_race_medium.cpp -lpthread
// Run: ./debug2

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

        // BUG 1: Rotation matrix is wrong -- the y-component uses wrong signs
        // Correct rotation: x' = dx*cos - dy*sin, y' = dx*sin + dy*cos
        current_pose_.x += odom.dx * cos_theta + odom.dy * sin_theta;
        current_pose_.y += odom.dx * sin_theta - odom.dy * cos_theta;  // BUG: should be + odom.dy * cos_theta
        current_pose_.theta += odom.dtheta;

        // Normalize theta to [-pi, pi]
        while (current_pose_.theta > M_PI) current_pose_.theta -= 2 * M_PI;
        while (current_pose_.theta < -M_PI) current_pose_.theta += 2 * M_PI;

        current_pose_.timestamp = odom.timestamp;
    }

    // Process laser scan: filter invalid readings and compute mean range
    // Used as a simple "am I close to something?" proximity check
    double processLaserScan(const LaserScan& scan) {
        std::vector<float> valid_ranges;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            float r = scan.ranges[i];
            // BUG 2: Range validation is inverted
            // Should keep ranges BETWEEN min and max, but the logic is wrong
            if (r > scan.range_min || r < scan.range_max) {  // BUG: should be && not ||
                valid_ranges.push_back(r);
            }
        }

        if (valid_ranges.empty()) {
            return -1.0;  // no valid readings
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

            // Transform to world frame
            // BUG 3: Using the scan angle directly without adding the robot's heading
            // The local->world transform needs: world_angle = local_angle + robot_theta
            double world_x = current_pose_.x + r * std::cos(angle);  // BUG: missing + current_pose_.theta
            double world_y = current_pose_.y + r * std::sin(angle);  // BUG: missing + current_pose_.theta

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
    bool running_;  // BUG 4: not atomic -- reading from one thread while potentially
                    // writing from another is a data race. Should be std::atomic<bool>
};

// --- Simulation ---

void simulateOdometry(SensorFusion& fusion) {
    double t = 0.0;
    double dt = 0.05;  // 20Hz odometry

    while (fusion.isRunning()) {  // Reading `running_` without lock -- data race with stop()
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

    while (fusion.isRunning()) {  // Same data race
        LaserScan scan;
        // Simulate 360 readings
        scan.ranges.resize(360);
        for (int i = 0; i < 360; ++i) {
            // Simulate a wall at ~5m in front, open sides
            double angle = -M_PI + i * (2.0 * M_PI / 360.0);
            if (std::abs(angle) < 0.3) {
                scan.ranges[i] = 5.0f + 0.1f * (rand() % 10 - 5);  // wall with noise
            } else {
                scan.ranges[i] = 25.0f + 0.5f * (rand() % 10 - 5); // far away
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

    // With correct odometry at 1 m/s forward + 0.2 rad/s turn for 5 seconds:
    // The vehicle should trace roughly a circle arc
    // Total distance ~ 5m, total turn ~ 1 rad
    // Expected rough position: x~4.2, y~2.3 (circular arc)
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

// === ANSWER KEY (for facilitator) ===
//
// BUG 1 (Line ~57): Rotation matrix sign error
//   `current_pose_.y += odom.dx * sin_theta - odom.dy * cos_theta;`
//   Should be: `current_pose_.y += odom.dx * sin_theta + odom.dy * cos_theta;`
//   The rotation matrix for body->world is:
//     [cos -sin] [dx]     [dx*cos - dy*sin]
//     [sin  cos] [dy]  =  [dx*sin + dy*cos]
//   The minus sign flips the lateral component, causing drift in the wrong direction.
//   In this specific simulation dy=0, so this bug is HIDDEN during normal forward driving.
//   It only manifests when there's lateral slip. Sneaky.
//
// BUG 2 (Line ~79): Logical operator error in range validation
//   `if (r > scan.range_min || r < scan.range_max)` is always true for finite r
//   Should be: `if (r > scan.range_min && r < scan.range_max)`
//   This means ALL readings pass the filter, including NaN/inf/negative values.
//   AI should catch this easily -- it's a classic boolean logic error.
//
// BUG 3 (Line ~101-102): Missing heading in local-to-world transform
//   `double world_x = current_pose_.x + r * std::cos(angle);`
//   Should be: `double world_x = current_pose_.x + r * std::cos(angle + current_pose_.theta);`
//   Same for y. Without adding the robot's heading, the scan points are always
//   projected as if the robot faces east, regardless of actual heading.
//   AI tools are moderate at catching this -- it requires understanding coordinate frames.
//
// BUG 4 (Line ~113): Data race on `running_` bool
//   `bool running_` is read by isRunning() from two threads without synchronization.
//   Should be `std::atomic<bool> running_`.
//   This is technically UB per the C++ standard. In practice it "works" on x86 due to
//   strong memory ordering, but it's still wrong and can fail on ARM.
//   Many AI tools catch this; it's a well-known pattern.
//
// EXPECTED AI BEHAVIOR:
//   - BUG 2 (|| vs &&): AI catches this 90%+ of the time
//   - BUG 4 (non-atomic bool): AI catches this 80%+ of the time
//   - BUG 3 (missing heading): AI catches this ~60% of the time -- requires domain knowledge
//   - BUG 1 (rotation matrix sign): AI catches this ~40% -- it's hidden because dy=0 in simulation
//     and requires understanding the 2D rotation matrix convention
