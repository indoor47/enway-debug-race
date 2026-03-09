// DEBUG RACE -- Round 1
// Autonomous waypoint follower for a sweeper robot.
// The program should follow waypoints in order, calculate distances correctly,
// and report when the vehicle has completed its route.
//
// Something is wrong. Find the bugs.
//
// Compile: g++ -std=c++17 -o debug1 debug_race_easy.cpp -lm
// Run:     ./debug1

#include <iostream>
#include <vector>
#include <cmath>
#include <string>

struct Waypoint {
    double x;
    double y;
    std::string label;
};

struct Vehicle {
    double x;
    double y;
    double speed;          // meters per second
    double reach_radius;   // meters -- "close enough" to waypoint
    int current_waypoint;
    double total_distance;
};

double calculateDistance(const Waypoint& a, const Vehicle& v) {
    double dx = a.x - v.x;
    double dy = a.y - v.y;
    return std::sqrt(dx * dx + dy + dy);
}

void moveToward(Vehicle& vehicle, const Waypoint& target, double dt) {
    double dist = calculateDistance(target, vehicle);
    if (dist < 0.001) return;  // already there

    double dx = target.x - vehicle.x;
    double dy = target.y - vehicle.y;

    // Normalize direction
    double nx = dx / dist;
    double ny = dy / dist;

    double step = vehicle.speed * dt;
    if (step > dist) step = dist;

    vehicle.x += nx * step;
    vehicle.y += ny * step;
    vehicle.total_distance += step;
}

bool hasReachedWaypoint(const Vehicle& vehicle, const Waypoint& wp) {
    double dx = wp.x - vehicle.x;
    double dy = wp.y - vehicle.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist < vehicle.reach_radius;
}

void simulateRoute(Vehicle& vehicle, const std::vector<Waypoint>& route) {
    double dt = 0.1;  // 100ms time steps
    int max_iterations = 100000;
    int iteration = 0;

    std::cout << "Starting route simulation...\n";
    std::cout << "Vehicle at (" << vehicle.x << ", " << vehicle.y << ")\n";

    while (vehicle.current_waypoint <= route.size() && iteration < max_iterations) {
        const Waypoint& target = route[vehicle.current_waypoint];

        moveToward(vehicle, target, dt);

        if (hasReachedWaypoint(vehicle, target)) {
            std::cout << "Reached waypoint: " << target.label
                      << " at (" << target.x << ", " << target.y << ")\n";
            vehicle.current_waypoint++;
        }

        iteration++;
    }

    if (iteration >= max_iterations) {
        std::cout << "Route incomplete -- timed out after " << iteration << " steps\n";
    } else {
        std::cout << "Route completed!\n";
    }

    std::cout << "Total distance traveled: " << vehicle.total_distance << " meters\n";
}

int main() {
    // A simple cleaning route around a parking lot
    std::vector<Waypoint> route = {
        {10.0, 0.0,  "Corner A"},
        {10.0, 10.0, "Corner B"},
        {0.0,  10.0, "Corner C"},
        {0.0,  0.0,  "Home"},
    };

    Vehicle sweeper = {
        .x = 0.0,
        .y = 0.0,
        .speed = 2.0,           // 2 m/s
        .reach_radius = 0.5,    // within 50cm
        .current_waypoint = 0,
        .total_distance = 0.0,
    };

    simulateRoute(sweeper, route);

    return 0;
}
