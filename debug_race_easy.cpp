// DEBUG RACE -- Exercise 1: EASY
// Bug difficulty: Medium-Easy
// Domain: Autonomous vehicle waypoint follower
//
// INSTRUCTIONS:
// This program simulates a simple waypoint follower for an autonomous sweeper.
// It has 3 bugs. Find them -- manually or with AI.
// The program should: follow waypoints in order, calculate distances correctly,
// and report when the vehicle has completed its route.
//
// Compile: g++ -std=c++17 -o debug1 debug_race_easy.cpp -lm
// Run: ./debug1

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
    // BUG 1: Using integer division instead of floating point
    // sqrt of int division truncates intermediate results
    return std::sqrt(dx * dx + dy + dy);  // BUG: should be dy * dy, not dy + dy
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

    // BUG 2: Off-by-one -- should be < route.size(), but also current_waypoint
    // starts at 0 and the loop condition uses <= which goes one past the end
    while (vehicle.current_waypoint <= route.size() && iteration < max_iterations) {
        const Waypoint& target = route[vehicle.current_waypoint];  // Can access out of bounds!

        moveToward(vehicle, target, dt);

        if (hasReachedWaypoint(vehicle, target)) {
            std::cout << "Reached waypoint: " << target.label
                      << " at (" << target.x << ", " << target.y << ")\n";
            vehicle.current_waypoint++;
        }

        iteration++;
    }

    // BUG 3: Wrong comparison -- checks if we iterated too many times
    // but doesn't check if we actually completed the route
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

// === ANSWER KEY (for facilitator) ===
//
// BUG 1 (Line ~35): Distance calculation is wrong
//   `dy + dy` should be `dy * dy`
//   This makes the distance formula incorrect: sqrt(dx^2 + 2*dy) instead of sqrt(dx^2 + dy^2)
//   This causes incorrect movement vectors and weird diagonal paths
//
// BUG 2 (Line ~63): Off-by-one / out-of-bounds access
//   `vehicle.current_waypoint <= route.size()` should be `<`
//   When current_waypoint equals route.size(), accessing route[current_waypoint]
//   is undefined behavior (reads past the vector). This is a classic C++ UB bug.
//   AI tools should catch this easily.
//
// BUG 3 (Line ~63 + 73): The route "completes" via UB
//   The while loop exits either by timeout or by accessing past the end.
//   In the "success" case, it actually triggered UB first.
//   The completion check should verify current_waypoint == route.size() explicitly.
//
// EXPECTED AI BEHAVIOR:
//   - AI should catch BUG 1 (arithmetic error) easily
//   - AI should catch BUG 2 (off-by-one) easily -- this is a classic
//   - BUG 3 is more subtle -- it's a logic error in the control flow
//   - Good AI tools might also note that calculateDistance() and hasReachedWaypoint()
//     compute distance differently (one is broken, one isn't), which is a code smell
