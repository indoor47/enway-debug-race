# ENWAY AI Workshop — Debug Race Exercises

Three C++ debugging exercises for the workshop hands-on session. Work in pairs.
Find the bugs manually, with AI, or both — then compare.

## Exercises

### Round 1 — Easy (15 min): Waypoint Follower
```bash
g++ -std=c++17 -o debug1 debug_race_easy.cpp -lm
./debug1
```
Autonomous sweeper route — 3 bugs in navigation logic.

### Round 2 — Medium (15 min): Sensor Fusion
```bash
g++ -std=c++17 -o debug2 debug_race_medium.cpp -lpthread
./debug2
```
LiDAR + odometry fusion — 4 bugs of varying subtlety.

### Round 3 — Hard (20 min): Lock-Free Ring Buffer
```bash
g++ -std=c++17 -O2 -o debug3 debug_race_hard.cpp -lpthread
./debug3
```
Real-time sensor pipeline — 4 bugs, some platform-specific (x86 vs ARM).

## Setup

Requires a C++17 compiler. On Ubuntu/Debian:
```bash
sudo apt install g++
```
On macOS:
```bash
xcode-select --install
```

## Goal

This is not a competition. Discover for yourself:
- What types of bugs does AI catch instantly?
- What requires your domain expertise?
- When is AI a distraction vs a force multiplier?
