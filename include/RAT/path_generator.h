#pragma once

#include "vex.h"

#include <initializer_list>
#include <functional>
#include <vector>

// Waypoint Structure

struct Waypoint {
    double x;
    double y;

    double targetVelocity;
    double curvature;
    double distanceAlongPath;

    std::function<void()> onReach;

    // Waypoint(x, y, velocity, curvature)
    Waypoint(double x_, double y_)
        : x(x_), y(y_), targetVelocity(0), curvature(0), distanceAlongPath(0) {}
};

// Path class

class Path {
    public:
        std::vector<Waypoint> waypoints;
        double maxVelocity = 50.0;
        double maxAcceleration = 50.0;

        // Add points to path
        void AddWaypoint(double x, double y, double targetVelocity = 0, double curvature = 0) {
            waypoints.emplace_back(x, y);
            waypoints.back().targetVelocity = targetVelocity;
            waypoints.back().curvature = curvature;
        }

        // Returns point count
        size_t size() const {return waypoints.size();}

        // Allows access with [] syntax, for easiness
       const Waypoint& operator[](size_t index) const {return waypoints[index];}
};

// Path generator

class PathGenerator {
    public:
        static Path GeneratePath(
            std::initializer_list<std::initializer_list<double>> rawPoints,
            double _maxVelocity,
            double _maxAcceleration,
            double spacing = 6.0,
            double smoothing = 0.8,
            double curveSlowdown = 3.0
        );

    private:
        // Path Generation Functions
        static Path InterpolatePath(const Path& inputPath, double spacing);
        static Path SmoothPath(const Path& inputPath, double a, double b, double tolerance);

        // Target Drive Velocity Functions
        static Path ComputeDistancesAlongPath(const Path& inputPath);
        static Path ComputeCurvatureAlongPath(const Path& inputPath);
        static Path ComputeTargetVelocityAlongPath(const Path& inputPath, double curveSlowdown, double maxVelocity, double maxAcceleration);
};