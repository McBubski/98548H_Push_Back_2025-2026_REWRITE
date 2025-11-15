#include "RAT/path_generator.h"

#include <cmath>

// Function to generate a path

Path PathGenerator::GeneratePath(
    std::initializer_list<std::initializer_list<double>> rawPoints,
    double _maxVelocity,
    double _maxAcceleration ,
    double spacing,
    double smoothing,
    double curveSlowdown
) {
    Path path; // Final output path

    path.maxVelocity = _maxVelocity;
    path.maxAcceleration = _maxAcceleration;

    std::vector<Waypoint> basePoints; // Initial unmodified points

    // Convert from simple list to a vector of Waypoints

    for (auto& p : rawPoints) {
        auto it = p.begin();
        double x = *it++;
        double y = *it;
        basePoints.emplace_back(x, y);
    }

    // Creates a path consisting of waypoints

    for (size_t i = 0; i < basePoints.size(); i++) {
        auto& bp = basePoints[i];

        path.AddWaypoint(bp.x, bp.y);
    }

    // Interpolate path
    path = InterpolatePath(path, spacing);

    // Smooth path
    path = SmoothPath(path, 1 - smoothing, smoothing, 0.001);

    // Calculate each point's distance along the path
    path = ComputeDistancesAlongPath(path);

    // Calculate each point's curvature along the path
    path = ComputeCurvatureAlongPath(path);

    // Calculate each point's target velocity, the final step
    
    // (2 - 5) Lower is higher slowdown at curves, higher is faster
    path = ComputeTargetVelocityAlongPath(path, curveSlowdown, _maxVelocity, _maxAcceleration);

    return path;
}

// Injects points into a path, given a desired spacing.
Path PathGenerator::InterpolatePath(const Path& inputPath, double spacing) {
    // Initialize new path
    Path interpolatedPath;

    // Can't inject without at least two points
    if (inputPath.size() < 2) {
        return inputPath;
    }

    // Runs through each segment and calculates how many waypoints fit
    for (size_t i = 0; i < inputPath.size() - 1; i++) {
        const auto& startPoint = inputPath.waypoints[i];
        const auto& endPoint = inputPath.waypoints[i + 1];

        // Calculates the length of the path segment
        double dx = endPoint.x - startPoint.x;
        double dy = endPoint.y - startPoint.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Determine how many waypoints to interject (length / spacing)
        int pointsThatFit = static_cast<int>(std::ceil(distance / spacing));

        // Calculate normalized direction to add points
        double dirX = dx / distance;
        double dirY = dy / distance;

        // Inject # of waypoints that fit, spaced at the correct spacing
        for (int j = 0; j < pointsThatFit; j++) {
            double newX = startPoint.x + dirX * spacing * j;
            double newY = startPoint.y + dirY * spacing * j;

          interpolatedPath.AddWaypoint(newX, newY, 0, 0);
        }

    }

    // Return new path
    return interpolatedPath; 
};

// Smoothes out the interpolated path, given some parameters for smoothing

Path PathGenerator::SmoothPath(const Path& inputPath, double a, double b, double tolerance) {
    Path clonedPath = inputPath;
    double change = tolerance;

    // Repeatedly iterate on the path until the points converge on a desired curvature within a certain tolerance

    while (change >= tolerance) {
        change = 0.0;

        for (int i = 1; i < inputPath.size() - 1; i++) {
            double clonedPathX = clonedPath.waypoints[i].x;
            double clonedPathY = clonedPath.waypoints[i].y;

            double previousNewClonedX = clonedPath.waypoints[i - 1].x;
            double previousNewClonedY = clonedPath.waypoints[i - 1].y;

            double nextNewClonedX = clonedPath.waypoints[i + 1].x;
            double nextNewClonedY = clonedPath.waypoints[i + 1].y;

            double inputPathX = inputPath[i].x;
            double inputPathY = inputPath[i].y;

            double aux = clonedPathX;
            clonedPath.waypoints[i].x += a * (inputPathX - clonedPathX) + b * (previousNewClonedX + nextNewClonedX - (2.0 * clonedPathX));
            change += std::abs(aux - clonedPath[i].x);

            aux = clonedPathY;
            clonedPath.waypoints[i].y += a * (inputPathY - clonedPathY) + b * (previousNewClonedY + nextNewClonedY - (2.0 * clonedPathY));
            change += std::abs(aux - clonedPath[i].y);

        }
    }

    return clonedPath;
}

// Computes how far along the path each waypoint is

Path PathGenerator::ComputeDistancesAlongPath(const Path& inputPath) {
    Path newPath = inputPath;

    if (inputPath.size() == 0) return inputPath;

    double runningDistanceTotal = 0.0;
    newPath.waypoints[0].distanceAlongPath = 0.0;

    for (size_t i = 1; i < inputPath.size(); i++) {
        double dx = inputPath.waypoints[i].x - inputPath.waypoints[i - 1].x;
        double dy = inputPath.waypoints[i].y - inputPath.waypoints[i - 1].y;
        double segmentLength = std::sqrt(dx * dx + dy * dy);

        runningDistanceTotal += segmentLength;
        newPath.waypoints[i].distanceAlongPath = runningDistanceTotal;
    }

    return newPath;
}

// Calculates the curvature at each waypoint along the path.
// This is done by calculating the curvature of a circle that intersects the point and adjacent points.

Path PathGenerator::ComputeCurvatureAlongPath(const Path& inputPath) {
    Path newPath = inputPath;

    for (size_t i = 1; i < inputPath.size() - 1; i++) {
        double x1 = inputPath.waypoints[i].x + 0.0001; // Prevents divide by 0 error
        double y1 = inputPath.waypoints[i].y;

        double x2 = inputPath.waypoints[i - 1].x;
        double y2 = inputPath.waypoints[i - 1].y;

        double x3 = inputPath.waypoints[i + 1].x;
        double y3 = inputPath.waypoints[i + 1].y;

        double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
        double k2 = (y1 - y2) / (x1 - x2);
        double b = 0.5 * ((x2 * x2) - (2 * x2 * k1) + (y2 * y2) - (x3 * x3) + (2 * x3 * k1) - (y3 * y3)) / ((x3 * k2) - y3 + y2 - (x2 * k2));

        double a = k1 - k2 * b;

        double r = std::sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));

        newPath.waypoints[i].curvature = 1.0 / r;
    }

    return newPath;
}

// Steps through and computes a target velocity, with slowdown for curvature and deceleration near the end.

Path PathGenerator::ComputeTargetVelocityAlongPath(const Path& inputPath, double curveSlowdown, double maxVelocity, double maxAcceleration) {
    Path newPath = inputPath;

    for (size_t i = 0; i < inputPath.size(); i++) {
        newPath.waypoints[i].targetVelocity = std::min(maxVelocity, curveSlowdown / inputPath.waypoints[i].curvature);
    }

    newPath.waypoints[inputPath.size() - 1].targetVelocity = 0;

    for (int i = (int)inputPath.size() - 2; i >= 0; i--) {
        double dx = newPath.waypoints[i + 1].x - newPath.waypoints[i].x;
        double dy = newPath.waypoints[i + 1].y - newPath.waypoints[i].y;
        double segmentLength = std::sqrt(dx * dx + dy * dy);

        double adjustedSpeed = std::sqrt((newPath.waypoints[i + 1].targetVelocity * newPath.waypoints[i + 1].targetVelocity) + 2 * maxAcceleration * segmentLength);
        newPath.waypoints[i].targetVelocity = std::min(newPath.waypoints[i].targetVelocity, adjustedSpeed);
    }

    newPath.waypoints[0].targetVelocity = maxVelocity;

    return newPath;
}