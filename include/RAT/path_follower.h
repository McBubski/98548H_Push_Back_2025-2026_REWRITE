#pragma once

#include "RAT/path_generator.h"
#include "vex.h"

void FollowPath(Path& path, vex::directionType direction = vex::directionType::fwd, double lookaheadDistance = 24.0);
extern bool RAT_Interrupt;

// Utility

int FindClosestPoint(Path& path, int lastClosestIndex);
std::pair<Waypoint, double> FindLookaheadPoint(const Path& path, int lastLookaheadIndex, double lookaheadDistance);
double ComputeCurvature(const Waypoint& lookaheadPoint, double lookaheadDistance);

std::vector<double> CircleLineIntersection(double r, const Waypoint& E, const Waypoint& L);
Waypoint InterpolateSegment(const Waypoint& start, const Waypoint& end, double t);