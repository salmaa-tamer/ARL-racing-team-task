#include "safety_monitor.hpp"

#include <cmath>

namespace arl {
namespace {

bool isFinite(const Detection& detection) {
    return std::isfinite(detection.forward)
        && std::isfinite(detection.left)
        && std::isfinite(detection.confidence);
}

}  // namespace

std::vector<Obstacle> processDetections(
    const std::vector<Detection>& detections,
    const RoverPose& pose,
    const SafetyConfig& config) {
    std::vector<Obstacle> obstacles;
    const double headingRadians = (pose.headingDegrees) * (3.14159/180.0); //changed degrees to radians
    const double cosine = std::cos(headingRadians);
    const double sine = std::sin(headingRadians);

    for (std::size_t index = 0; index  < detections.size(); ++index) { //changed index condition
        const auto& detection = detections[index];
        const double range = std::hypot(detection.forward, detection.left);
        const bool validConfidence = detection.confidence >= config.minimumConfidence
            && detection.confidence <= 1.0;  //changed condition to be more than minimum confidence and less than 1.0
        const bool validRange = range > 0.0 && range <= config.maximumRangeMeters;

        if (!isFinite(detection) || !validConfidence || !validRange) {
            continue;
        }

        obstacles.push_back({
            detection.id,
            detection.forward,
            detection.left,
            pose.worldX + cosine * detection.forward - sine * detection.left, // made it subtract the position as the car's left is opposite to correct x-axis direction
            pose.worldY + sine * detection.forward + cosine * detection.left, 
            range,
        });
    }

    return obstacles;
}

std::optional<Obstacle> findNearestObstacle(const std::vector<Obstacle>& obstacles) {
    if (obstacles.empty()) {
        return std::nullopt;
    }

    const Obstacle* nearest = &obstacles.front();
    for (const auto& obstacle : obstacles) {
        if (obstacle.range < nearest->range) { // nearest object should have least distance
            nearest = &obstacle;
        }
    }

    return *nearest;
}

double calculateStoppingDistance(double speedKph, const SafetyConfig& config) {
    const double speedMps = speedKph * (5.0/18.0);  // Converted km/h to m/s
    const double reactionDistance = speedMps * config.reactionTimeSeconds;
    const double brakingDistance = speedMps * speedMps
        / (2.0 * config.maximumDecelerationMps2);
    return reactionDistance + brakingDistance;
}

bool shouldEmergencyBrake(const std::vector<Obstacle>& obstacles, double speedKph, const SafetyConfig& config) {
    if (obstacles.empty()) // changed it from always returning false, to checking if there is an obstacle, if so, checking range vs stopping distance,and braking if the rage is less
    return false;
    else{ 
        const double stoppingDistance = calculateStoppingDistance(speedKph, config);
        const auto nearestObs = findNearestObstacle(obstacles);
     nearestObs->range <= stoppingDistance; 
     return true;
        }
        return false;
}

}  // namespace arl
