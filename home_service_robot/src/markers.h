#ifndef MARKERS_H
#define MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

const uint32_t MARKER_SHAPE = visualization_msgs::Marker::CUBE;
const double MARKER_SIZE = 0.2;

struct MarkerPosition {

    MarkerPosition(double x_init, double y_init) : x{x_init}, y{y_init} { }

    double x;
    double y;
    
};

class Cycler {

private:
    
    std::array<MarkerPosition, 2> positions;
    int current = 0;

public:

    Cycler(const MarkerPosition& p1, const MarkerPosition& p2) : positions{p1, p2} {}

    void next() {
        current = (current == 0) ? 1 : 0;
    }

    const MarkerPosition& getCurrentPosition() {
        return positions[current];
    }

};

visualization_msgs::Marker prepare_marker(const MarkerPosition &pos);

#endif