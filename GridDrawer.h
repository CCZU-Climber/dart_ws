#ifndef GRIDDRAWER_H
#define GRIDDRAWER_H

#include <opencv2/opencv.hpp>

class GridDrawer {
public:
    static void drawGridLines(cv::Mat& image);
};

#endif