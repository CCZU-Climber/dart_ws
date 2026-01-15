#include "GridDrawer.h"
#include <string>

void GridDrawer::drawGridLines(cv::Mat& image) {
    int center_x = image.cols / 2;
    int center_y = image.rows / 2;
    
    cv::Scalar grid_color(255, 255, 0);
    int line_thickness = 1;
    int dash_length = 10;
    int gap_length = 5;
    
    for (int x = center_x + 50; x < image.cols; x += 50) {
        for (int y = 0; y < image.rows; y += dash_length + gap_length) {
            int y_end = std::min(y + dash_length, image.rows);
            cv::line(image, cv::Point(x, y), cv::Point(x, y_end), grid_color, line_thickness);
        }
        cv::putText(image, std::to_string(x - center_x), cv::Point(x, center_y - 15), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, grid_color, 1);
    }
    
    for (int x = center_x - 50; x >= 0; x -= 50) {
        for (int y = 0; y < image.rows; y += dash_length + gap_length) {
            int y_end = std::min(y + dash_length, image.rows);
            cv::line(image, cv::Point(x, y), cv::Point(x, y_end), grid_color, line_thickness);
        }
        cv::putText(image, std::to_string(x - center_x), cv::Point(x, center_y - 15), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, grid_color, 1);
    }
    
    for (int y = center_y + 50; y < image.rows; y += 50) {
        for (int x = 0; x < image.cols; x += dash_length + gap_length) {
            int x_end = std::min(x + dash_length, image.cols);
            cv::line(image, cv::Point(x, y), cv::Point(x_end, y), grid_color, line_thickness);
        }
        cv::putText(image, std::to_string(y - center_y), cv::Point(center_x + 10, y + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, grid_color, 1);
    }
    
    for (int y = center_y - 50; y >= 0; y -= 50) {
        for (int x = 0; x < image.cols; x += dash_length + gap_length) {
            int x_end = std::min(x + dash_length, image.cols);
            cv::line(image, cv::Point(x, y), cv::Point(x_end, y), grid_color, line_thickness);
        }
        cv::putText(image, std::to_string(y - center_y), cv::Point(center_x + 10, y + 5), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, grid_color, 1);
    }
    
    cv::Scalar center_color(0, 255, 255);
    int center_line_thickness = 2;
    
    cv::line(image, cv::Point(0, center_y), cv::Point(image.cols, center_y), 
             center_color, center_line_thickness);
    cv::line(image, cv::Point(center_x, 0), cv::Point(center_x, image.rows), 
             center_color, center_line_thickness);
    
    cv::circle(image, cv::Point(center_x, center_y), 8, center_color, 2);
    cv::circle(image, cv::Point(center_x, center_y), 4, cv::Scalar(0, 0, 255), -1);
    
    std::string center_coord = "Center: (" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")";
    cv::putText(image, center_coord, cv::Point(10, image.rows - 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    std::string grid_info = "Grid: 50px spacing";
    cv::putText(image, grid_info, cv::Point(10, image.rows - 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    
    cv::putText(image, "→ X", cv::Point(image.cols - 40, center_y - 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, grid_color, 1);
    cv::putText(image, "↓ Y", cv::Point(center_x + 10, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, grid_color, 1);
}