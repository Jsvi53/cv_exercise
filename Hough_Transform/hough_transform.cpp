#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat img = cv::imread("assets/hough_image.png");
    if (img.empty()) {
        std::cerr << "Image not found" << std::endl;
        return 1;
    }

    // Convert to gray
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // Edge detection
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150, 3);

    // Hough transform for line detection
    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 1, CV_PI/360, 1000); // Increase angular resolution

    // Find the longest line and compute its angle
    float max_len = 0;
    float best_theta = 0;
    for (const auto& line : lines) {
        float rho = line[0], theta = line[1];
        float a = cos(theta), b = sin(theta);
        float x0 = a*rho, y0 = b*rho;
        cv::Point pt1(cvRound(x0 + 1000*(-b)), cvRound(y0 + 1000*(a)));
        cv::Point pt2(cvRound(x0 - 1000*(-b)), cvRound(y0 - 1000*(a)));
        float len = cv::norm(pt1 - pt2);

        if (len > max_len) {
            max_len = len;
            best_theta = theta;
        }
    }

    // Calculate the needed angle to make the line horizontal
    double angle = best_theta * 180 / CV_PI - 90;
    if (angle > 90) angle -= 180;  // Correct angles greater than 90 degrees

    // Calculate rotated center
    cv::Point center(img.cols/2, img.rows/2);

    // Rotation matrix
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);

    // Rotate the image
    cv::Mat dst;
    cv::warpAffine(img, dst, rot_mat, img.size());

    // Show the images
    cv::imshow("Original Image", img);
    cv::imshow("Rotated Image", dst);
    cv::waitKey(0);

    return 0;
}
