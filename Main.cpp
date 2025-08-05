// main.cpp
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Kan ikke åbne kameraet.\n";
        return -1;
    }

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    while (true) {
        cv::Mat frame, gray;
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        zbar::Image image(gray.cols, gray.rows, "Y800", gray.data, gray.total());

        int n = scanner.scan(image);

        for (auto symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            std::string data = symbol->get_data();
            std::cout << "QR Code: " << data << std::endl;
            cv::putText(frame, data, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
        }

        cv::imshow("QR Scanner", frame);
        if (cv::waitKey(1) == 27) break; // ESC afslutter
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
