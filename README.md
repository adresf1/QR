# QR
Great! Since you're on Windows, here’s a clear guide to set up a QR code scanner in C++ using OpenCV and ZBar on your system.

🧰 What You Need on Windows
1. Install Prerequisites
CMake – https://cmake.org/download/

MinGW or Visual Studio – You need a C++ compiler.

OpenCV for Windows – Download prebuilt: https://github.com/opencv/opencv/releases

ZBar for Windows – You will likely need to build it manually using CMake.

✅ Step-by-Step Setup
🔹 Step 1: Install OpenCV
Download the OpenCV Windows release from opencv.org.

Extract to C:\opencv.

Add C:\opencv\build\x64\vc16\bin to your System PATH.

🔹 Step 2: Build ZBar with CMake
ZBar doesn’t have official precompiled binaries for Windows, but you can build it:

Clone the ZBar repo:

bash
Kopiér
Rediger
git clone https://github.com/mchehab/zbar.git
Use CMake GUI or terminal:

bash
Kopiér
Rediger
mkdir zbar_build
cd zbar_build
cmake .. -G "Visual Studio 16 2019" -DBUILD_ZBAR_QT=OFF -DWITH_X=OFF -DWITH_GTK=OFF -DBUILD_SHARED_LIBS=ON
cmake --build . --config Release
After build:

.lib and .dll files will be in zbar_build/Release/

Include ZBar headers in your project from the zbar/include folder

🔹 Step 3: Write the Code
Save this file as qr_scanner.cpp:

cpp
Kopiér
Rediger
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Cannot open camera\n";
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
        if (cv::waitKey(1) == 27) break; // ESC key to exit
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
🔹 Step 4: Compile It
If you're using MinGW and OpenCV is at C:\opencv, compile it like this (adjust paths):

bash
Kopiér
Rediger
g++ qr_scanner.cpp -o qr_scanner -IC:\opencv\build\include -LC:\opencv\build\x64\vc16\lib -lopencv_world480 -lzbar
If you're using Visual Studio, create a new project, add the .cpp file, and configure:

C++ include directories (OpenCV + ZBar headers)

Linker input: .lib files

Runtime: include zbar.dll in your working directory

🧪 Test It
Run the app.

Show a QR code to your webcam (you can use your phone to display one).

The decoded text should appear in the console and be overlaid on the video.

Want Easier Setup?
If building ZBar is too complicated, you can use ZXing-C++, which is easier to set up with CMake and doesn’t require external dependencies like GTK or Qt.

Let me know if you want a version using ZXing-C++ instead.


📄 CMakeLists.txt
cmake
Kopiér
Rediger
cmake_minimum_required(VERSION 3.10)
project(QRScanner)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/libs/include
)

# Link directories
link_directories(
    ${PROJECT_SOURCE_DIR}/libs/lib
)

# Source file
add_executable(QRScanner main.cpp)

# Link with OpenCV and ZBar
target_link_libraries(QRScanner
    opencv_world480
    zbar
)






Spørg ChatGPT
