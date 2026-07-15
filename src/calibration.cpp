#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>

#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    cv::Size boardSize(9, 7);
    float squareSize = 0.02f;
    
    if (!cap.isOpened()) return -1;

    std::vector<cv::Point3f> obj;
    for(int i = 0; i < boardSize.height; i++) {
        for(int j = 0; j < boardSize.width; j++) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0.0f));
        }
    }

    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    cv::Mat frame, gray;
    std::vector<cv::Point2f> corners;

    std::cout << "Press 'C' to capture a frame. Capture 15 frames to calibrate." << std::endl;

    while (true) {
        cap.read(frame);

        if (frame.empty()) break;

        cv::flip(frame, frame, 1);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        bool found = cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        if (found) {
            cv::drawChessboardCorners(frame, boardSize, corners, found);
        }

        imshow("Calibrator", frame);

        char key = (char)cv::waitKey(1);
        if (key == 27) { 
            break;
        } 
        else if (key == 'c' || key == 'C') {
            if (found) {
                imagePoints.push_back(corners);
                objectPoints.push_back(obj);
                std::cout << "Captured frame! (" << imagePoints.size() << "/15)" << std::endl;
            } else {
                std::cout << "Cannot capture: Grid not fully visible." << std::endl;
            }
        }
    }
    

    cap.release();
    cv::destroyAllWindows();
    
    if (imagePoints.size() < 10) {
        std::cerr << "Not enough frames captured for a good calibration. (Need at least 10)" << std::endl;
        return -1;
    }

    std::cout << "Calculating Intrinsic Matrix... This might take a few seconds." << std::endl;

    // 5. The Output Variables
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs; // We don't need these for now, but the function requires them

    // 6. The Core Calibration Algorithm
    // This crunches the math on all your frames and calculates the matrix and lens distortion
    double rms = cv::calibrateCamera(objectPoints, imagePoints, gray.size(), 
                                     cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "\n=== CALIBRATION SUCCESSFUL ===" << std::endl;
    std::cout << "RMS Error: " << rms << std::endl;
    std::cout << "\nCamera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "\nDistortion Coefficients:\n" << distCoeffs << std::endl;

    // 7. Save the data to an XML file so our ArUco program can read it later
    cv::FileStorage fs("camera_calibration.xml", cv::FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    std::cout << "\nSaved to camera_calibration.xml" << std::endl;
    
    return 0;
}