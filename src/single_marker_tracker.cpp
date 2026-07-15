#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open the webcam." << std::endl;
        return -1;
    }

    std::cout << "Camera initialized. Press ESC to close the window." << std::endl;

    cv::Mat frame;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::DetectorParameters dectectorParams = cv::aruco::DetectorParameters();

    cv::aruco::ArucoDetector detector(dictionary,dectectorParams);

    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("camera_calibration.xml", cv::FileStorage::READ);

    if (!fs.isOpened()) {
        std::cerr << "Error: Could not find camera_calibration.xml. Make sure it is in your build folder!" << std::endl;
        return -1;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();

    std::cout << "Calibration loaded successfully." << std::endl;

    float markerLength = 0.094f;
    
    std::vector<cv::Point3f> markerObjPoints;
    markerObjPoints.push_back(cv::Point3f(-markerLength/2.0f,  markerLength/2.0f, 0.0f));
    markerObjPoints.push_back(cv::Point3f( markerLength/2.0f,  markerLength/2.0f, 0.0f));
    markerObjPoints.push_back(cv::Point3f( markerLength/2.0f, -markerLength/2.0f, 0.0f));
    markerObjPoints.push_back(cv::Point3f(-markerLength/2.0f, -markerLength/2.0f, 0.0f));

    while (true) {
        cap.read(frame);

        if (frame.empty()) {
            std::cerr << "Error: Camera returned a blank frame." << std::endl;
            break;
        }

        std::vector<int> markerId;
        std::vector<std::vector<cv::Point2f>> markerCord;

        detector.detectMarkers(frame, markerCord, markerId);
        if(!markerId.empty()){
            cv::aruco::drawDetectedMarkers(frame, markerCord, markerId);

            for (size_t i = 0; i < markerId.size(); i++) {
                cv::Mat rvec, tvec;
                cv::solvePnP(markerObjPoints, markerCord[i], cameraMatrix, distCoeffs, rvec, tvec);
                double distance_z = tvec.at<double>(2);
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.05f);
                std::cout << "Marker " << markerId[i] << " is " << distance_z << " meters away." << std::endl;
            }
            std::cout << "Marker's ID:" << markerId[0] << std::endl;
        }

        imshow("Amazon AMR Docking - Live Feed", frame);

        if (cv::waitKey(1) == 27) {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    
    return 0;
}