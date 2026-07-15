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

    float markerLength = 0.015f;
    float markerSeparation = 0.004f;

    cv::Size boardDimensions(5,7);
    cv::aruco::GridBoard board(boardDimensions, markerLength, markerSeparation, dictionary);

    float boardWidth = boardDimensions.width * markerLength + (boardDimensions.width - 1) * markerSeparation;
    float boardHeight = boardDimensions.height * markerLength + (boardDimensions.height - 1) * markerSeparation;
    float centerX = boardWidth / 2.0f;
    float centerY = boardHeight / 2.0f;

    cv::KalmanFilter kf(6, 3, 0);
    kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 1,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    cv::setIdentity(kf.measurementMatrix);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-2));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    bool kf_init = false;



    while (true) {
        cap.read(frame);

        if (frame.empty())  break;

        std::vector<int> markerId;
        std::vector<std::vector<cv::Point2f>> markerCord;

        detector.detectMarkers(frame, markerCord, markerId);
        if(!markerId.empty()){
            //cv::aruco::drawDetectedMarkers(frame, markerCord, markerId);
            std::vector<cv::Point3f> objPoints;
            std::vector<cv::Point2f> imgPoints;
            
            board.matchImagePoints(markerCord,markerId, objPoints,imgPoints);
            if(objPoints.size()>=4)
            {
                for (auto& pt : objPoints) {
                    pt.x -= centerX;
                    pt.y -= centerY;
                }
                cv::Mat rvec, tvec;
                cv::solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);

                cv::Mat prediction = kf.predict();
                cv::Mat measurement = cv::Mat::zeros(3, 1, CV_32F);
                measurement.at<float>(0) = static_cast<float>(tvec.at<double>(0));
                measurement.at<float>(1) = static_cast<float>(tvec.at<double>(1));
                measurement.at<float>(2) = static_cast<float>(tvec.at<double>(2));

                if (!kf_init) {
                    measurement.copyTo(kf.statePost(cv::Rect(0, 0, 1, 3)));
                    kf_init = true;
                } else {
                    cv::Mat estimated = kf.correct(measurement);

                    tvec.at<double>(0) = estimated.at<float>(0);
                    tvec.at<double>(1) = estimated.at<float>(1);
                    tvec.at<double>(2) = estimated.at<float>(2);
                }

                double dist_z = tvec.at<double>(2);
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.05f);

                std::cout << "Box Center is " << dist_z << " meters away." << std::endl;
            }
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