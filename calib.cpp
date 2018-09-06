//
// Created by Haotao Lai on 2018-09-05.
//            haotao.lai@gmail.com
//
// A Example explain how to use OpenCV to undistort an image
// and calculate the intrinsic and exterinsic parameters
//
//
// Implementation of the following link in C++ (original in Python):
// https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/
//       py_calib3d/py_calibration/py_calibration.html#calibration
//
//
// This piece of code cannot be used to any commercial activity
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

#define DEBUG true

using namespace cv;

using std::cout;
using std::endl;
using std::vector;
using std::string;


void calcChessboardCorners(int rows, int cols, vector<Point3f> &objPoints) {
    for (int i = 0; i < cols; ++i) {
        for (int j = 0; j < rows; ++j) {
            Point3f p(j, i, 0);
            objPoints.push_back(p);
        }
    }
    if (DEBUG) {
        cout << "prepared object points" << endl;
        for (const Point3f &p : objPoints) {
            cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << endl;
        }
        cout << "- - - - - - -" << endl;
    }
}


void runCalibration(const vector<Mat> &imgList,
                    vector<vector<Point3f>> &objPoints,
                    const vector<vector<Point2f>> &imgPoints
) {
    Mat cameraMatrix = Mat::eye(3, 3, CV_32F);
    Mat distCoeffs= Mat::zeros(8, 1, CV_32F);
    objPoints.resize(imgPoints.size(), objPoints[0]);
    Size imgSize(640, 480);
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objPoints, imgPoints, imgSize,
                    cameraMatrix, distCoeffs, rvecs, tvecs);
    if (DEBUG) {
        cout << "Camera Matrix" << endl;
        cout << cameraMatrix << endl;
        cout << "Distortion Coefficients" << endl;
        cout << distCoeffs << endl;
        cout << "Rotation Vector" << endl;
        for (const Mat &v : rvecs) {
            cout << v << endl;
        }
        cout << "Traslation Vector" << endl;
        for (const Mat &v : tvecs) {
            cout << v << endl;
        }
    }

    Rect roi;
    Mat newCameraMatrix
            = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize,
                                        1.0, imgSize, &roi);
    for (const Mat& originImg : imgList) {
        Mat fixedImg;
        undistort(originImg, fixedImg, cameraMatrix, distCoeffs, newCameraMatrix);
        imshow("original image", originImg);
        imshow("undistorted image", fixedImg(roi));
        waitKey(1000);
    }

    destroyAllWindows();
}

void loadTestingImgAndFindCorner (vector<string> &fileNameList,
                                  vector<Mat> &imageList,
                                  vector<vector<Point3f>> &objPoints,
                                  vector<vector<Point2f>> &imgPoints
) {
    const TermCriteria &criteria
        = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);

    const int ROWS = 7;
    const int COLS = 6;
    Size patternSize = Size(ROWS, COLS);

    calcChessboardCorners(ROWS, COLS, objPoints[0]);

    for (const string& fileName : fileNameList) {
        Mat img = imread(fileName);
        if (img.data == nullptr) {
            cout << "fail to open image: " << fileName << endl;
            exit(1);
        }
        Mat grayImg;
        cvtColor(img, grayImg, COLOR_BGR2GRAY);
        imageList.push_back(img);

        vector<Point2f> corners;

        if (findChessboardCorners(grayImg, patternSize, corners,
                                  CALIB_CB_ADAPTIVE_THRESH
                                  + CALIB_CB_NORMALIZE_IMAGE
                                  + CALIB_CB_FAST_CHECK)) {

            cornerSubPix(grayImg, corners, Size(11, 11), Size(-1, -1), criteria);
            imgPoints.push_back(corners);

            if (DEBUG) {
               drawChessboardCorners(img, patternSize, corners, true);
               imshow("img", img);
               waitKey(500);
            }
        }
    }

    destroyAllWindows();
}


void prepareFileName(vector<string> &fileNameList) {
    string prefix = "/Users/ERIC_LAI/CLionProjects/EricCV/calib/";
    string surfix = ".jpg";
    string tmp = "left0";
    string tmp1 = "left";
    for (int i = 1; i < 10; ++i) {
        fileNameList.push_back(prefix + tmp + std::to_string(i) + surfix);
    }
    for (int j = 11; j <= 14; ++j) {
        fileNameList.push_back(prefix + tmp1 + std::to_string(j) + surfix);
    }
    if (DEBUG) {
        cout << "file name list -> " << endl;
        for (const string &name : fileNameList) {
            cout << name << endl;
        }
        cout << "- - - - - - -" << endl;
    }
}

int main() {
    vector<string> fileNameList;
    vector<vector<Point3f>> objPoints(1);
    vector<vector<Point2f>> imgPoints;
    vector<Mat> imageList;
    
    prepareFileName(fileNameList);
    loadTestingImgAndFindCorner(fileNameList, imageList, objPoints, imgPoints);
    runCalibration(imageList, objPoints, imgPoints);

    return 0;
}