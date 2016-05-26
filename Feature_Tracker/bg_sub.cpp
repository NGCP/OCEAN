/**
 * @file bg_sub.cpp
 * @brief Background subtraction tutorial sample code
 * @author Domenico D. Bloisi
 */

//opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/features2d/features2d.hpp>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>

#define MIN_FEATURES 30
#define FAST_THRESHOLD 100
#define FEATURE_POINT_SIZE 3
#define MAX_FRAMES 50

using namespace cv;
using namespace std;

// Global variables
Mat frame; //current frame
Mat oldframe;
Mat frame_gray;
Mat old_gray;
Mat warp_matrix;
Mat aligned;
Mat sub_gray;

Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
int keyboard; //input from keyboard

//mine
Mat inverted; //inverted mask
Mat masked; //frame after mask
Mat gaussian; //mask w/ Gaussian blur
Mat gMasked; //gaussian masked image

/* Matrix for Rigid Transform */
Mat rigid_transform;

/* Feature Point Vectors */
std::vector<KeyPoint> keypoints;
std::vector<Point2f> pts, tracked_pts, prev_pts, saved_pts;

/* Status and Error Flags for KLT Algorithm */
std::vector<uchar> status;
std::vector<float> err;

/* Pixed Distance Calculation Variables */
float sum_x, sum_y, dist_x = 0, dist_y = 0;

/** Function Headers */
void processVideo(char* videoFilename);

/**
 * @function processVideo
 */
void processVideo(char* videoFilename) {
    int col, row, frame_count = 0;
    //create the capture object
    VideoCapture capture(0);
    if(!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open video file: " << videoFilename << endl;
        exit(EXIT_FAILURE);
    }
    //read input data. ESC or 'q' for quitting
    while( (char)keyboard != 'q' && (char)keyboard != 27 ){
        //read the current frame
        if(!capture.read(frame)) {
            cerr << "Unable to read next frame." << endl;
            cerr << "Exiting..." << endl;
            exit(EXIT_FAILURE);
        }
        frame_count++;

        //update the background model
        pMOG2->apply(frame, fgMaskMOG2, 0.1);
       
        //show the current frame and the fg masks
        //imshow("Frame", frame);
        //imshow("FG Mask MOG 2", fgMaskMOG2);
        
        //invert fg mask
        inverted = Mat::zeros(fgMaskMOG2.rows, fgMaskMOG2.cols, CV_8UC1);
        bitwise_not(fgMaskMOG2, inverted);
        //imshow("inverted mask", inverted);

        //gaussian blur on mask
        gaussian = Mat::zeros(inverted.rows, inverted.cols, CV_8UC1);
        GaussianBlur(inverted, gaussian, Size(7, 7), 0, 0);
        //blur it some more b/c changing sigma vals made some parts more visible?
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);

        //threshold mask
        for(col = 0; col < gaussian.cols; col++)
        {
            for(row = 0; row < gaussian.rows; row++)
            {
                if(gaussian.at<uchar>(row, col) < 254)
                {
                    gaussian.at<uchar>(row, col) = 0;
                }
            }
        }
        //imshow("threshold", gaussian);

        //mask original frame w/ inverted
        frame.copyTo(masked, inverted);
        //imshow("masked frame", masked);

        //mask original w/ gaussian
        frame.copyTo(gMasked, gaussian);
        imshow("gaussian masked", gMasked);

        /* Convert Frame to Grayscale */
        cvtColor(gMasked, sub_gray, COLOR_BGR2GRAY);

        /* Check for Minimum Number of Features */
        if (tracked_pts.size() < MIN_FEATURES || frame_count == MAX_FRAMES) {
           /* Find New Features (FAST Algorithm) */
           FAST(sub_gray, keypoints, FAST_THRESHOLD, true);
           KeyPoint::convert(keypoints, pts);

           //goodFeaturesToTrack(frame_gray, pts, 50, 0.01, 0.1);

           /* Keep Track of Feature Points */
           tracked_pts = pts;
           frame_count = 0;

           printf("FAST: Found %d Features\n", (int)pts.size());
      }
      else {
         /* Track Points (KLT Algorithm) */
         calcOpticalFlowPyrLK(oldframe, sub_gray, tracked_pts, pts, status, err);

         /* Failed Point Tracking */
         if (countNonZero(status) < status.size() * 0.8) {
            printf("ERROR\n");
            tracked_pts.clear();
            oldframe.release();
            continue;
         }

         /* Save Old Points */
         prev_pts = tracked_pts;

         /* Clear Tracked Points */
         tracked_pts.clear();
         saved_pts.clear();

         /* Reset Sum Variables */
         //sum_x = 0;
         //sum_y = 0;

         /* Rebuild Tracked Points Without Failed Points */
         for (int i = 0; i < status.size(); i++) {
            if(status[i]) {
               tracked_pts.push_back(pts[i]);

               /* Calculate Sum of Movement */
               //sum_x += pts[i].x - prev_pts[i].x;
               //sum_y += pts[i].y - prev_pts[i].y;

               saved_pts.push_back(prev_pts[i]);
            }
         }

         if (saved_pts.size()) {
            rigid_transform = estimateRigidTransform(saved_pts, tracked_pts, false);

            if (!rigid_transform.empty()) {
               /* Calculate Average Movement in X and Y Directions */
               dist_x += rigid_transform.at<double>(0,2);
               dist_y += rigid_transform.at<double>(1,2);
            }

            //dist_x += sum_x / tracked_pts.size();
            //dist_y += sum_y / tracked_pts.size();
            printf("X: %8.3f        Y: %8.3f\n", dist_x, dist_y);
         }
      }

      /* Plot Feature Points */
      for (int i = 0; i < tracked_pts.size(); i++) {
         circle(gMasked, tracked_pts[i], FEATURE_POINT_SIZE, Scalar(0, 0, 255), CV_FILLED);
      }

      /* Save Old Frame */
      oldframe = sub_gray;

      /* Release Grayscale Frame */
      sub_gray.release();
    
      /* Display Frame in Window */
      imshow("Live View", gMasked);

        //get the input from the keyboard
        keyboard = waitKey( 30 );
    }
    //delete capture object
    capture.release();
}

/**
 * @function main
 */
int main(int argc, char* argv[])
{
    //check for the input parameter correctness
    if(argc != 3) {
        cerr <<"Incorret input list" << endl;
        cerr <<"exiting..." << endl;
        return EXIT_FAILURE;
    }

    //create Background Subtractor objects
    pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach

    if(strcmp(argv[1], "-vid") == 0) {
        //input data coming from a video
        processVideo(argv[2]);
    }
    else {
        //error in reading input parameters
        cerr <<"Please, check the input parameters." << endl;
        cerr <<"Exiting..." << endl;
        return EXIT_FAILURE;
    }

    //destroy GUI windows
    destroyAllWindows();
    return EXIT_SUCCESS;
}
