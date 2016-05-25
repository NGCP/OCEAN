//opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

int main() {
  VideoCapture cam(0);
  Mat frame;
  Mat old_frame;
  Mat frame_gray;
  Mat old_gray;

  int test = 1;
// Read the images to be aligned
  Mat im1 = imread("1.png");
  Mat im2 = imread("2.png");
  imshow("img1", im1); 
  // Convert images to gray scale;
  Mat im1_gray, im2_gray;

  while (1) {
    cam >> frame;

    imshow("blah", frame);

    if (test) {
      test = 0;
      old_frame = frame;
    }

    cvtColor(old_frame, old_gray, CV_BGR2GRAY);
    cvtColor(frame, frame_gray, CV_BGR2GRAY);
     
    // Define the motion model
    const int warp_mode = MOTION_EUCLIDEAN;
     
    // Set a 2x3 or 3x3 warp matrix depending on the motion model.
    Mat warp_matrix;
     
    // Initialize the matrix to identity
    if ( warp_mode == MOTION_HOMOGRAPHY )
        warp_matrix = Mat::eye(3, 3, CV_32F);
    else
        warp_matrix = Mat::eye(2, 3, CV_32F);
     
    // Specify the number of iterations.
    int number_of_iterations = 5000;
     
    // Specify the threshold of the increment
    // in the correlation coefficient between two iterations
    double termination_eps = 1e-10;
     
    // Define termination criteria
    TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);
     
    // Run the ECC algorithm. The results are stored in warp_matrix.
    findTransformECC(old_gray, frame_gray, warp_matrix, warp_mode, criteria);
     
    // Storage for warped image.
    Mat im2_aligned;
     
    if (warp_mode != MOTION_HOMOGRAPHY)
        // Use warpAffine for Translation, Euclidean and Affine
        warpAffine(frame, im2_aligned, warp_matrix, old_frame.size(), INTER_LINEAR + WARP_INVERSE_MAP);
    else
        // Use warpPerspective for Homography
        warpPerspective (frame, im2_aligned, warp_matrix, old_frame.size(),INTER_LINEAR + WARP_INVERSE_MAP);
     
    // Show final result
    imshow("Image 1", old_frame);
    imshow("Image 2", frame);
    imshow("Image 2 Aligned", im2_aligned);
    //waitKey(0);
  }

  return(0);
}
