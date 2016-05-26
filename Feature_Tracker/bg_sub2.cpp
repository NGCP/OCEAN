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
//C
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;

// Global variables
Mat frame; //current frame
Mat fgMaskMOG2; //fg mask fg mask generated by MOG2 method
Ptr<BackgroundSubtractor> pMOG2; //MOG2 Background subtractor
int keyboard; //input from keyboard

//mine
Mat inverted; //inverted mask
Mat masked; //frame after mask
Mat gaussian; //mask w/ Gaussian blur
Mat gMasked; //gaussian masked image

/** Function Headers */
void help();
void processVideo(char* videoFilename);
void processImages(char* firstFrameFilename);

void help()
{
    cout
    << "--------------------------------------------------------------------------" << endl
    << "This program shows how to use background subtraction methods provided by "  << endl
    << " OpenCV. You can process both videos (-vid) and images (-img)."             << endl
                                                                                    << endl
    << "Usage:"                                                                     << endl
    << "./bs {-vid <video filename>|-img <image filename>}"                         << endl
    << "for example: ./bs -vid video.avi"                                           << endl
    << "or: ./bs -img /data/images/1.png"                                           << endl
    << "--------------------------------------------------------------------------" << endl
    << endl;
}


// find out what type Mat is
string type2str(int type) {
   string r;

   uchar depth = type & CV_MAT_DEPTH_MASK;
   uchar chans = 1 + (type >> CV_CN_SHIFT);

   switch ( depth ) {
      case CV_8U:  r = "8U"; break;
      case CV_8S:  r = "8S"; break;
      case CV_16U: r = "16U"; break;
      case CV_16S: r = "16S"; break;
      case CV_32S: r = "32S"; break;
      case CV_32F: r = "32F"; break;
      case CV_64F: r = "64F"; break;
      default:     r = "User"; break;
   }

   r += "C";
   r += (chans+'0');

   return r;
}

/**
 * @function processVideo
 */
void processVideo(char* videoFilename) {
    int col, row;
    //create the capture object
    VideoCapture capture(videoFilename);
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
        //update the background model
        pMOG2->apply(frame, fgMaskMOG2);
        //get the frame number and write it on the current frame
        stringstream ss;
        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
                  cv::Scalar(255,255,255), -1);
        ss << capture.get(CAP_PROP_POS_FRAMES);
        string frameNumberString = ss.str();
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
        //show the current frame and the fg masks
        imshow("Frame", frame);
        imshow("FG Mask MOG 2", fgMaskMOG2);

    //image closing
    Mat morph = Mat::zeros(fgMaskMOG2.rows, fgMaskMOG2.cols, CV_8UC1);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(fgMaskMOG2, morph, MORPH_CLOSE, element, Point(-1,-1), 5);
    imshow("closin", morph);




        //image dilation
/*        void Dilation( int, void* )
//        {
            Mat dilation = Mat::zeros(fgMaskMOG2.rows, fgMaskMOG2.cols, CV_8UC1);
            int dilation_type = MORPH_ELLIPSE;
            if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
                else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
                else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

            Mat element = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                 Point( dilation_size, dilation_size ) );*/
/*            Mat element = getStructuringElement( dilation_type, Size(7, 7));
            /// Apply the dilation operation
            dilate( fgMaskMOG2, dilation, element );
            imshow( "Dilation", dilation );
//        }*/
        
        
        //invert fg mask
        inverted = Mat::zeros(fgMaskMOG2.rows, fgMaskMOG2.cols, CV_8UC1);
//        bitwise_not(fgMaskMOG2, inverted);
//        bitwise_not(dilation, inverted);
        bitwise_not(morph, inverted);
        imshow("inverted mask", inverted);

        //gaussian blur on mask
        gaussian = Mat::zeros(inverted.rows, inverted.cols, CV_8UC1);
//        GaussianBlur(inverted, gaussian, Size(7, 7), 0, 0);
        //blur it some more b/c changing sigma vals made some parts more visible?
        /*GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);
        GaussianBlur(gaussian, gaussian, Size(7, 7), 0, 0);*/

        //image opening
        //suposed to be closing
        /*Mat kernel;
        Mat opening;
        kernel = getStructuringElement(CV_SHAPE_CUSTOM, Size(5, 5), Point(-1,-1));
        morphologyEx(gaussian, opening, MORPH_OPEN, kernel);
        imshow("image opening", opening);*/


GaussianBlur(inverted, gaussian, Size(7, 7), 0, 0);

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
        imshow("threshold", gaussian);

        //mask original frame w/ inverted
        frame.copyTo(masked, inverted);
        imshow("masked frame", masked);

        //mask original w/ gaussian
        frame.copyTo(gMasked, gaussian);
        imshow("gaussian masked", gMasked);


        //get the input from the keyboard
        keyboard = waitKey( 30 );
    }
    //delete capture object
    capture.release();
}

/**
 * @function processImages
 */
void processImages(char* fistFrameFilename) {
    //read the first file of the sequence
    frame = imread(fistFrameFilename);
    if(frame.empty()){
        //error in opening the first image
        cerr << "Unable to open first image frame: " << fistFrameFilename << endl;
        exit(EXIT_FAILURE);
    }
    //current image filename
    string fn(fistFrameFilename);
    //read input data. ESC or 'q' for quitting
    while( (char)keyboard != 'q' && (char)keyboard != 27 ){
        //update the background model
        pMOG2->apply(frame, fgMaskMOG2);
        //get the frame number and write it on the current frame
        size_t index = fn.find_last_of("/");
        if(index == string::npos) {
            index = fn.find_last_of("\\");
        }
        size_t index2 = fn.find_last_of(".");
        string prefix = fn.substr(0,index+1);
        string suffix = fn.substr(index2);
        string frameNumberString = fn.substr(index+1, index2-index-1);
        istringstream iss(frameNumberString);
        int frameNumber = 0;
        iss >> frameNumber;
        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
                  cv::Scalar(255,255,255), -1);
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
        //show the current frame and the fg masks
        imshow("Frame", frame);
        imshow("FG Mask MOG 2", fgMaskMOG2);

        //try to mask original frame
        //imshow("masked frame", masked);

        //get the input from the keyboard
        keyboard = waitKey( 30 );
        //search for the next image in the sequence
        ostringstream oss;
        oss << (frameNumber + 1);
        string nextFrameNumberString = oss.str();
        string nextFrameFilename = prefix + nextFrameNumberString + suffix;
        //read the next frame
        frame = imread(nextFrameFilename);
        if(frame.empty()){
            //error in opening the next image in the sequence
            cerr << "Unable to open image frame: " << nextFrameFilename << endl;
            exit(EXIT_FAILURE);
        }
        //update the path of the current frame
        fn.assign(nextFrameFilename);
    }
}

/**
 * @function main
 */
int main(int argc, char* argv[])
{
    //print help information
    help();

    //check for the input parameter correctness
    if(argc != 3) {
        cerr <<"Incorret input list" << endl;
        cerr <<"exiting..." << endl;
        return EXIT_FAILURE;
    }

    //create GUI windows
    namedWindow("Frame");
    namedWindow("FG Mask MOG 2");

    //create Background Subtractor objects
    pMOG2 = createBackgroundSubtractorMOG2(); //MOG2 approach

    if(strcmp(argv[1], "-vid") == 0) {
        //input data coming from a video
        processVideo(argv[2]);
    }
    else if(strcmp(argv[1], "-img") == 0) {
        //input data coming from a sequence of images
        processImages(argv[2]);
    }
    else {
        //error in reading input parameters
        cerr <<"Please, check the input parameters." << endl;
        cerr <<"Exiting..." << endl;
        return EXIT_FAILURE;
    }
    /* find out Mat type and num channels
    string ty = type2str(fgMaskMOG2.type());
    printf("Matrix: %s %dx%d \n", ty.c_str(), fgMaskMOG2.cols, fgMaskMOG2.rows);
    ty = type2str(frame.type());
    printf("Matrix: %s %dx%d \n", ty.c_str(), frame.cols, frame.rows);

    cout << "frame channels: " << frame.channels() << endl;
    cout << "masked channels: " << masked.channels() << endl;
    */

    //destroy GUI windows
    destroyAllWindows();
    return EXIT_SUCCESS;
}
