//COMMAND LINE :


//g++-6 sc_facedetect.cpp sc_usb_serial_comm.cpp -lopencv_core -lopencv_imgproc -lopencv_objdetect -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -std=c++17 -lpthread  -o a.out


#define BOARD RASPBERRY_PI

#include <errno.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <stdlib.h>
#include <set>
#include <functional>
#include <memory>
#include <string>
#include <future>


#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <linux/i2c-dev.h>

#include "sc_ObjectTracking.hpp"
#include "sc_Robot.hpp"
#include "opencv2/core/core_c.h"

using namespace std;
using namespace USB_COMM;


#define FRAME_WIDTH_PIX 320
#define FRAME_HEIGHT_PIX 240
#define FRAME_RATE 5 //of the camera

#define UPDATE_ROBOT_FREQ 2 //allow to consecutive detections to avoid false positive
#define FRAME_COORDINATES_WIDTH_PIX FRAME_WIDTH_PIX

#define SENSOR_IMAGE_WIDTH_MM 3.68f
#define PIX_PER_MM_RATIO ( FRAME_WIDTH_PIX / SENSOR_IMAGE_WIDTH_MM)
#define MM_PER_PIX_RATIO ( SENSOR_IMAGE_WIDTH_MM / FRAME_WIDTH_PIX)

#define FOCAL_LENGTH 3.04f //mm

#define RADIUS_REFERENCE 30

#define START_MSG_BYTE 0x7B
#define END_MSG_BYTE   0x7D
#define ESCAPE_BYTE    0x7E
#define UNESCAPE_BYTE(x) (0xFF & (0x20 ^ (x)))
#define MSG_FROM_ROBOT_MAX_LENGTH 32

#define USER_ENABLED true
#define COMM_WITH_ROBOT true
#define PROCESS_NESTED false

#define NUM_OF_REC_POS 8


vector<pair<int,int>> vpos( NUM_OF_REC_POS, pair<int,int>(0,0) );
int lastPopulatedIndex = -1;

//Detect Object and pushes message to Queue - Robot will read from queue and process the data
void TrackObject( unique_ptr<Camera>& camp,
                  unique_ptr<Robot>& robp,
                  VideoCapture& capture, CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
                  double scale, bool tryflip,
                  CriticalQueue<string>& ccQ,
                  CriticalQueue<Mat>& cuQ)
{
    static int32_t debugCount = 0;
    double timeStamp = 0;

    int frameIndex = 0;
    Mat frame, frame1;
    int xErrorPix = 0;
    int zError = 0;
    int angle = 0;
    int deviation = 0;

    cout << "Entered TrackObject Thread" ;

    while(1)
    {
        capture >> frame;
        if( frame.empty() )
        {
            cout << "Frame is empty, quitting!";
            continue;
        }

        timeStamp = (double)cvGetTickCount();

        uint32_t estimateDistance = robp->GetTargetDistance(); //in cm

        ++camp->mNumOfTrials;
        pair<bool,bool> objectState = camp->detectAndDraw( frame, cascade, nestedCascade, scale, false, xErrorPix, zError, angle );

        timeStamp = (double)cvGetTickCount() - timeStamp;
        uint32_t detectTime = static_cast<uint32_t>(timeStamp/((double)cvGetTickFrequency()*1000.0));

        //if object detected
        if (objectState.first == true )
        {
            cout <<"detection time = " <<  detectTime <<
                    "ms - ObjectState = " << objectState.first << " " << objectState.second << endl;

            stringstream str_stream;
            //if motion detected update the robot
            //if (objectState.second == true)
            {
                 //compute the error on the x-axis in the real world
                double xErrorMM = xErrorPix  * MM_PER_PIX_RATIO;
                int xRealErrorMM = xErrorMM * estimateDistance * 10.0f / FOCAL_LENGTH;

                deviation = static_cast<int>(floor(atan(xRealErrorMM/10.0f / estimateDistance) * 180.0f / 3.1416f));
                //cout << "\n xRealErrorMM = " << xRealErrorMM << " mm   estimateDistance = " << estimateDistance << " cm deviation =" << deviation <<endl;
                str_stream << xRealErrorMM << " " << ++debugCount << " " << static_cast<int>(floor(deviation));

            }
            //else
            //{
                //cout << "\n xErrorMM = " << 0 << " mm   estimateDistance = " << estimateDistance << " cm deviation =" << 0 <<endl;
            //    str_stream << 0 << " " << ++debugCount << " " << 0;
            //}

            //push message for Robot on queue
            //ccQ.push(str_stream.str());
            if ((++camp->mNumOfPositiveDetect >= 1)) // && ((camp->mNumOfPositiveDetect % UPDATE_ROBOT_FREQ) == 0))
            {
                camp->mNumOfNegativeDetect = 0;
                robp->SendInstructionToRobot(SENSOR, str_stream.str());
                robp->SetTargetDeviation(static_cast<int>(floor(deviation)));
                robp->SetOperationMode(Robot::FOLLOWING);

                cout << "\n\r mNumOfPositiveDetect = " << camp->mNumOfPositiveDetect
                     << " mNumOfTrials = " << camp->mNumOfTrials << endl;
                cout << "\n\r ROBOT IN FOLLOWING MODE !!!!" << endl;


            }
            //still need to send something to robot just check distance ?
            else
            {
                robp->SendInstructionToRobot(NOCHANGE, "");
            }

        }
        else
        {
            ++camp->mNumOfNegativeDetect;
            //if (camp->mNumOfNegativeDetect < 2)
            //{
            //    robp->SendInstructionToRobot(NOCHANGE, "");
            //}
            //else
            if (camp->mNumOfNegativeDetect == 1)
            {
                camp->mNumOfPositiveDetect = 0;
                robp->SendInstructionToRobot(STOP, "");
                cout << "\n\r mNumOfNegativeDetect = " << camp->mNumOfNegativeDetect
                     << " mNumOfTrials = " << camp->mNumOfTrials << endl;
                robp->SetOperationMode(Robot::SCANNING);
                cout << "\n\r ROBOT IN SCANNING MODE !!!!" << endl;
            }
            else
            {
                robp->SendInstructionToRobot(NOCHANGE, "");
            }
        }

        #if USER_ENABLED == true
        //push image clone for user Task
        cuQ.push(frame.clone());
        //imshow( "result", frame.clone());
        #endif
    

        //uint32_t delay = 200 - detectTime;
        //this_thread::sleep_for(chrono::milliseconds(10));
    }
}


bool Camera::InitializeCascade(CascadeClassifier& cascade, CascadeClassifier& nestedCascade)
{
    //string cascadeName = "haarcascades/haarcascade_fullbody.xml";
    //string cascadeName = "haarcascades/haarcascade_upperbody.xml";
    //string cascadeName =  "haarcascades/cars.xml";
    string cascadeName =  "haarcascades/haarcascade_frontalface_alt.xml";
    //string nestedCascadeName =  "haarcascade_frontalface_alt.xml";
    //string cascadeName =  "haarcascades/haarcascade_frontalface_alt.xml";
    string nestedCascadeName = "haarcascades/haarcascade_eye.xml";
    //string cascadeName = "haarcascade_russian_plate_number.xml";
    //string cascadeName = "haarcascades/haarcascade_licence_plate_rus_16stages.xml";
    //string cascadeName = "haarcascades/haarcascade_russian_plate_number.xml";
    
    if (!cascade.load(cascadeName))
    {
        cerr << "Failed loading cascade " << cascadeName;
        return false;
    }

    if (!nestedCascade.load(nestedCascadeName))
    {
        cerr << "Failed loading nestedcascade " << nestedCascadeName;
        return false;
    }

    return true;
}

void Camera::InitializeCamera(VideoCapture& capture)
{
    capture.open(0, 0.05);

    if (!capture.set(CAP_PROP_FRAME_WIDTH, FRAME_WIDTH_PIX))
    {
        cout << "CAP_PROP_FRAME_WIDTH " << FRAME_WIDTH_PIX << " not supported" <<endl;
    }

    if (!capture.set(CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT_PIX))
    {
        cout << "CAP_PROP_FRAME_HEIGHT " << FRAME_HEIGHT_PIX << " not supported" <<endl;
    }

    if (!capture.set(CAP_PROP_FPS, FRAME_RATE))
    {
        cout << "CAP_PROP_FPS " << FRAME_RATE << " not supported" <<endl;
    }
    
    cout << "\n\r " << capture.get(CAP_PROP_FRAME_WIDTH) << " "
                    << capture.get(CAP_PROP_FRAME_HEIGHT)<< " "
                    << capture.get(CAP_PROP_FPS) << endl;

}


pair<bool,bool> Camera::detectAndDraw( Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
                                       double scale, bool tryflip,
                                       int& xErrorPix, int& zError, int& angle )
{
    vector<Rect> objects, faces2;
    bool objectDetected = false;
    bool motionDetected = false;

    static Point centerPrev;
    static int estimateDistance = 0;

    double deviation = 0.0f;
    double fx = 0.0f;

    Mat gray, smallImg;

    cvtColor( img, gray, COLOR_BGR2GRAY );
    fx = 1 / scale;
    resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    //save processing time by scanning only bottom half of the image
    //Rect roi(0, FRAME_HEIGHT_PIX/2, FRAME_WIDTH_PIX, FRAME_HEIGHT_PIX /2);
    //smallImg = smallImg(roi);

    vector<int> reject;
    //flip(smallImg, smallImg, 1);
    cascade.detectMultiScale(smallImg,
                             objects,
                             //reject,
                             1.3,
                             4,
                             0
                            |CASCADE_FIND_BIGGEST_OBJECT
                            |CASCADE_DO_ROUGH_SEARCH,
                            //|CASCADE_DO_CANNY_PRUNING,
                            //|CASCADE_SCALE_IMAGE,
                            Size(25, 25)) ;
                            //Size(80,80) );



    for ( size_t i = 0; i < objects.size(); i++ )
    {
        Rect r = objects[i];
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        Scalar color = colors[i%8];
        int radius;

        double aspect_ratio = (double)r.width/r.height;
        if( 0.75 < aspect_ratio && aspect_ratio < 1.3 )
        {
            objectDetected = true;
            center.x = cvRound((r.x + r.width*0.5)*scale);
            center.y = cvRound((r.y + r.height*0.5)*scale);

            //cout << "\n\r aspect ratio = " << aspect_ratio << "Reject = " << reject[i];

            //cout << "\n\r r = " << r << endl;

            //compute error on the x-axis
            if (center.x  != centerPrev.x)
            {
                motionDetected = true;
                //horizontal deviation from center

                xErrorPix = center.x - FRAME_COORDINATES_WIDTH_PIX/2.0f;

                //cout << "\n\r  center.x = " <<  center.x << " xErrorPix = " << xErrorPix;
                centerPrev = center;
            }


            radius = cvRound((r.width + r.height)*0.25*scale);
            circle( img, center, radius, color, 3, 8, 0 );

            rectangle(img, cvPoint(cvRound(r.x*scale), cvRound(r.y*scale)),
                      cvPoint(cvRound((r.x + r.width-1)*scale), cvRound((r.y + r.height-1)*scale)),
                      color, 3, 8, 0);
        }

        #if PROCESS_NESTED == true

        smallImgROI = smallImg( r );

        nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
                                        1.1,
                                        4,
                                        0
                                        //|CASCADE_FIND_BIGGEST_OBJECT
                                        //|CASCADE_DO_ROUGH_SEARCH
                                        //|CASCADE_DO_CANNY_PRUNING
                                        |CASCADE_SCALE_IMAGE,
                                        Size(30, 30) );

        for ( size_t j = 0; j < nestedObjects.size(); j++ )
        {
                Rect nr = nestedObjects[j];
                center.x = cvRound((r.x + nr.x + nr.width*0.5)*scale);
                center.y = cvRound((r.y + nr.y + nr.height*0.5)*scale);
                radius = cvRound((nr.width + nr.height)*0.25*scale);

                cout <<"\n\r Found nested object !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" ;
        }
        #endif
    }
   //imshow( "result", img );

   return make_pair(objectDetected, motionDetected);
}

namespace CAMERA
{
    static void help()
    {
        cout << "\nThis program demonstrates the cascade recognizer. Now you can use Haar or LBP features.\n"
                "This classifier can recognize many kinds of rigid objects, once the appropriate classifier is trained.\n"
                "It's most known use is for objects.\n"
                "Usage:\n"
                "./facedetect [--cascade=<cascade_path> this is the primary trained classifier such as frontal face]\n"
                   "   [--nested-cascade[=nested_cascade_path this an optional secondary classifier such as eyes]]\n"
                   "   [--scale=<image scale greater or equal to 1, try 1.3 for example>]\n"
                   "   [--try-flip]\n"
                   "   [filename|camera_index]\n\n"
                "see facedetect.cmd for one call:\n"
                "./facedetect --cascade=\"../../data/haarcascades/haarcascade_frontalface_alt.xml\" --nested-cascade=\"../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml\" --scale=1.3\n\n"
                "During execution:\n\tHit any key to quit.\n"
                "\tUsing OpenCV version " << CV_VERSION << "\n" << endl;
    }

}









