//COMMAND LINE :


//g++ sc_Robot.cpp sc_ObjectTracking.cpp sc_usb_serial_comm.cpp -lopencv_core -lopencv_imgproc -lopencv_objdetect -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -std=c++17 -lpthread -I ../opencv/modules/highgui/include/ -I ../opencv/modules/videoio//include/ -I ../opencv/modules/core/include/ -I ../opencv/build/ -I ../opencv/modules/objdetect/include/ -I ../opencv/modules/imgcodecs/include/ -I ../opencv/modules/gapi/include -I ../opencv/modules/imgproc/include/ -o a.out 

#define BOARD RASPBERRY_PI


#include <errno.h>
//#include <wiringPiI2C.h>

#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>
#include <stdlib.h>
#include <set>
#include <queue>
#include <functional>
#include <memory>
#include <string>

#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <linux/i2c-dev.h>

#include "sc_Robot.hpp"
#include "sc_ObjectTracking.hpp"
#include "sc_common.hpp"
#include "opencv2/core/core_c.h"

using namespace std;
using namespace cv;
using namespace USB_COMM;

#define USER_ENABLED false
#define COMM_WITH_ROBOT true
#define OBJECT_TRACKING_ENABLED true

#define START_MSG_BYTE 0x7B
#define END_MSG_BYTE   0x7D
#define ESCAPE_BYTE    0x7E
#define UNESCAPE_BYTE(x) (0xFF & (0x20 ^ (x)))
#define MSG_FROM_ROBOT_MAX_LENGTH 32



#define TRACKING_DISTANCE 25 //cm
#define SAFETY_DISTANCE 15

#define NUM_OF_TARGET_POSITION_SAMPLES 5

namespace USER
{
    void userInput(CriticalQueue<Mat>& cuQ)
    {
        int c = 0;

        while(1)
        {
            //pop image clone sent from detection task

            if (!cuQ.empty())
            {
                imshow( "result", cuQ.pop());
            }

            c = waitKey(1);

            std::this_thread::yield();
        }
    }
}

void ListenToRobot(unique_ptr<Robot>& robotp, int serComHdl)
{
     cout << "ListenToRobot comm port = " <<  serComHdl << endl;
     
    string rx_buffer;
    rx_buffer.resize(MSG_FROM_ROBOT_MAX_LENGTH);

    char c;
    ssize_t readLen = 0;

    ROBOT::MSG_STATE msgState = ROBOT::WAITING_MSG;

    while(1)
    {
        readLen = read(serComHdl, &c, 1);

        if (readLen == 1)
        {
            switch (msgState)
            {
                case ROBOT::WAITING_MSG:
                    if (c == START_MSG_BYTE)
                    {
                        msgState = ROBOT::RECEIVING_MSG;
                    }
                break;

                case ROBOT::RECEIVING_MSG:
                    if (c == END_MSG_BYTE)
                    {
                        robotp->time_stamp = (double)cvGetTickCount() - robotp->time_stamp;

                        //keep previous distance
                        uint32_t prevDistance = robotp->GetTargetDistance();
                        stringstream str_istream(rx_buffer) ;

                        uint32_t currDistance = 0xFFFFFF;
                        str_istream >> currDistance;
                        robotp->SetTargetDistance(currDistance);

                        rx_buffer.clear();
                        msgState = ROBOT::WAITING_MSG;

                        //if no target detected in scanning mode we are done
                        if (robotp->GetOperationMode() == Robot::SCANNING)
                        {
                            //robotp->SendInstructionToRobot(SCANNING, "");
                            break;
                        }

                        //stop, not safe
                        if (currDistance <= SAFETY_DISTANCE)
                        {
                           robotp->mSpeed = 0;
                           cout  << "\n 2 Got distance = " << currDistance << " Speed= " << robotp->mSpeed;
                        }
                        else
                        //if is acceptable tracking zone, slightly adjust speed to get closer to the safety line
                        if (currDistance > SAFETY_DISTANCE && currDistance <= TRACKING_DISTANCE)
                        {
                            //if distance reduced, slow down else if increased speed up else maintain
                            //robotp->mSpeed += (currDistance - prevDistance) * 5;
                        }
                        else
                        //catch up
                        if (currDistance > TRACKING_DISTANCE)
                        {
                            //int32_t diffSpeed = robotp->mSpeed + ((currDistance - prevDistance) * 5);
                            robotp->mSpeed += ((currDistance - prevDistance) * 5);
                            //robotp->mSpeed = max(static_cast<int32_t>((robotp->mSpeed + diffSpeed) / 2.0), 0);
                            cout  << "\n 1 Got distance = " << currDistance << " Speed= " << robotp->mSpeed;
                        }

                        if (robotp->mSpeed == 0)
                        {
                            robotp->SendInstructionToRobot(STOP, "");
                        }
                        else
                        {
                            //adjust speed in function of target deviation, slow down with greater deviation
                            robotp->mSpeed = static_cast<uint32_t>(robotp->mSpeed * (abs(cos(robotp->GetTargetDeviation() * 3.1416f / 180.0f))));

                            cout << "\n\r Adjusted speed is " << robotp->mSpeed << " robotp->GetTargetDeviation() = " << robotp->GetTargetDeviation()
                                 << "sin = " << sin(robotp->GetTargetDeviation() * 3.1416f / 180.0f);

                            //if camera rotation angle is close to 90 or -90, we need decelerate and rotate the vehicle sharply
                            //ratio between left whell and right wheel is goes high as camera rotation is higher
                            //LWSpeed = (1-abs((GetTargetDeviation() / 90 ))) * RWSpeed;
                            //RWSpeed = (1-abs((GetTargetDeviation() / 90 ))) * LWSpeed;

                            string msg = to_string(robotp->mSpeed);
                            robotp->SendInstructionToRobot(SPEED, msg);
                        }
                    }
                    else
                    {
                        rx_buffer.push_back(c);
                    }
                 break;

                default:
                break;
            }
        }
     }
}


//read data from ObjectTracking via queue and sends to the robot on serial comm
void ListenToCameraUpdates(CriticalQueue<string>& ccQ, unique_ptr<Robot>& robp)
{
    string msg;
    int commHandler = robp->GetSerialComHdl();
    cout << "ListenToCameraUpdates comm port = " <<  commHandler << endl;

    while(1)
    {
        if (!ccQ.empty())
        {
            msg = ccQ.pop();

            int count =  write(commHandler, msg.c_str(), msg.size());
            robp->time_stamp = (double)cvGetTickCount();
            cout << "\n" << msg << " sent " << count << " bytes at " << robp->time_stamp / ((double)cvGetTickFrequency()*1000.) << " ms" << endl;
        }

        std::this_thread::yield();
    }
}
void Robot::ConfigSerial()
{
    mSerialComHdl = USB_COMM::serial_port_open();
    cout << "Robot::ConfigSerial opened port " << mSerialComHdl << endl;
}

void Robot::GetPositionSensorsData()
{

}

//fuse data from distance sensors and image object tracking
void Robot::ProcessPositionData()
{
    static int sumX = 0;
    static int sumZ = 0;
    static int lastPopulatedIndex = 0;

    //Integrate the last n positions for error on x
    //sumX += vpos[lastPopulatedIndex].first - vpos[(lastPopulatedIndex+1) % NUM_OF_REC_POS ].first;

     //Integrate the last n positions for error on z
    //sumZ += vpos[lastPopulatedIndex].second - vpos[(lastPopulatedIndex+1) % NUM_OF_REC_POS ].second;
}

void Robot::SendInstructionToRobot(enum RobotInstructions instruction, const string& msg)
{
    if (mSerialComHdl == -1) return;

    switch(instruction)
    {
        case RobotInstructions::NOCHANGE:
        {
            string inst = "{ NOCHANGE }";
            int count =  write(mSerialComHdl, inst.c_str(), inst.size());
            //time_stamp = (double)cvGetTickCount();
            //cout << "\n" << inst << " NOCHANGE sent " << count << " bytes at " << time_stamp / ((double)cvGetTickFrequency()*1000000.) << " s" << endl;

        }
        break;

        case RobotInstructions::SENSOR:
        {
            string inst = "{ SENSOR " + msg + " }";
            int count =  write(mSerialComHdl, inst.c_str(), inst.size());
            time_stamp = (double)cvGetTickCount();
            cout << "\n" << inst << " SENSOR sent " << count << " bytes at " << time_stamp / ((double)cvGetTickFrequency()*1000000.) << " s" << endl;
        }
        break;

        case RobotInstructions::SPEED:
        {
            string inst = "{ SPEED " + msg + " }";
            int count =  write(mSerialComHdl, inst.c_str(), inst.size());
            time_stamp = (double)cvGetTickCount();
            cout << "\n" << inst << " SPEED sent " << count << " bytes at " << time_stamp / ((double)cvGetTickFrequency()*1000000.) << " s" << endl;
        }
        break;

        case RobotInstructions::STOP:
        {
            string inst = "{ STOP }";
            int count =  write(mSerialComHdl, inst.c_str(), inst.size());
            time_stamp = (double)cvGetTickCount();
            cout << "\n" << inst << " STOP sent " << count << " bytes at " << time_stamp / ((double)cvGetTickFrequency()*1000.) << " ms" << endl;
        }
        break;

        case RobotInstructions::SCANNING:
        {
            string inst = "{ SCANNING }";
            int count =  write(mSerialComHdl, inst.c_str(), inst.size());
            time_stamp = (double)cvGetTickCount();
            cout << "\n" << inst << " SCANNING sent " << count << " bytes at " << time_stamp / ((double)cvGetTickFrequency()*1000.) << " ms" << endl;
        }

        default:
        break;
    }


}

void Robot::SendDistanceRequestToRobot()
{
    string message("{ GETDISTANCE }");
    //write(serComHdl, message.c_str(), message.size());
}

uint32_t  Robot::GetTargetDistance()
{
    uint32_t tmp = *mTargetDistance;
    return tmp;
}

void Robot::SetTargetDistance(uint32_t updatedTargetDistance)
{
    static uint32_t index = 0;
    mTargetDistance.Set(updatedTargetDistance);

    //to update average of the last N distances, substract the oldest and add the new
    /*
    mAvgTargetDistance =
        ((mAvgTargetDistance * NUM_OF_TARGET_POSITION_SAMPLES)
        - mTargetPositionHist[mTargetPositionHistIndex].first +
        updatedTargetDistance) / NUM_OF_TARGET_POSITION_SAMPLES;

    mTargetPositionHist[mTargetPositionHistIndex].first = updatedTargetDistance;

    mTargetPositionHist[mTargetPositionHistIndex].second = GetTargetDeviation();

    mTargetPositionHistIndex = ++mTargetPositionHistIndex % NUM_OF_TARGET_POSITION_SAMPLES;
    */
}

int32_t  Robot::GetTargetDeviation()
{
    return *mTargetDeviation;
}

void Robot::SetTargetDeviation(uint32_t targetDeviation)
{
    mTargetDeviation.Set(targetDeviation);
}

float Robot::ComputeAvgTargetPosition()
{
    mAvgTargetPosition.first =
        (*mTargetDistance  + mAvgTargetPosition.first * (NUM_OF_TARGET_POSITION_SAMPLES - 1)) / NUM_OF_TARGET_POSITION_SAMPLES;
}



enum VEHICLE_STATE
{
    STAND_BY,
    STATE_SCANNING,
    FOLLOWING
};

int main( int argc, const char** argv )
{

    //create robot abstraction object
    unique_ptr<Robot> robp(new Robot);
    robp->ConfigSerial();

    //we need a queue between comm and capture
    CriticalQueue<string> TrackerRobotQ;

    //we need a queue between capture and userInput
    CriticalQueue<Mat> captUserQ;

    VideoCapture capture;
    Mat image;
    string inputName;
    bool tryflip = false;
    CascadeClassifier cascade, nestedCascade;
    double scale = 1.0f;

    unique_ptr<Camera> camp(new Camera(1));
    camp->InitializeCascade(cascade, nestedCascade);
    camp->InitializeCamera(capture);

    //cascade = cv2.CascadeClassifier("../../data/haarcascades/cars.xml");

    if( capture.isOpened() )
    {
        cout << "Video capturing has been started ..." << endl;

        //we keep threads ID in the set
        std::set<std::thread::id>  ts;

        #if USER_ENABLED == true
        std::thread userThread(USER::userInput,  std::ref(captUserQ)  );
        std::thread::id ut1 = userThread.get_id();
        ts.insert(ut1);
        cout <<"\n\r Started Thread to get input from user id=" << ut1;
        userThread.detach();
        #endif

        //start a thread for detection
        #if OBJECT_TRACKING_ENABLED == true
        std::thread trackObjectThread(TrackObject, 
                                      std::ref(camp),
                                      std::ref(robp),
                                      std::ref(capture), std::ref(cascade), std::ref(nestedCascade),
                                      scale, tryflip,
                                      std::ref(TrackerRobotQ) ,
                                      std::ref(captUserQ));

        std::thread::id dt1 = trackObjectThread.get_id();
        ts.insert(dt1);
        cout << "\n\r Started Thread to detect object id=" << dt1;
        trackObjectThread.detach();
        #endif

        //start a thread for communication of the moves

        #if MSG_QUEUE_FROM_OBJ_TRACK == true
        std::thread commThread(ListenToCameraUpdates, std::ref(TrackerRobotQ) , std::ref(robp));
        std::thread::id ct1 = commThread.get_id();
        ts.insert(ct1);
        cout <<"\n\r Started Thread to communicate moves id=" << ct1 << endl;
        commThread.detach();
        #endif
        
        #if COMM_WITH_ROBOT == true
        std::thread commRxThread(ListenToRobot, std::ref(robp), robp->GetSerialComHdl());
        std::thread::id ct2 = commRxThread.get_id();
        ts.insert(ct2);
        cout << "\r\n Started thread listening to robot id=" << ct2 << endl;
        commRxThread.detach();
        #endif

        //state machine

        VEHICLE_STATE VehicleState = STAND_BY;

        while(1)
        {
            sleep(200);

            switch(VehicleState)
            {
                case STAND_BY:
                {
                    //instruct robot to scan for target
                    //robp->SendInstructionToRobot("SCAN");
                    VehicleState = STATE_SCANNING;
                }
                break;

                case STATE_SCANNING:
                {


                }
                break;

                //take picture, ask for potential distance from acoustic/infra-red sensor
                // and send visual position of target to robot
                case FOLLOWING:
                {

                }
                break;

                default:
                {

                }
            }
        }
    }
    else
    {
        cout << "Video capturing cannot be started, not opened ..." << endl;

    }

    return 0;
}

