#ifndef SC_OBJECT_TRACKING_HPP
#define SC_OBJECT_TRACKING_HPP

#include <mutex>
#include <memory>

#include "opencv2/videoio.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "sc_common.hpp"
#include "sc_Robot.hpp"
#include <queue>

using namespace std;
using namespace cv;

namespace CAMERA
{
    /*
    void DetectObject(VideoCapture capture, CascadeClassifier cascade, CascadeClassifier nestedCascade, 
                      double scale, bool tryflip,
                      std::mutex& mCommCapt, std::queue<ObjTrackMsg>& ccQ,
                      std::mutex& mMatCapt, std::queue<Mat>& cuQ);
                     
    bool detectAndDraw(Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
                       double scale, bool tryflip,
                       int& xError, int& zError, int& angle );
    */
}

const static Scalar colors[] =
{
    Scalar(255,0,0),
    Scalar(255,128,0),
    Scalar(255,255,0),
    Scalar(0,255,0),
    Scalar(0,128,255),
    Scalar(0,255,255),
    Scalar(0,0,255),
    Scalar(255,0,255)
};


                                
class Camera
{
    public:
    
        Camera(uint32_t id):mId(id) {}
        //Camera(const Camera& c) {};
        void InitializeCamera(VideoCapture& capture);
        bool InitializeCascade(CascadeClassifier& cascade, CascadeClassifier& nestedCascade);
        

                                  
        pair<bool,bool> detectAndDraw(Mat& img, CascadeClassifier& cascade, CascadeClassifier& nestedCascade,
                                      double scale, bool tryflip,
                                      int& xError, int& zError, int& angle );

        ~Camera() { }
        
        uint32_t mNumOfPositiveDetect = 0;
        uint32_t mNumOfNegativeDetect = 0;
        uint32_t mNumOfTrials = 0;

    private:
    
        uint32_t mId;
};

void TrackObject(unique_ptr<Camera>& cam,
		unique_ptr<Robot>& robp,
		VideoCapture& capture, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, 
		double scale, bool tryflip,
		CriticalQueue<string>&,
        CriticalQueue<Mat>& cuQ);
                

class Movement
{
        public:
                
                Movement(int xpos, int ypos, int zpos);
                
        private:
                
                
                Movement(){}

};

class PositionsCollector
{


};

#endif
