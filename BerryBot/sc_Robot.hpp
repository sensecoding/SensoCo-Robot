#ifndef SC_ROBOT_HPP
#define SC_ROBOT_HPP

#include <mutex>
#include <queue>
#include "sc_common.hpp"

namespace ROBOT
{
    enum MSG_STATE
    {
        WAITING_MSG,
        RECEIVING_MSG
    };

}

class Robot
{
    public:

        enum OperationMode
        {
            SCANNING,
            FOLLOWING
        };
   
        Robot() {}
        
        void ConfigSerial();
        void GetPositionSensorsData();
        void SendInstructionToRobot(enum RobotInstructions instruction, const string& msg);
        void SendDistanceRequestToRobot();
        void ProcessPositionData();
                                  
        ~Robot() { }
        
        double time_stamp;

        bool isPotentialTarget = false;
        bool isTargetLocked = false;
        
        
        int GetSerialComHdl() 
        {
            return mSerialComHdl;    
        }
        
        uint32_t GetTargetDistance();
        void SetTargetDistance(uint32_t dist);

        int32_t  GetTargetDeviation();
        void SetTargetDeviation(uint32_t dist);

        uint32_t GetNumberOfSamples() { return mNumberOfSamples; }
        void SetNumberOfSamples(uint32_t num) { mNumberOfSamples = num; }

        float ComputeAvgTargetPosition();

        OperationMode GetOperationMode() { return mOperationMode ; }
        void SetOperationMode(OperationMode op) { mOperationMode = op ; }

        int32_t mSpeed = 0;
    private:
        
        int mSerialComHdl;
        
        vector<pair<uint32_t,int32_t>> mTargetPositionHist; //store the last N relative position of the target
        float mAvgTargetDistance { 0 } ;
        uint32_t mTargetPositionHistIndex { 0 };
        
        CriticalInst<uint32_t> mTargetDistance { 0 };
        CriticalInst<int32_t> mTargetDeviation { 0 };
        uint32_t mNumberOfSamples { 0 };
        pair<float, int32_t> mAvgTargetPosition { make_pair(0,0) } ;

        OperationMode mOperationMode { SCANNING };

};


void ListenToRobot(unique_ptr<Robot>& robot, int serComHdl);
void ListenToCameraUpdates(CriticalQueue<string>& ccQ, unique_ptr<Robot>& robp);

#endif

