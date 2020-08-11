#include <mutex>

enum MSG_HEADER
{
        /*
        MSG_HEADER_NONE               = 0x00,
        MSG_HEADER_MOVE_CMD_FIRST     = 0x01,
        MSG_HEADER_MOVE_CMD_STRAIGHT  = 0x01,
        MSG_HEADER_MOVE_CMD_RIGHT     = 0x02,
        MSG_HEADER_MOVE_CMD_LEFT      = 0x03,
        MSG_HEADER_SCAN_CMD_LEFT      = 0x04,
        MSG_HEADER_SCAN_CMD_RIGHT     = 0x05,
        MSG_HEADER_STOP_CMD           = 0x06,
        MSG_HEADER_ACCLERATE          = 0x07,
        MSG_HEADER_DECELERATE         = 0x08,
        */
        
        MSG_HEADER_NONE               = 0x00,
        MSG_HEADER_MOVE_CMD_FIRST = 1,
        MSG_HEADER_ADJUST_MOVE = 2,
        MSG_HEADER_STOP = 3,
        MSG_HEADER_MOVE_CMD_LAST      = 0x08
};
 
struct  CommCaptMsg
{
    int key;
   int xError;
   int zError;
};


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


