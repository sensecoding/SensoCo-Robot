#ifndef SC_COMMON_HPP
#define SC_COMMON_HPP

#include <queue>
#include <mutex>
#include <string>

using namespace std;

namespace USB_COMM
{
    #define MAX_COMMAND_LENGTH 32
    int serial_port_open();
    void serial_port_close();
    int serial_port_read(char* read_buffer, size_t max_chars_to_read);
    void serial_port_write(char *write_buffer, int bufLen);
    void  sigint_handler(int sig);
    
}

enum RobotInstructions
{
    STOP,
    SENSOR,
    NOCHANGE,
    SPEED,
    SCANNING
};


template <typename T>
class CriticalInst
{
    public:
    
        CriticalInst(const T& t) : 
                object(t){}

        
        T operator * ()
        {
            m.lock();
            T tmp(object);
            m.unlock();
            return tmp;
        }
       
        void Set(T value)
        {
            m.lock();
            object = value;
            m.unlock();
        }
        
    private:
    
        T object;
        std::mutex m;
    
    
};

template <class T>
class CriticalQueue
{
    public:

        CriticalQueue(){}

        void push(T element)
        {
            m.lock();
            cQueue.push(element);
            m.unlock();
        }

        T pop()
        {
            m.lock();
            T tmp(cQueue.front());
            cQueue.pop();
            m.unlock();
            return tmp;
        }

        bool empty()
        {
            m.lock();
            bool tmp = cQueue.empty();
            m.unlock();
            return tmp;
        }

    private:

        queue<T> cQueue;
        std::mutex m;
};


#endif
