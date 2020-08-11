/*
 *
 */

#include <math.h>
#include "constants.h"
#include "simpletools.h"
#include "abdrive.h"
#include "ping.h"
#include "fdserial.h"
#include <stdarg.h>
#include <stdlib.h>
#include <propeller.h>
#include "serial.h"
#include <errno.h>
#include "servo.h"
#include "ping.h" // Include adcDCpropab library


#define START_MSG_BYTE   0x7B
#define END_MSG_BYTE     0x7D
#define ESCAPE_BYTE      0x7E
#define UN_ESCAPE(x)    (0xFF & (0x20 ^ (x)))

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
     
#define sign(a) \
   ({ __typeof__ (a) _a = (a); \
     _a >= 0 ? 1 : -1; })


const int MAX_SPEED = 128;
const int ACOUSTIC_SENSOR_SCANNING_FREQ = 5;
const int SAFE_DISTANCE_FROM_TARGET = 20; //cm

enum
{
  NONE = 0,
  STARTUP,
  E_STOP_1,
  E_STOP_2,
  RUNNING
};  

enum
{
   NOCHANGE,
   STOP,
   SENSOR,
   REPORT,
   SPEED,
   SCANNING
}; 

#include "adcDCpropab.h"                      // Include adcDCpropab library



static unsigned int RotateSensorsStack[256 + 20]; 

int distance;
int prev_distance;
float ir_adc_voltage;

void ping_cog(void* params);
void speed_control_cog(void* params);

void rotate_sensors_cog(void* params);
static volatile int rotate_sensors_done = 1;

unsigned char stopMsg[] = "STOP";
unsigned char sensorMsg[] = "SENSOR";
unsigned char reportMsg[] = "REPORT";
unsigned char speedMsg[] = "SPEED";
unsigned char nochangeMsg[] = "NOCHANGE";
unsigned char scanningMsg[] = "SCANNING";

fdserial *xbee;

char* rx_buffer; // = { '\0' };
int rx_index = 0;

int object_speed = 0;
int speed_control_flag = 0;

int LWSpeed = 0;
int RWSpeed = 0;

int rotation_angle = 900;

typedef enum MSG_STATE
{
    WAITING_MSG = 0,
    RECEIVING_MSG,
    PROCESSING_MSG
};

enum MSG_STATE msg_state = WAITING_MSG;

void DecodeCharacter(int* c)
{
    switch(*c)
    {
        case START_MSG_BYTE:
        {
          msg_state = RECEIVING_MSG;
        }
        break;
        
        case END_MSG_BYTE:
        {
          msg_state = PROCESSING_MSG;
        }
        break;
        
        default:
        break;          
    }   
   
}  

int ProcessMessage(int* xError, int* zError, int* angle)
{
    
    //start at byte 1 just after START_MSG_BYTE
    char* ptr = &rx_buffer[2];
    
    if (memcmp(ptr, scanningMsg, 8) == 0)
    {
        return SCANNING;
    }
    else      
    if (memcmp(ptr, nochangeMsg, 8) == 0)
    {
          
        return NOCHANGE;
    }
    else         
    if (memcmp(ptr, stopMsg, 4) == 0)
    {

        return STOP;
    }
    else
    if (memcmp(ptr, speedMsg, 5) == 0)
    {
        char* end; 
        int count;
        int error[10];     
        ptr = &rx_buffer[7];
        object_speed = strtol(ptr, &end, 10);
        //print("\n\r got Speed = %d !!!!!!!!!!!!!!!!!!", object_speed); 
        
        return SPEED;
    }      
    if (memcmp(ptr, sensorMsg, 6) == 0)  
    {      
        char* end; 
        int count;
        int error[10];     
        //print("\n\r CHANGE ");
        ptr = &rx_buffer[8];

        for (long i = strtol(ptr, &end, 10); ptr != end; i = strtol(ptr, &end, 10))
        {
           //printf("'%.*s' -> ", (int)(end-ptr), ptr);
           ptr = end;
           if (errno == ERANGE)
           {
              print("range error, got ");
              errno = 0;
           }
            //printf("%ld\n", i);
            error[count++] = (int)i;
            
        }
        *xError = error[0];
        *zError = error[1];
        *angle  = error[2];
        
        //print("\n\r got SENSOR = %d !!!!!!!!!!!!!!!!!!", xError); 
        
        return SENSOR;
    }
    else
        return -1; 
        
    return 0;          
                       
}

void RotateCamera(int angle)
{
    servo_speed (16, 90);
    servo_angle(16, angle * 10); // P16 servo to angle  x 10  degrees
}  

//report distance to master
void SendPingDistance()
{
    distance = ping_cm(17);
    //pause(500);
    print("\n\r Sensing Ping Distance = %d", distance);
    writeChar(xbee, 0x7B);
    writeDec(xbee, distance);
    writeChar(xbee, 0x7D);
   
    //emergency stop
    if (distance < 10)
    {
        LWSpeed = RWSpeed = 0;
        drive_speed(LWSpeed, RWSpeed);    
    }      
    prev_distance = distance;
}  

void SendIRDistance()
{
    ir_adc_voltage = adc_volts(0);
    print("\n\r Sensing IR Distance = %f", ir_adc_voltage);
}  
  
int main()
{
  int c;
  
  rx_buffer = malloc(32);
  if (!rx_buffer) 
  {
     print("failed allocating memory"); 
     return -1; 
  }
  memset(&rx_buffer[0], '\0', 32);
  
  xbee = fdserial_open(9, 8, 0, 115200);
  print("Acknowledged!!!!!!: %d\n", xbee);


  //adc_init(21, 20, 19, 18);      // Initialize ADC on Activity Board
  //ir_adc_voltage = adc_volts(0); //read fron ADC0
  
 
  

  //InitSensors();
  servo_angle(16, 900);
  pause(10);
  //cog_run(ping_cog, 512);
  rotate_sensors_done = 1;
  //cog_run(rotate_sensors_cog,512);
  //cogstart(rotate_sensors_cog, NULL, RotateSensorsStack, sizeof(RotateSensorsStack));

  

  
  //drive_speed(128, 128);
  drive_setAcceleration(0, 200);
  
  while(1)
  {    
    #ifdef ULTRASONIC_SENSOR_ONLY
    while(1)
    {
         distance = ping_cm(17);
         pause(200);
         print("\n\r Sensing Ping Distance = %d", distance);
    }      
    if (object_speed)
    {
        printf("\nobject prev_distance= %d distance= %d cm speed=%d cm/s", 
               prev_distance, distance, object_speed);
        LWSpeed = 3 * object_speed;
        RWSpeed = 3 * object_speed;
        
        drive_speed(LWSpeed, RWSpeed);
    }
    #endif
    
    if (fdserial_rxReady(xbee) == 0)
    {
       //printf("object speed=%d cm", object_speed);
       pause(1);
       
       continue;
    }      
    
    c = fdserial_rxChar(xbee);
    DecodeCharacter(&c);
    if (msg_state == RECEIVING_MSG)
    {
        rx_buffer[rx_index++] = c;  
    }
    else 
    if (msg_state == PROCESSING_MSG)
    {
        int xError = 0;
        int zError = 0;
        int angle  = 0;
        
        rx_buffer[rx_index] = '\0';
        if (ProcessMessage(&xError, &zError, &angle) == SCANNING)
        {
            LWSpeed = RWSpeed = 0;
            drive_speed(LWSpeed, RWSpeed);
            rotation_angle = 900;               
            servo_angle(16, rotation_angle);  
        }
        else          
        if (ProcessMessage(&xError, &zError, &angle) == NOCHANGE)
        {
            //SendPingDistance();
        }
        else          
        if (ProcessMessage(&xError, &zError, &angle) == STOP)
        {
            LWSpeed = RWSpeed = 0;
            drive_speed(LWSpeed, RWSpeed);
            speed_control_flag = 1;
            rotation_angle = 900;               
            servo_angle(16, rotation_angle);  
            //SendPingDistance();
            //print("\n\r GOT STOP !!!!!!!!!!!!!!!!!!!!!!!");
        }
        else
        if (ProcessMessage(&xError, &zError, &angle) == SPEED)
        {
            if (rotation_angle > 900)
            {
                float delta = abs(rotation_angle - 900) / 900.0; 
                RWSpeed = sign(object_speed) * min(abs(3*object_speed), MAX_SPEED);
                LWSpeed = max((int)(RWSpeed/2.0), RWSpeed* (1-delta));
                //LWSpeed = RWSpeed * (1-delta);
                print("\n\r rotation_angle=%d LWSpeed=%d RWSpeed=%d", rotation_angle, LWSpeed, RWSpeed);
                //speed_control_flag = 1;
            }
            else
            if (rotation_angle < 900)
            {
                float delta = abs(rotation_angle - 900) / 900.0;
                LWSpeed = sign(object_speed) * min(abs(3*object_speed), MAX_SPEED);
                RWSpeed = max((int)(LWSpeed/2.0), LWSpeed* (1-delta)); 
                //RWSpeed = LWSpeed * (1-delta);
                print("\r\n rotation_angle=%d LWSpeed=%d RWSpeed=%d", rotation_angle, LWSpeed, RWSpeed);
                //speed_control_flag = 1;
            }
            else
            {
                 LWSpeed = RWSpeed = sign(object_speed) * min(abs(3*object_speed), MAX_SPEED);
                 //speed_control_flag = 1;
            }      
            
            drive_speed(LWSpeed, RWSpeed);            
        }
        else           
        if (ProcessMessage(&xError, &zError, &angle) == SENSOR) //adjust the motor supporting the sensors
        {
            printf("\n\r %d %d %d current rotation_angle = %d Speed ( %d , %d ) \n\r", xError, zError, angle, rotation_angle, LWSpeed, RWSpeed);
            
            
            if (angle)
            {
                rotation_angle += 10*(angle);
                rotation_angle = (rotation_angle) > 1800 ? 1800 :
                                 (rotation_angle) <   0 ?   0 :
                                  rotation_angle;
                                  
                servo_angle(16, rotation_angle);  
                //pause(1);
                //flag rotate_sensors_cog that there is an update
                //rotate_sensors_done = 0;
                
                //while (rotate_sensors_done == 0) pause(10);
            } 
                      
            //measure distance to object after rotation is done and calculate approxinate relative 
            //from target speed: if negative, target is getting closer and we have to slow down
            SendPingDistance();
            //SendIRDistance();
            
            //from the provided angle calculate speed of the wheels while avoiding sharp turns
            //print("\n\r (%d %d %d %d)", xError, zError, angle, angle);
                   
                             
        }          
        msg_state = WAITING_MSG;
        rx_index = 0;
        memset(&rx_buffer[0], '\0', 32);     
        
    }  
    
    //pause(1); 
  }  
  
   
   //for (i = 0; i < 2; i++)
      
   return 0;
}
void rotate_sensors_cog(void* params)
{
   while(1)
   {
      if (rotate_sensors_done == 0)
      {
          servo_angle(16, rotation_angle); 
          //pause(200);
          rotate_sensors_done = 1; 
      }            
   }        
}
  
void speed_control_cog(void* params)
{
    while(1)
    {
        if (speed_control_flag == 1)
        {
            drive_speed(LWSpeed, RWSpeed); 
            speed_control_flag = 0;
             
        }  
        pause(1);        
    }          
}  

void ping_cog(void* params)
{
   //char buffer[8];
   int count = 10;
   while(1)
   { 
       count = 10;
       prev_distance = distance;
       distance = ping_cm(17);
       //report distance to master
       //itoa(distance, buffer, 10);
       writeChar(xbee, 0x7B);
       writeDec(xbee, distance);
       writeChar(xbee, 0x7D);
       //fdserial_txChar(xbee, 'a');
       object_speed = (distance - prev_distance) * ACOUSTIC_SENSOR_SCANNING_FREQ;
       pause(200);
       
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        pause(100); 
   }     

}


