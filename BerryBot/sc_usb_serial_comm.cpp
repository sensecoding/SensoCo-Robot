#include <fcntl.h>
#include <signal.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>   // Standard input/output definitions
#include <string.h>  // String function definitions
#include <errno.h>


#include "sc_common.hpp"

//#define UNIT_TEST 


namespace USB_COMM
{
    static const char *PORT_NAME = "/dev/ttyUSB0";
    int serial_port;
    struct termios options_original;
    char read_buffer[MAX_COMMAND_LENGTH + 1] ;

    // Opens a USB virtual serial port at ttyUSB0.
    //
    // returns - the port's file descriptor or -1 on error.
    
 
    int set_interface_attribs (int fd, int speed, int parity)
    {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
    }


    void set_blocking (int fd, int should_block)
    {
            struct termios tty;
            memset (&tty, 0, sizeof tty);
            if (tcgetattr (fd, &tty) != 0)
            {
                    //error_message ("error %d from tggetattr", errno);
                    return;
            }

            tty.c_cc[VMIN]  = should_block ? 1 : 0;
            tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

            if (tcsetattr (fd, TCSANOW, &tty) != 0){}
                    //cout << "error "<< errno << "setting term attributes");
    }

    int serial_port_open(void)
    {
 

          struct termios options;
          //if not working do chmod 777 /dev/ttyUSB0

          serial_port = open(PORT_NAME, O_RDWR | O_NONBLOCK | O_NOCTTY | O_SYNC);
          
          set_interface_attribs (serial_port , B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
          set_blocking (serial_port , 0);                // set no blocking

          return (serial_port);
     }   
     
     // Resets the terminal and closes the serial port.

    void serial_port_close()
    {
	    tcsetattr(serial_port,TCSANOW,&options_original);
	    close(serial_port);
    }

    int serial_port_read(char* rxBuffer, size_t max_chars_to_read)
    {
	    int chars_read = read(serial_port, rxBuffer, max_chars_to_read);
	   
	    /*
	     if (chars_read > 0) 
        {
            read_buffer[chars_read] = '\0';
            printf("%c-%s", chars_read, read_buffer);
                       //if (count < 0)
                       // {
                        //    printf("UART TX error\n");
                       // }
        }
        */

	    return chars_read;
    }

     void serial_port_write(char *write_buffer, int bufLen)
    {
	    int bytes_written;
	    size_t len = 0;

	    //len = strlen(write_buffer);
	    bytes_written = write(serial_port, write_buffer, bufLen);
	    if (bytes_written < len)
	    {
            printf("Write failed \n");
	    }
    }
	    
    void  sigint_handler(int sig)
    {
        serial_port_close();
        exit (sig);
    }
}

#ifdef UNIT_TEST
int main()
{
    USB_COMM::serial_port_open();
    return -1;
}

#endif

