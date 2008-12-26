#ifndef _serial_file_h_
#define _serial_file_h_

#include <librobotics.h>

#if (librobotics_OS == 1)
#include <sys/termios.h>
#else
#error Not yet support OS
#endif

/**
 *  Base class for serial port communicate device
 */
class serial_file {
public:
    serial_file() {
        file = 0;
    }
    virtual ~serial_file() { }

    /** Open and set up specified serial port
     * @param aDevName [in] Serial port device file name
     * @param aSpeed [in] Communication baudrate in bps
     * @return Serial port file handle
     * Returns NULL when failed to open.
     */
    int open(const char *aDevName, int aSpeed) {
        file = fopen(aDevName, "w+");

        if (file == NULL) {
           return -1;  // invalid device file
        }

        // setup parameters
        if (setup_serial(fileno(file), aSpeed))
           return 0;

        // fail
        perror("Cannot open serial device");
        fclose(file);

        file = 0;
        return -1;
    }

    bool is_open() {
        return (file != 0);
    }

    int close() {
        if (file) {
           if( fclose(file) != 0)
               return -1;
           file = 0;
        }
        return 0;
    }

protected:

    /** Parameter settings for serial port communication
     * @param aFd [in] File descriptor to point serial port
     * @param aSpeed [in] Communication speed (bps)
     * @retval 0 Failed
     * @retval 1 Success
     */
    int setup_serial(int aFd, int aSpeed) {
        int speed;

        struct termios tio;

        speed = bitrate(aSpeed);

        if (speed == 0)
            return 0;  // invalid bitrate

        tcgetattr(aFd, &tio);

        cfsetispeed(&tio, speed); // bitrate

        cfsetospeed(&tio, speed); // bitrate

        tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8; // data bits = 8bit

        tio.c_iflag &= ~(BRKINT | ICRNL | ISTRIP);

        tio.c_iflag &= ~ IXON;   // no XON/XOFF

        tio.c_cflag &= ~PARENB;  // no parity

        tio.c_cflag &= ~CRTSCTS; // no CTS/RTS

        tio.c_cflag &= ~CSTOPB;  // stop bit = 1bit

        // Other
        tio.c_lflag &= ~(ISIG | ICANON | ECHO);

        // Commit
        if (tcsetattr(aFd, TCSADRAIN, &tio) == 0)
            return 1;

        return 0;
    }


    /** SPEED SETTINGS
     * @param aBR [in] Declared for communication speed (4800 -500000)
     * @return cfset[i|o] Returned value for speed function
     * Returns B0 if failed
     */
    speed_t bitrate(int aBR) {
        switch (aBR)
        {
           case 4800:
                 return B4800;

           case 9600:
               return B9600;

           case 19200:
               return B19200;

           case 38400:
               return B38400;

           case 57600:
               return B57600;

           case 115200:
               return B115200;

           case 230400:
               return B230400;

           case 460800:
               return B460800;

           case 500000:
               return B500000;

           default: // invalid bitrate
               return B0;
        }
    }

    FILE* file;
};




#endif
