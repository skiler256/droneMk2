#include "drone/blocks/SensorsFusion.hpp"
#include <fcntl.h>
#include <cstdint>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

namespace Sensors {

    bool DriverBase::openI2C(const char *bus, uint8_t address) {

                    
                    file = open(bus, O_RDWR);
                    if (file < 0) {
                        return false;
                    }

                    
                    if (ioctl(file, I2C_SLAVE, address) < 0) {
                        ::close(file);
                        file = -1;
                        return false;
                    }

                    // test
                    if (write(file, nullptr, 0) < 0) {
                        ::close(file);
                        file = -1;
                        return false;
                    }

                    return true;
                }
    bool DriverBase::openUART(const char *portName, speed_t baudrate){
                 
                file = open(portName, O_RDWR | O_NOCTTY | O_SYNC);

                if (file < 0) {
                    return false;
                }

               
                if (tcgetattr(file, &tty) < 0) {
                    ::close(file);
                    file = -1;
                    return false;
                }

                
                cfsetospeed(&tty, baudrate);
                cfsetispeed(&tty, baudrate);

                
                tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);
                tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);
                tty.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
                tty.c_cflag |=  static_cast<tcflag_t>(CS8);
                tty.c_cflag |=  static_cast<tcflag_t>(CREAD | CLOCAL);

                tty.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG);
                tty.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY);
                tty.c_oflag &= ~static_cast<tcflag_t>(OPOST);

                
                tty.c_cc[VMIN]  = 0;
                tty.c_cc[VTIME] = 10; 

                
                if (tcsetattr(file, TCSANOW, &tty) != 0) {
                    ::close(file);
                    file = -1;
                    return false;
                }

                // Vérifie que le port répond encore
                if (tcflush(file, TCIOFLUSH) != 0) {
                    ::close(file);
                    file = -1;
                    return false;
                }
        
            return true;
                }
   }