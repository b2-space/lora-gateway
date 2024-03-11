#ifndef MGPS_USB_SER_H_
    void connectSerial(char* serialPort, int baudRate);
    void disconnectUSB();
    void sendStringUSB(char* str);
    void *GpsUsbLoop( void *some_void_ptr );
#define GPS_USB_SER_H_
#endif
