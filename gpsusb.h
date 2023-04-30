#ifndef MGPS_USB_SER_H_
    void connectSerial(char* serialPort, int baudRate);
    void disconnectUSB();
    void sendUSB(char* latLongAz);
    void *GpsUsbLoop( void *some_void_ptr );
#define GPS_USB_SER_H_
#endif
