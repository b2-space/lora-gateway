#ifndef _H_Gateway
#define _H_Gateway

#include "global.h"

int receiveMessage( int Channel, char *message, rx_metadata_t *Metadata );
void hexdump_buffer( const char *title, const char *buffer,
                     const int len_buffer );
void LogPacket( rx_metadata_t *Metadata, int Bytes, unsigned char MessageType );
void LogTelemetryInit();
void LogTelemetryPacket(int Channel, char *telemetry_str, received_t *rx_message);
void LogTelemetryCommand(int Channel, char *command_str);
void LogError(char *Message1, char *Message2);
void LogMessage( const char *format, ... );
void ChannelPrintf( int Channel, int row, int column, const char *format,
                    ... );
void displayChannel (int Channel); 

#endif
