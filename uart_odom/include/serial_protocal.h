#ifndef __SERIAL_PROTOCOL_H__
#define __SERIAL_PROTOCOL_H__
#include <math.h>
#include <stddef.h>
#ifdef  __cplusplus  
extern "C" 
{
#endif
    void    make_packet(char* data, char *send_buffer, int packet_id, int data_size, int* packet_size);
    int     onRec(char c, char* rec_buffer, int* current_index, int *id, int *data_size, char* rec_packet);
#ifdef  __cplusplus  
}
#endif 

#endif
