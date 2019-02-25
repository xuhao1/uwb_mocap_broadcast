#ifndef SERIAL_SERVICE_H
#define SERIAL_SERVICE_H

#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

//#include "serial/serial.h"
#include "../../serial/include/serial/serial.h"
#include "serial_protocal.h"

#define SERIAL_SERVICE_VERSION "Serial_service_v0.1"
#define SERIAL_SERVICE_DESC "The beta version."
#define MAX_SEND_BUFFER 100000
#define MAX_REC_BUFFER 100000
#define TEMP_BUFFER_SIZE 10240

struct Serial_packet
{
    enum Serial_direction
    {
        e_direction_unset = 0,
        e_direction_in = 1,
        e_direction_out = -1,
    };
    int              data_length;
    int              packet_length;
    int              id;
    Serial_direction direction;

    char data[ 2048 ];

    Serial_packet()
    {
        //data = new char[1024];
        data_length = 0;
        packet_length = 0;
        id = 0;
        direction = e_direction_unset;
    }
    ~Serial_packet()
    {
        //delete data;
    }
    void display()
    {
        //CONSOLE_SET_STYLE(CONSOLE_COLOR_YELLOW, CONSOLE_COLOR_BLACK);
        switch ( direction )
        {
        case e_direction_unset:
            break;
        case e_direction_in:
            printf( "==> " );
            break;
        case e_direction_out:
            printf( "<== " );
            break;
        };
        printf( "id=%d, size=%d, ", id, data_length );
        for ( int i = 0; i < data_length; i++ )
        {
            printf( "%d, ", data[ i ] );
        }
        printf( "\r\n" );
        //CONSOLE_RESET_DEFAULT;
    }
};

//template <class T> // TODO
class Protocal_to_mcu
{
  public:
    enum e_mode_type
    {
        e_mode_return_when_get_packet = 0, //When packet size is large (>100), enable this
        e_mode_scan_all_packet = 1,        // When packet size is small (<100), enable this
    };
    int         m_debug_info = 0;
    int         m_busy = 0;
    char        m_send_buffer[ MAX_SEND_BUFFER ];
    char        m_rec_buffer[ MAX_REC_BUFFER ];
    char        m_rec_buffer_last[ MAX_REC_BUFFER ];
    char        m_temp_buffer[ TEMP_BUFFER_SIZE ];
    int         m_current_rec_index = 0;
    int         m_last_buffer_size = 0;
    std::mutex  m_send_mutex;
    std::mutex  m_rec_mutex;
    e_mode_type m_mode = e_mode_return_when_get_packet;

    //    QSerialPort * m_serial;
    serial::Serial *m_serial;
#if _WIN32
    Win_serial_port *m_win_serial;
    void init( Win_serial_port *serial );
#endif
    Protocal_to_mcu();
    ~Protocal_to_mcu();

    void set_debug_info( int debug_info )
    {
        m_debug_info = debug_info;
    }

    void set_protocal_mode( int mode_type )
    {
        m_mode = ( e_mode_type ) mode_type;
    }
    void display( int count, int direction = 1 ); // direction =1 out, =0 in;
    //void init(QSerialPort * serial);
    void init( serial::Serial *serial );
    void send_packet( Serial_packet &packet );
    int receive_packet( Serial_packet &packet );

  public:
  private:
};

#endif // SERIAL_SERVICE_H
