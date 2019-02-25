#include "serial_service.h"

using namespace std;

Protocal_to_mcu::Protocal_to_mcu()
{
    //InitializeCriticalSection(&m_cs_lock);
}

Protocal_to_mcu::~Protocal_to_mcu()
{
}

void Protocal_to_mcu::init( serial::Serial *serial )
{
    m_serial = serial;
}

void Protocal_to_mcu::display( int count, int direction )
{
    cout << __FUNCTION__ << "  ";
    for ( int i = 0; i < count; i++ )
    {
        if ( direction == 1 )
        {
            printf( "%2x, ", m_send_buffer[ i ] );
        }
        else
        {
            printf( "%2x, ", m_rec_buffer[ i ] );
        }
    }
}

void Protocal_to_mcu::send_packet( Serial_packet &packet )
{
    m_busy = 1;
    //Protocol_auto_lock auto_lock(m_cs_lock);
    std::unique_lock< mutex > lock( m_send_mutex );
    packet.direction = Serial_packet::e_direction_out;
    //packet.data_length = packet.packet_length;
    //    char *send_data = ( char * ) malloc( packet.data_length * sizeof( char ) );
    //    for ( int i = 0; i < packet.data_length; i++ )
    //    {
    //        send_data[ i ] = packet.data[ i ];
    //        //cout << (int)send_data[i] << endl;
    //    }

    make_packet( packet.data, m_send_buffer, packet.id, packet.data_length, &packet.packet_length );
    m_serial->write( ( uint8_t * ) m_send_buffer, packet.packet_length );
    m_busy = 0;
    return;
    //    for ( int i = 0; i < packet.packet_length; i++ )
    //    {
    //        m_serial->putChar( m_send_buffer[ i ] );
    //        m_ser
    //    }
    m_serial->flush();
    //    m_serial->waitForBytesWritten( 1 );

    //    delete send_data;
}

int Protocal_to_mcu::receive_packet( Serial_packet &packet )
{
    //    std::unique_lock< mutex > lock( m_rec_mutex );
    Serial_packet packet_temp;
    int           has_data = 0;
    int           i = 0;
    int           buffer_data_size;
    //    if ( m_serial->isOpen() )
    if ( 1 )
    {
        packet_temp.direction = Serial_packet::e_direction_in;
        if ( m_last_buffer_size == 0 )
        {
            buffer_data_size = m_serial->read( ( uint8_t * ) m_temp_buffer, TEMP_BUFFER_SIZE );
        }
        else
        {
            buffer_data_size = m_last_buffer_size;
        }
        for (  i = 0; i < buffer_data_size; i++ )
        {
            //printf("%x ", serial_data[i] & 0xff );
            if ( m_current_rec_index > MAX_REC_BUFFER )
            {
                printf( " m_current_rec_index > MAX_REC_BUFFER" );
                m_current_rec_index = 0;
            }
            if ( onRec( m_temp_buffer[ i ], m_rec_buffer, &m_current_rec_index, &packet_temp.id, &packet_temp.data_length, packet_temp.data ) )
            {
                if(m_debug_info)
                    cout << "Serial size = " << buffer_data_size << endl;
                //cout << "Serial size = " << serial_data.size() << endl;
                //get_data = 1;
                //rec_buffer_count++;
                has_data = 1;
                packet = packet_temp;

                if (m_mode == e_mode_return_when_get_packet)
                {
                    m_last_buffer_size = 0;
                    int current_inx =i;
                    //memcpy( &m_temp_buffer[ 0 ], &m_temp_buffer[ i ], m_last_buffer_size );
                    for(;i< buffer_data_size;i++)
                    {
                        m_temp_buffer[ i-current_inx ] = m_temp_buffer[ i ];
                        m_last_buffer_size++;
                    }
                    break;
                }

                //break;
                //return 1;
            }
        }
        if (has_data == 0 )
        {
            m_last_buffer_size = 0;
        }
    }
    else
    {
        return has_data;
    }
    return has_data;
}
