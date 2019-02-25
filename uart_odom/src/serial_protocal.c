#include "serial_protocal.h"
#define IF_DEBUG 0
#define PACKET_END 0xff
#define PACKET_START 0xff

#if IF_DEBUG
#define PRINTF printf
#else
#define PRINTF //
#endif

// From: https://stackoverflow.com/questions/15169387/definitive-crc-for-c
static unsigned char const crc8_table[] = {
    0xea, 0xd4, 0x96, 0xa8, 0x12, 0x2c, 0x6e, 0x50, 0x7f, 0x41, 0x03, 0x3d,
    0x87, 0xb9, 0xfb, 0xc5, 0xa5, 0x9b, 0xd9, 0xe7, 0x5d, 0x63, 0x21, 0x1f,
    0x30, 0x0e, 0x4c, 0x72, 0xc8, 0xf6, 0xb4, 0x8a, 0x74, 0x4a, 0x08, 0x36,
    0x8c, 0xb2, 0xf0, 0xce, 0xe1, 0xdf, 0x9d, 0xa3, 0x19, 0x27, 0x65, 0x5b,
    0x3b, 0x05, 0x47, 0x79, 0xc3, 0xfd, 0xbf, 0x81, 0xae, 0x90, 0xd2, 0xec,
    0x56, 0x68, 0x2a, 0x14, 0xb3, 0x8d, 0xcf, 0xf1, 0x4b, 0x75, 0x37, 0x09,
    0x26, 0x18, 0x5a, 0x64, 0xde, 0xe0, 0xa2, 0x9c, 0xfc, 0xc2, 0x80, 0xbe,
    0x04, 0x3a, 0x78, 0x46, 0x69, 0x57, 0x15, 0x2b, 0x91, 0xaf, 0xed, 0xd3,
    0x2d, 0x13, 0x51, 0x6f, 0xd5, 0xeb, 0xa9, 0x97, 0xb8, 0x86, 0xc4, 0xfa,
    0x40, 0x7e, 0x3c, 0x02, 0x62, 0x5c, 0x1e, 0x20, 0x9a, 0xa4, 0xe6, 0xd8,
    0xf7, 0xc9, 0x8b, 0xb5, 0x0f, 0x31, 0x73, 0x4d, 0x58, 0x66, 0x24, 0x1a,
    0xa0, 0x9e, 0xdc, 0xe2, 0xcd, 0xf3, 0xb1, 0x8f, 0x35, 0x0b, 0x49, 0x77,
    0x17, 0x29, 0x6b, 0x55, 0xef, 0xd1, 0x93, 0xad, 0x82, 0xbc, 0xfe, 0xc0,
    0x7a, 0x44, 0x06, 0x38, 0xc6, 0xf8, 0xba, 0x84, 0x3e, 0x00, 0x42, 0x7c,
    0x53, 0x6d, 0x2f, 0x11, 0xab, 0x95, 0xd7, 0xe9, 0x89, 0xb7, 0xf5, 0xcb,
    0x71, 0x4f, 0x0d, 0x33, 0x1c, 0x22, 0x60, 0x5e, 0xe4, 0xda, 0x98, 0xa6,
    0x01, 0x3f, 0x7d, 0x43, 0xf9, 0xc7, 0x85, 0xbb, 0x94, 0xaa, 0xe8, 0xd6,
    0x6c, 0x52, 0x10, 0x2e, 0x4e, 0x70, 0x32, 0x0c, 0xb6, 0x88, 0xca, 0xf4,
    0xdb, 0xe5, 0xa7, 0x99, 0x23, 0x1d, 0x5f, 0x61, 0x9f, 0xa1, 0xe3, 0xdd,
    0x67, 0x59, 0x1b, 0x25, 0x0a, 0x34, 0x76, 0x48, 0xf2, 0xcc, 0x8e, 0xb0,
    0xd0, 0xee, 0xac, 0x92, 0x28, 0x16, 0x54, 0x6a, 0x45, 0x7b, 0x39, 0x07,
    0xbd, 0x83, 0xc1, 0xff
};

// Return the CRC-8 of data[0..len-1] applied to the seed crc. This permits the
// calculation of a CRC a chunk at a time, using the previously returned value
// for the next seed. If data is NULL, then return the initial seed. See the
// test code for an example of the proper usage.
unsigned crc8( unsigned crc, unsigned char const *data, size_t len )
{
    if ( data == NULL )
        return 0;
    crc &= 0xff;
    unsigned char const *end = data + len;
    while ( data < end )
    {
        crc = crc8_table[ crc ^ *data++ ];
    }
    return crc;
}

// crc8_slow() is an equivalent bit-wise implementation of crc8() that does not
// need a table, and which can be used to generate crc8_table[]. Entry k in the
// table is the CRC-8 of the single byte k, with an initial crc value of zero.
// 0xb2 is the bit reflection of 0x4d, the polynomial coefficients below x^8.
unsigned crc8_slow( unsigned crc, unsigned char const *data, size_t len )
{
    if ( data == NULL )
        return 0;
    crc = ~crc & 0xff;
    while ( len-- )
    {
        crc ^= *data++;
        for ( unsigned k = 0; k < 8; k++ )
            crc = crc & 1 ? ( crc >> 1 ) ^ 0xb2 : crc >> 1;
    }
    return crc ^ 0xff;
}

#define LONG_PROTOCAL 1

#if LONG_PROTOCAL

//protocal form
//ff id size_h size_l data_1 data_2 ... data_size checksum size_h size_l id ff
//sample: ff 00 01 66 00 01 00 ff
void make_packet( char *data, char *send_buffer, int packet_id, int data_size, int *packet_size )
{
    PRINTF( "Long protocal\n" );
    unsigned int i = 0;
    char         check_sum = 0xff;

    send_buffer[ 0 ] = 0xff;
    send_buffer[ 1 ] = packet_id;
    send_buffer[ 2 ] = ( char ) ( ( data_size&0x00ffff) >> 8 ) & 0xff ;
    send_buffer[ 3 ] = ( char ) ( data_size & 0x0000ff );

    for ( i = 0; i < data_size; i++ )
    {
        send_buffer[ 4 + i ] = data[ i ];
        //check_sum += ( data[ i ] & 0x01 );
    }

    check_sum = crc8( 0x00, &data[ 0 ], data_size );
    //check_sum = 0;
    PRINTF( "check_sum = %x\r\n", check_sum );
    //check_sum = crc8_slow(0x08, send_buffer[ 4 + i ] , data_size);

    send_buffer[ 4 + data_size + 0 ] = check_sum;
    send_buffer[ 4 + data_size + 1 ] = send_buffer[ 2 ];
    send_buffer[ 4 + data_size + 2 ] = send_buffer[ 3 ];
    send_buffer[ 4 + data_size + 3 ] = packet_id;
    send_buffer[ 4 + data_size + 4 ] = 0xFF;
    *packet_size = data_size + 9;
}

int onRec( char c, char *rec_buffer, int *current_index, int *packet_id, int *_data_size, char *rec_packet )
{
    int data_size = 0;
    int check_sum = 0;
    int start_index = 0;
    int i = 0;
    rec_buffer[ *current_index ] = c;
    if ( *current_index >= 6 && ( char ) PACKET_END == c )
    {
        *packet_id = ( char ) rec_buffer[ *current_index - 1 ];
        data_size = (rec_buffer[ *current_index - 2 ] &0xff) | ( rec_buffer[ *current_index - 3 ] << 8 );
        start_index = *current_index - data_size - 4 - 4;
        PRINTF( "--- %s ---\r\n", __FUNCTION__ );
        PRINTF( "current idx = %d \r\n", *current_index);
        PRINTF( "e_id = %d\r\n", *packet_id );
        PRINTF( "e_size = %d\r\n", data_size );
        //printf("id = %d\n", *packet_id);
        PRINTF( "s_id = %d\r\n", rec_buffer[ start_index + 1 ] );
        PRINTF( "s_size = %d\r\n", ( rec_buffer[ start_index + 2 ] << 8 ) | (rec_buffer[ start_index + 3 ])&0xff );
        PRINTF( "check_sum = %x\r\n", rec_buffer[ start_index + 4 + data_size ] & 0xff);
        if ( *current_index >= 6 - 1 + data_size )
        {
            //Length satisfied
            if ( (rec_buffer[ start_index + 2 ]&0xff) == ( data_size >> 8 ) &&
                 (rec_buffer[ start_index + 3 ]&0xff) == ( data_size & 0xff ) &&
                 rec_buffer[ start_index + 1 ] == *packet_id &&
                 rec_buffer[ start_index + 0 ] == ( char ) PACKET_START )
            {
                //protocol valid
                for ( i = 0; i < data_size; i++ )
                {
                    rec_packet[ i ] = rec_buffer[ start_index + 4 + i ]& 0xff;
                    //check_sum += ( rec_packet[ i ] & 0x01 );
                }

                check_sum = crc8( 0x00, &rec_packet[ 0 ], data_size );
                PRINTF( "computed check_sum = %x\r\n", check_sum );
                //check_sum = crc8_slow( check_sum, rec_packet[ i ], data_size );

                if ( check_sum == (rec_buffer[ start_index + 4 + data_size ] & 0xff)  )
                {
                    //Here set to zero.
                    //printf("Current index =  %d\r\n", *current_index);
                    *current_index = 0;
                    *_data_size = data_size;
                    /*printf("Get the message\r\n");
                    for (i = 0; i < data_size; i++)
                    cout << int(rec_packet[i]) << ",";
                    cout << endl;*/
                    return 1;
                    }
            }
        }
    }
    *current_index = ( *current_index + 1 );
    return 0;
}

#else
//protocal form
//ff id size data_1 data_2 ... data_size checksum size id ff
//sample: ff 00 01 66 00 01 00 ff
void make_packet( char *data, char *send_buffer, int packet_id, int data_size, int *packet_size )
{
    unsigned int i = 0;
    char         check_sum = 0;
    send_buffer[ 0 ] = 0xff;
    send_buffer[ 1 ] = packet_id;
    send_buffer[ 2 ] = ( char ) ( data_size );
    for ( i = 0; i < data_size; i++ )
    {
        send_buffer[ 3 + i ] = data[ i ];
        check_sum += ( data[ i ] & 0x01 );
    }
    send_buffer[ 3 + data_size + 0 ] = check_sum;
    send_buffer[ 3 + data_size + 1 ] = data_size;
    send_buffer[ 3 + data_size + 2 ] = packet_id;
    send_buffer[ 3 + data_size + 3 ] = 0xFF;
    *packet_size = data_size + 7;
}

int onRec( char c, char *rec_buffer, int *current_index, int *packet_id, int *_data_size, char *rec_packet )
{
    int data_size = 0;
    int check_sum = 0;
    int start_index = 0;
    int i = 0;
    rec_buffer[ *current_index ] = c;
    if ( *current_index >= 6 && ( char ) PACKET_END == c )
    {
        *packet_id = ( char ) rec_buffer[ *current_index - 1 ];
        data_size = rec_buffer[ *current_index - 2 ];
        start_index = *current_index - data_size - 3 - 3;
        PRINTF( "--- %s ---\r\n", __FUNCTION__ );
        PRINTF( "e_id = %x\r\n", *packet_id );
        PRINTF( "e_size = %x\r\n", data_size );
        //printf("id = %d\n", *packet_id);
        PRINTF( "s_id = %x\r\n", rec_buffer[ start_index + 1 ] );
        PRINTF( "s_size = %x\r\n", rec_buffer[ start_index + 2 ] );
        PRINTF( "check_sum = %x\r\n", rec_buffer[ start_index + 4 + data_size ] );
        if ( *current_index >= 6 - 1 + data_size )
        {
            //Length satisfied
            if ( rec_buffer[ start_index + 2 ] == data_size &&
                 rec_buffer[ start_index + 1 ] == *packet_id &&
                 rec_buffer[ start_index + 0 ] == ( char ) PACKET_START )
            {
                //protocol valid
                for ( i = 0; i < data_size; i++ )
                {
                    rec_packet[ i ] = rec_buffer[ start_index + 3 + i ];
                    check_sum += ( rec_packet[ i ] & 0x01 );
                }
                if ( check_sum == rec_buffer[ start_index + 3 + data_size ] )
                {
                    //Here set to zero.
                    *current_index = 0;
                    *_data_size = data_size;
                    /*printf("Get the message\r\n");
                    for (i = 0; i < data_size; i++)
                    cout << int(rec_packet[i]) << ",";
                    cout << endl;*/
                    return 1;
                }
            }
        }
    }
    *current_index = ( *current_index + 1 );
    return 0;
}
#endif
