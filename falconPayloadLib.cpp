#include "falconPayloadLib.h"
#include "string.h"




#define ss 18
#define rst 14
#define dio0 26
#define dio1 33



SX1276 radio = new Module(ss, dio0, rst, dio1);




uint8_t check_sum_hesapla( const uint8_t * const ptVeri , const uint8_t START_IDX , const uint8_t END_IDX )
{
    uint32_t check_sum = 0;
     
    for( uint8_t i = START_IDX ; i < END_IDX ; i++ )
    {
        check_sum += ptVeri[i];
    }
    return ( uint8_t ) ( check_sum % 256 ) ;
}


void veriPaketle ( dataPaket_t * const pkt , const  dataStruct_t * const data )
{
    memcpy ( &( pkt->data ) , data , sizeof( dataStruct_t ) );
    pkt->u8_crc_data = check_sum_hesapla( pkt->u8_array , 4 , sizeof( dataPaket_t )-3 );
}
void verileriYolla (  uint8_t * ptVeri , const uint8_t veriLength )
{
    // radio.transmit( ptVeri , veriLength );

    radio.startTransmit( ptVeri , veriLength );
    Serial.println("A [PAYLOAD] package has been sended.");
}

void initDataPaket( dataPaket_t * const pkt , const uint8_t WHICH_NODE )
{
    memset( pkt->u8_array , 0 , sizeof( dataPaket_t ) );
    pkt->u32_start_header       = 0x5254FFFF; // 4
    pkt->u8_node_information    = WHICH_NODE ;  // 1
    pkt->u8_package_length      = sizeof( dataPaket_t );    
    pkt->u16_end_header         = 0x0A0D;   // 2
}



