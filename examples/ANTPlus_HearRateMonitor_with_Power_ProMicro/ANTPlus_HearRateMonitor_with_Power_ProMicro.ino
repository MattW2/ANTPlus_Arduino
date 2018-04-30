/* Example for the ANT+ Library @ https://github.com/brodykenrick/ANTPlus_Arduino
Copyright 2013 Brody Kenrick.

Developed for http://retrorunnerreadout.blogspot.com

Interfacing of Garmin ANT+ device (via a cheap Nordic nRF24AP UART module) to an Arduino.

Opens an ANT+ channel listening for HRM. Prints out computed hear rate.

Hardware
An Arduino Pro Mini 3v3 connected to this nRF24AP2 module : http://www.goodluckbuy.com/nrf24ap2-networking-module-zigbee-module-with-ant-transceiver-.html

The connector on nRF24AP2 board is (looking from the front, pin 1 is marked []):
[]GND(=VSS) | VDD(=3.3 volts)
UART_TX   | UART_RX
!(SUSP)   | SLEEP
RTS       | !(RESET)

Wiring to the Arduino Pro Mini 3v3 can be seen in 'antplus' below.
*/

#include <Arduino.h>

//#define NDEBUG
#define ANTPLUS_DEBUG //output the ANTPLUS Packets
#define ANTPLUS_MSG_STR_DECODE // decode ANTPLUS Packets

#define __ASSERT_USE_STDERR
#include <assert.h>

#define ANTPLUS_ON_HW_UART //using a Arduino Pro Micro with "Real" USB serial. This is on Serial, nRF24AP2 will be on Serial1

#if !defined(ANTPLUS_ON_HW_UART)
#include <SoftwareSerial.h>
#endif

#include <ANTPlus.h>

#define USE_SERIAL_CONSOLE //!<Use the hardware serial as the console. This needs to be off if using hardware serial for driving the ANT+ module.

#if defined(NDEBUG)
#undef USE_SERIAL_CONSOLE
#endif

//Logging macros
//********************************************************************
#define SERIAL_DEBUG

#if !defined(USE_SERIAL_CONSOLE)
//Disable logging under these circumstances
#undef SERIAL_DEBUG
#endif

//F() stores static strings that come into existence here in flash (makes things a bit more stable)
#ifdef SERIAL_DEBUG

#define SERIAL_DEBUG_PRINT(x)  	    (Serial.print(x))
#define SERIAL_DEBUG_PRINTLN(x)	    (Serial.println(x))
#define SERIAL_DEBUG_PRINT_F(x)  	  (Serial.print(F(x)))
#define SERIAL_DEBUG_PRINTLN_F(x)	  (Serial.println(F(x)))
#define SERIAL_DEBUG_PRINT2(x,y)    (Serial.print(x,y))
#define SERIAL_DEBUG_PRINTLN2(x,y)	(Serial.println(x,y))

#else

#define SERIAL_DEBUG_PRINT(x)
#define SERIAL_DEBUG_PRINTLN(x)
#define SERIAL_DEBUG_PRINT_F(x)
#define SERIAL_DEBUG_PRINTLN_F(x)
#define SERIAL_DEBUG_PRINT2(x,y)
#define SERIAL_DEBUG_PRINTLN2(x,y)

#endif

//********************************************************************

#define ANTPLUS_BAUD_RATE (9600) //!< The moduloe I am using is hardcoded to this baud rate.


//The ANT+ network keys are not allowed to be published so they are stripped from here.
//They are available in the ANT+ docs at thisisant.com
#define ANT_SENSOR_NETWORK_KEY {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}
#define ANT_GPS_NETWORK_KEY    {0xE8, 0xE4, 0x21, 0x3B, 0x55, 0x7A, 0x67, 0xC1}

#if !defined( ANT_SENSOR_NETWORK_KEY ) || !defined(ANT_GPS_NETWORK_KEY)
#error "The Network Keys are missing. Better go find them by signing up at thisisant.com"
#endif

// ****************************************************************************
// ******************************  GLOBALS  ***********************************
// ****************************************************************************

//Arduino Pro Mini pins to the nrf24AP2 modules pinouts
static const int RTS_PIN      = 2; //!< RTS on the nRF24AP2 module
static const int RTS_PIN_INT  = 1; //!< The interrupt equivalent of the RTS_PIN


#if !defined(ANTPLUS_ON_HW_UART)
static const int TX_PIN       = 8; //Using software serial for the UART
static const int RX_PIN       = 9; //Ditto
static SoftwareSerial ant_serial(TX_PIN, RX_PIN); // RXArd, TXArd -- Arduino is opposite to nRF24AP2 module
#else
//Using Hardware Serial (0,1) instead
#endif

static ANTPlus        antplus   = ANTPlus(RTS_PIN, 6/*SUSPEND*/, 4/*SLEEP*/, 9/*RESET*/ );

//ANT Channel config for HRM
static ANT_Channel hrm_channel =
{
  1, //Channel Number
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_HRM,
  DEVCE_SENSOR_FREQ,
  DEVCE_HRM_LOWEST_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};

static ANT_Channel power_channel =
{
  2, //Channel Number
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_POWER,
  DEVCE_SENSOR_FREQ,
  DEVCE_POWER_LOWEST_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};

static ANT_Channel fitness_channel =
{
  0, //Channel Number
  PUBLIC_NETWORK,
  DEVCE_TIMEOUT,
  DEVCE_TYPE_FITNESS_EQUIPMENT,
  DEVCE_SENSOR_FREQ,
  DEVCE_FITNESS_LOWEST_RATE,
  ANT_SENSOR_NETWORK_KEY,
  ANT_CHANNEL_ESTABLISH_PROGRESSING,
  FALSE,
  0, //state_counter
};



volatile int rts_ant_received = 0; //!< ANT RTS interrupt flag see isr_rts_ant()

// **************************************************************************************************
// *********************************  ISRs  *********************************************************
// **************************************************************************************************

//! Interrupt service routine to get RTS from ANT messages
void isr_rts_ant()
{
  rts_ant_received = 1;
  Serial.println("ISR_ANT");
}

// **************************************************************************************************
// ***********************************  ANT+  *******************************************************
// **************************************************************************************************

void process_packet( ANT_Packet * packet )
{
#if defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
  //This function internally uses Serial.println
  //Only use it if the console is available and if the ANTPLUS library is in debug mode
  antplus.printPacket( packet, false );
#endif //defined(USE_SERIAL_CONSOLE) && defined(ANTPLUS_DEBUG)
   
  switch ( packet->msg_id )
  {
    case MESG_BROADCAST_DATA_ID:
    {
      const ANT_Broadcast * broadcast = (const ANT_Broadcast *) packet->data;
      SERIAL_DEBUG_PRINT_F( "CHAN " );
      SERIAL_DEBUG_PRINT( broadcast->channel_number );
      SERIAL_DEBUG_PRINT_F( " " );
      const ANT_DataPage * dp = (const ANT_DataPage *) broadcast->data;
      
      //Update received data
      if( broadcast->channel_number == hrm_channel.channel_number )
      {
        hrm_channel.data_rx = true;
        //To determine the device type -- and the data pages -- check channel setups
        if(hrm_channel.device_type == DEVCE_TYPE_HRM)
        {
            switch(dp->data_page_number)
            {
              case DATA_PAGE_HEART_RATE_0:
              case DATA_PAGE_HEART_RATE_0ALT:
              case DATA_PAGE_HEART_RATE_1:
              case DATA_PAGE_HEART_RATE_1ALT:
              case DATA_PAGE_HEART_RATE_2:
              case DATA_PAGE_HEART_RATE_2ALT:
              case DATA_PAGE_HEART_RATE_3:
              case DATA_PAGE_HEART_RATE_3ALT:
              case DATA_PAGE_HEART_RATE_4:
              case DATA_PAGE_HEART_RATE_4ALT:
              {
                //As we only care about the computed heart rate
                // we use a same struct for all HRM pages
                const ANT_HRMDataPage * hrm_dp = (const ANT_HRMDataPage *) dp;
                SERIAL_DEBUG_PRINT_F( "HR[any_page] : BPM = ");
                SERIAL_DEBUG_PRINTLN( hrm_dp->computed_heart_rate );
              }
              break;
  
              default:
                  SERIAL_DEBUG_PRINT_F(" HRM DP# ");
                  SERIAL_DEBUG_PRINTLN( dp->data_page_number );
                break;
            }
        }
    }
    
      if( broadcast->channel_number == power_channel.channel_number )
      {
        power_channel.data_rx = true;
        //To determine the device type -- and the data pages -- check channel setups
        if(power_channel.device_type == DEVCE_TYPE_POWER)
        {
            switch(dp->data_page_number)	
			{
				case DATA_PAGE_POWER_POWER_ONLY:
				{
				  const ANT_Power_PowerOnly_DataPage * power_dp = (const ANT_Power_PowerOnly_DataPage *) dp;
				  byte Instant_Power_LSB = power_dp->instantaneous_power_LSB;
				  byte Instant_Power_MSB = power_dp->instantaneous_power_MSB;
				  unsigned int Instant_Power = (Instant_Power_MSB << 8) + Instant_Power_LSB;
				  SERIAL_DEBUG_PRINT_F( "Power[Power-Only] : Power = ");
				  SERIAL_DEBUG_PRINTLN(Instant_Power);
				}
				break;
				
				case DATA_PAGE_POWER_WHEEL_TORQUE:
				
				default:
                  SERIAL_DEBUG_PRINT_F(" POWER DP# ");
                  SERIAL_DEBUG_PRINTLN( dp->data_page_number );
                break;				

			}					  
		}
	  }
	  
      if( broadcast->channel_number == fitness_channel.channel_number )
      {
        fitness_channel.data_rx = true;
        //To determine the device type -- and the data pages -- check channel setups
        if(fitness_channel.device_type == DEVCE_TYPE_FITNESS_EQUIPMENT)
        {
            switch(dp->data_page_number)	
			{
				case DATA_PAGE_FITNESS_BASIC_RESISTANCE:
				{
					const ANT_Fitness_Basic_Resistance_DataPage * resistance_dp = (const ANT_Fitness_Basic_Resistance_DataPage *) dp; 
					byte Total_Resistance = resistance_dp->Total_Resistance;
					SERIAL_DEBUG_PRINT_F( "Resistance[Basic] : = ");
					SERIAL_DEBUG_PRINTLN(Total_Resistance);
				}
				
				/*
				case 
				
				
				
				{
				  const ANT_Power_PowerOnly_DataPage * power_dp = (const ANT_Power_PowerOnly_DataPage *) dp;
				  byte Instant_Power_LSB = power_dp->instantaneous_power_LSB;
				  byte Instant_Power_MSB = power_dp->instantaneous_power_MSB;
				  unsigned int Instant_Power = (Instant_Power_MSB << 8) + Instant_Power_LSB;
				  SERIAL_DEBUG_PRINT_F( "Power[Power-Only] : Power = ");
				  SERIAL_DEBUG_PRINTLN(Instant_Power);
				}
				break;
				
				case DATA_PAGE_POWER_WHEEL_TORQUE:
				
				default:
                  SERIAL_DEBUG_PRINT_F(" POWER DP# ");
                  SERIAL_DEBUG_PRINTLN( dp->data_page_number );
                break;				
*/	
			}			
			}
	  }	  
	  
	  
	  
	  
	  
	
	
	
	break;
    }
    
    default:
      SERIAL_DEBUG_PRINTLN_F("Non-broadcast data received.");
      break;
  }
}




// **************************************************************************************************
// ************************************  Setup  *****************************************************
// **************************************************************************************************
void setup()
{
	delay(5000);
	SERIAL_DEBUG_PRINTLN_F("press any key to go");
	while(Serial.available() == 0){}
	
#if defined(USE_SERIAL_CONSOLE)
  Serial.begin(115200); 
#endif //defined(USE_SERIAL_CONSOLE)

  SERIAL_DEBUG_PRINTLN("ANTPlus HRM Test!");
  SERIAL_DEBUG_PRINTLN_F("Setup.");

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config.");

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each valid ANT message received from the host at the chip
  attachInterrupt(RTS_PIN_INT, isr_rts_ant, RISING);


#if defined(ANTPLUS_ON_HW_UART)
  //Using hardware UART
  Serial1.begin(ANTPLUS_BAUD_RATE); 
  antplus.begin( Serial1 );
#else
  //Using soft serial
  ant_serial.begin( ANTPLUS_BAUD_RATE ); 
  antplus.begin( ant_serial );
#endif

  SERIAL_DEBUG_PRINTLN_F("ANT+ Config Finished.");
  SERIAL_DEBUG_PRINTLN_F("Setup Finished.");
}

// **************************************************************************************************
// ************************************  Loop *******************************************************
// **************************************************************************************************

void loop()
{
  byte packet_buffer[ANT_MAX_PACKET_LEN];
  ANT_Packet * packet = (ANT_Packet *) packet_buffer;
  MESSAGE_READ ret_val = MESSAGE_READ_NONE;
  
  if(rts_ant_received == 1)
  {
    SERIAL_DEBUG_PRINTLN_F("Received RTS Interrupt. ");
    antplus.rTSHighAssertion();
    //Clear the ISR flag
    rts_ant_received = 0;
  }

  //Read messages until we get a none
  while( (ret_val = antplus.readPacket(packet, ANT_MAX_PACKET_LEN, 0 )) != MESSAGE_READ_NONE )
  {
    if((ret_val == MESSAGE_READ_EXPECTED) || (ret_val == MESSAGE_READ_OTHER))
    {
      SERIAL_DEBUG_PRINT_F( "ReadPacket success = " );
      if( (ret_val == MESSAGE_READ_EXPECTED) )
      {
        SERIAL_DEBUG_PRINTLN_F( "Expected packet" );
      }
      else
      if( (ret_val == MESSAGE_READ_OTHER) )
      {
        SERIAL_DEBUG_PRINTLN_F( "Other packet" );
      }
      process_packet(packet);
    }
    else
    {
      SERIAL_DEBUG_PRINT_F( "ReadPacket Error = " );
      SERIAL_DEBUG_PRINTLN( ret_val );
      if(ret_val == MESSAGE_READ_ERROR_MISSING_SYNC)
      {
        //Nothing -- allow a re-read to get back in sync
      }
      else
      if(ret_val == MESSAGE_READ_ERROR_BAD_CHECKSUM)
      {
        //Nothing -- fully formed package just bit errors
      }
      else
      {
        break;
      }
    }
  }

  /*
//setup heart rate
  if(hrm_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
  {
    antplus.progress_setup_channel( &hrm_channel );
    if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
    {
      SERIAL_DEBUG_PRINT( hrm_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - Established." );
    }
    else
    if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
    {
      SERIAL_DEBUG_PRINT( hrm_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - Progressing." );
    }
    else
    {
      SERIAL_DEBUG_PRINT( hrm_channel.channel_number );
      SERIAL_DEBUG_PRINTLN_F( " - ERROR!" );
    }
  }
  */
 
 
 /*
//setup power
//if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
//	{
	  if(power_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
	  {
		antplus.progress_setup_channel( &power_channel );
		if(power_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
		{
		  SERIAL_DEBUG_PRINT( power_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Power Established." );
		}
		else
		if(power_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
		{
		  SERIAL_DEBUG_PRINT( power_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Power Progressing." );
		}
		else
		{
		  SERIAL_DEBUG_PRINT( power_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Power ERROR!" );
		}
	  }  
//	}
*/

//setup fitness
//if(hrm_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
//	{
	  if(fitness_channel.channel_establish != ANT_CHANNEL_ESTABLISH_COMPLETE)
	  {
		antplus.progress_setup_channel( &fitness_channel );
		if(fitness_channel.channel_establish == ANT_CHANNEL_ESTABLISH_COMPLETE)
		{
		  SERIAL_DEBUG_PRINT( fitness_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Fitness Established." );
		}
		else
		if(fitness_channel.channel_establish == ANT_CHANNEL_ESTABLISH_PROGRESSING)
		{
		  SERIAL_DEBUG_PRINT( fitness_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Fitness Progressing." );
		}
		else
		{
		  SERIAL_DEBUG_PRINT( fitness_channel.channel_number );
		  SERIAL_DEBUG_PRINTLN_F( " - Fitness ERROR!" );
		}
	  }  
//	}


}


