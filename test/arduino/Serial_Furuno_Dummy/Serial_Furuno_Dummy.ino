/*
 Serial Furuno Dummy
 */

/* ================== START OF USER DEFINES =====================  */

/* \brief Defines the number of messages to transmit. */
#define MAX_LOOP_COUNT   100

/* \brief Defines the number of messages to transmit.
   
   \warning IMPORTANT! Keep this value in sync with the
            corresponding baud rate! E.g. when setting
            this value to 5ms (= 200Hz) you should set
            the baudrate to 1000000 (= 1MBit/s).
            Otherwise the data rate is to slow for the
            TX intervall.
*/
#define TX_INTERVALL_MS    1000

/* \brief Defines the baudrate for the serial port.
          Up to 1MBit/s (= 1000000) is supported.
          
   \warning IMPORTANT! Keep this value in sync with the
            TX intervall!
*/
#define SERIAL_BAUD_RATE   38400

/* ================== END OF USER DEFINES =====================  */


int i;
int loopCounter;

char furuno[] = 
"$GNRMC,000359.000,V,0000.0000,N,00000.0000,E,0.00,0.00,220899,,,N,V*15\r\n"     
"$GNGNS,000359.000,0000.0000,N,00000.0000,E,NNN,00,,-18.0,18.0,,,V*60\r\n"        
"$GNGGA,000359.000,0000.0000,N,00000.0000,E,0,00,,-18.0,M,18.0,M,,*7F\r\n"
"$GNGLL,0000.0000,N,00000.0000,E,000359.000,V,N*50\r\n"  
"$GNVTG,0.00,T,,M,0.00,N,0.00,K,N*2C\r\n"
"$GNGSA,A,1,,,,,,,,,,,,,,,,1*1D\r\n"
"$GPZDA,000359.000,22,08,1999,+00,00*72\r\n"
"$GPGSV,1,1,00,,,,1*48\r\n"
"$GLGSV,1,1,00,,,,1*54\r\n"
"$PERDCRW,TPS1,19990822000359,0,00000000000000,+17,+00,0,+00000.000,+4266*2E\r\n"
"$PERDCRX,TPS2,0,2,0,500,+000000,1,1,9999,+0.000,0000,08388608,+034846*09\r\n"
"$PERDCRY,TPS3,1,1000,000,000000,000000,2,2,00,0x20002F86,0x00000030*78\r\n"
"$PERDCRZ,TPS4,0,1,01,01,+000000000,+00000,0000,0000000,000000,+000000*1D\r\n";

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  i = 0;
  loopCounter = 0;
}

void loop() // run over and over
{
  delay(TX_INTERVALL_MS);
  
  if(loopCounter <= MAX_LOOP_COUNT)
  {
    for(i=0; i < (sizeof(furuno) / sizeof(char)); i++ )
    {
      Serial.write( furuno[i] );
    }
  
    loopCounter++;
  }
}

