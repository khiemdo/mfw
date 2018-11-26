/*
 Serial Echo

 Receives from the hardware serial, sends out to hardware serial (echo).
 Connecting to Arduino through USB COM port (by using any serial COM tool, e.g. HTerm.exe) 
 allows monitoring of received/transmitted bytes.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 * Baudrate: 115200 8-N-1

 */

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
}

void loop() // run over and over
{
  if (Serial.available())
  {
    Serial.write(Serial.read());
  }
}

