#include "Arduino.h"
#include "oregon.hpp"


#define SEND_HIGH() digitalWrite(OregonTxPin, HIGH)
#define SEND_LOW() digitalWrite(OregonTxPin, LOW)

#define RF_BIT_TIME  512
#define RF_2BIT_TIME  2*RF_BIT_TIME

Oregon::Oregon(void)
{

}

void Oregon::begin(byte tx_pin)
{
  OregonTxPin = tx_pin;
  pinMode(OregonTxPin, OUTPUT);
  digitalWrite(OregonTxPin, LOW);
}

void Oregon::end()
{
  pinMode(OregonTxPin, INPUT);
  digitalWrite(OregonTxPin, LOW);
}


void Oregon::send_temperature(uint8_t ch, uint8_t id, float temperature, byte battery)
{
  DEBUG_OREGON_PRINTLN(F("- Sending temperature"));
  OregonMessageSize = 8;
  // Set type
  OregonMessage[0] = 0xEA;
  OregonMessage[1] = 0x4C;
  // Channel
  OregonMessage[2] = ch;
  // Id
  OregonMessage[3] = id;
  // Battery info
  if (!battery) OregonMessage[4] = 0x0C;
  else OregonMessage[4] = 0x00;
  setTemperature(temperature);
  calculateAndSetChecksum();
  sendMessage();
}


void Oregon::send_temperature_from_ds18(uint8_t ch, uint8_t id, int temperature, byte battery)
{
  DEBUG_OREGON_PRINTLN(F("- Sending temperature from DS18 raw"));
  OregonMessageSize = 8;
  // Set type
  OregonMessage[0] = 0xEA;
  OregonMessage[1] = 0x4C;
  // Channel
  OregonMessage[2] = ch;
  // Id
  OregonMessage[3] = id;
  // Battery info
  if (!battery) OregonMessage[4] = 0x0C;
  else OregonMessage[4] = 0x00;
  setTemperature_ds18(temperature);
  calculateAndSetChecksum();
  sendMessage();
}


void Oregon::send_temperature_hum(uint8_t ch, uint8_t id, float temperature, byte hum, byte battery)
{
  DEBUG_OREGON_PRINTLN(F("- Sending temperature & hum"));
  OregonMessageSize = 9;
  OregonMessage[0] = 0x1A;
  OregonMessage[1] = 0x2D;
  // Channel
  OregonMessage[2] = ch;
  // Id
  OregonMessage[3] = id;
  // Battery info
  if (!battery) OregonMessage[4] = 0x0C;
  else OregonMessage[4] = 0x00;
  setTemperature(temperature);
  setHumidity(hum);
  calculateAndSetChecksum();
  sendMessage();
}

void Oregon::sendMessage()
{
  DEBUG_OREGON_PRINTLN(F("- Oregon Message:"));
  for (byte i = 0; i < OregonMessageSize; ++i)   {
    DEBUG_OREGON_PRINT(OregonMessage[i] >> 4, HEX);
    DEBUG_OREGON_PRINT(OregonMessage[i] & 0x0F, HEX);
  }
  DEBUG_OREGON_PRINTLN();
  DEBUG_OREGON_PRINTLN(F("- Oregon Message (reverse):"));
  for (byte i = 0; i < OregonMessageSize; ++i)   {
    DEBUG_OREGON_PRINT(OregonMessage[OregonMessageSize - i - 1] >> 4, HEX);
    DEBUG_OREGON_PRINT(OregonMessage[OregonMessageSize - i - 1] & 0x0F, HEX);
  }
  DEBUG_OREGON_PRINTLN();
  // Send the Message over RF
  sendOregon();
  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(RF_2BIT_TIME * 8);
  // Send a copie of the first message. The v2.1 protocol send the
  // message two RF_BIT_TIME
  sendOregon();
}

/**
   \brief    Set the sensor temperature
   \param    temp       the temperature
*/
void Oregon::setTemperature(float temp)
{
  // Set temperature sign
  if (temp < 0)
  {
    OregonMessage[6] = 0x08;
    temp *= -1;
  }
  else
  {
    OregonMessage[6] = 0x00;
  }
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt / 10 - (float)td) * 10);
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);

  // Set temperature decimal part
  OregonMessage[5] = (td << 4);
  OregonMessage[5] |= tf;
  // Set temperature float part
  OregonMessage[4] |= (tempFloat << 4);
}


/*
 * 
 * 
 * 
uint8_t bcd2dec2(uint8_t n)
{
  return n - 6 * (n/16); 
}
*/
uint8_t Oregon::dec2bcd2(uint8_t n)
{
  uint16_t a = n;
  byte b = (a*103) >> 10;  // this equals:  b = a/10; 
  return  n + b*6;
}

/**
   \brief    Set the sensor temperature
   \param    temp       the temperature in 1/128°C raw value from DS18B20
*/
void Oregon::setTemperature_ds18(int16_t raw_temperature)
{
  // raw temperature input is in 1/128°C
  int td;
  if (raw_temperature < 0)
  {
    OregonMessage[6] = 0x08;
    raw_temperature *= -1;
  }
  else
  {
    OregonMessage[6] = 0x00;
  }
  // Convert temperature in 1/10 of °C ( *10/128 )
  raw_temperature = (10*raw_temperature)>>7;
  td = raw_temperature /10;
  OregonMessage[5] = dec2bcd2(td);
  OregonMessage[4] |= dec2bcd2(raw_temperature-(10*td))<<4;
}

/**
   \brief    Set the sensor humidity
   \param    hum        the humidity
*/
void Oregon::setHumidity(byte hum)
{
  OregonMessage[7] = (hum / 10);
  OregonMessage[6] |= (hum - OregonMessage[7] * 10) << 4;
}

/**
   \brief    Set the sensor temperature
   \param    temp       the temperature
*/
void Oregon::setPressure(float pres)
{
  if ((pres > 850) && (pres < 1100)) {
    OregonMessage[8] = (int)round(pres) - 856;
    OregonMessage[9] = 0xC0;
  }
}

/**
   \brief    Sum data for checksum
   \param    count      number of bit to sum
*/
int Oregon::Sum(byte count)
{
  int s = 0;
  for (byte i = 0; i < count; i++) {
    s += (OregonMessage[i] & 0xF0) >> 4;
    s += (OregonMessage[i] & 0xF);
  }
  if (int(count) != count)
    s += (OregonMessage[count] & 0xF0) >> 4;
  return s;
}

/**
   \brief    Calculate checksum
*/
void Oregon::calculateAndSetChecksum()
{
  if (OregonMessageSize == 8) {
    int s = ((Sum(6) + (OregonMessage[6] & 0xF) - 0xa) & 0xff);
    OregonMessage[6] |=  (s & 0x0F) << 4;
    OregonMessage[7] =  (s & 0xF0) >> 4;
  } else if (OregonMessageSize == 9) {
    OregonMessage[8] = ((Sum(8) - 0xa) & 0xFF);
  } else if (OregonMessageSize == 11) {
    OregonMessage[10] = ((Sum(10) - 0xa) & 0xFF);
  }
}

/**
   \brief    Send logical "0" over RF
   \details  azero bit be represented by an off-to-on transition
   \         of the RF signal at the middle of a clock period.
   \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
void Oregon::sendZero(void)
{
  SEND_HIGH();
  delayMicroseconds(RF_BIT_TIME);
  SEND_LOW();
  delayMicroseconds(RF_2BIT_TIME);
  SEND_HIGH();
  delayMicroseconds(RF_BIT_TIME);
}

/**
   \brief    Send logical "1" over RF
   \details  a one bit be represented by an on-to-off transition
   \         of the RF signal at the middle of a clock period.
   \         Remenber, the Oregon v2.1 protocol add an inverted bit first
*/
void Oregon::sendOne(void)
{
  SEND_LOW();
  delayMicroseconds(RF_BIT_TIME);
  SEND_HIGH();
  delayMicroseconds(RF_2BIT_TIME);
  SEND_LOW();
  delayMicroseconds(RF_BIT_TIME);
}

/**
  Send a bits quarter (4 bits = MSB from 8 bits value) over RF

  @param data Source data to process and sent
*/

/**
   \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
   \param    data   Data to send
*/
void Oregon::sendQuarterMSB(const byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}

/**
   \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
   \param    data   Data to send
*/
void Oregon::sendQuarterLSB(const byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}


/**
   \brief    Send an Oregon message
   \param    data   The Oregon message
*/
void Oregon::sendOregon(void)
{
  //sendPreamble();
  sendQuarterLSB(0xFF);
  sendQuarterMSB(0xFF);
  sendQuarterLSB(0xFF);
  sendQuarterMSB(0xFF);

  for (byte i = 0; i < OregonMessageSize; ++i)
  {
    sendQuarterLSB(OregonMessage[i]);
    sendQuarterMSB(OregonMessage[i]);
  }
  //sendPostamble();
  if (OregonMessageSize == 8) {
    sendQuarterLSB(0x00);
  } else {
    sendQuarterLSB(0x00);
    sendQuarterMSB(0x00);
  }
}



/*

   Exemple de trames:
   OSV2 EA4C106F7011D0D30300
   OSV3 FA28A428202290834B46
   OSV2 EA4C106F7011D0D30300
   OSV2 EA4C106F7011D0D30300
   OSV3 FA28A428202290834B46
   OSV2 EA4C106F7011D0D30300
   OSV2 EA4C106F7011D0D30300
   OSV3 FA28A428202290834B46
   OSV3 FA28A428202290834B46
   OSV2 EA4C106F6011C0A30600
   OSV2 EA4C106F6011C0A30600
   OSV2 EA4C106F6011C0A30600
   OSV3 FA28A428202290834B46
   OSV2 EA4C106F6011C0A30600
   OSV3 FA28A428202290834B46
   OSV2 EA4C106F6011C0A30600
   OSV2 EA4C106F6011C0A30600

   Bit 0 is sent first, then bit 1, 2, 3, 4, 5, 6, and then bit 7 of a byte

   The sample packet above is from a THGR122NX on channel 1 with rolling code  EC . It's returning a temperature of +27.3°C, and humidity 65%:
   1A 2D 10 EC 32 27 50 06 44 25

  - 0-3: Device ID. The ID for THGN132N sensors is  1A2D .
  - 4: Channel. This corresponds to the channel slider on the back of the sensor ( 1, 2, or 4 for channel 1, 2 or 3).
  - 5: Battery? All of my readings have 0 for this nibble. I'm half-expecting it to become non-zero on low battery.
  - 6-7: Rolling code. This is a unique identifier for the sensor. It resets when the battery is replaced.
  - 8: The tenths digit of the temperature.
  - 10: The tens digit of the temperature.
  - 11: The unit digit of the temperature.
  - 12: The unit digit of the humidity.
  - 13: The sign for the temperature. This nibble will be 0 for a +ve temp, and non-zero for -ve. During my testing with the sensor in the freezer, I've only seen this return 0 or 8.
  - 15: The tens digit of the humidity.

  --------------------------------------------------------
  |  Sensor Name      |  Code   | Type                   |
  --------------------------------------------------------
  |  Oregon-THR128    |         |                        |
  |  Oregon-THR138    | 0x0A4D  | Inside Temperature     |
  |  Oregon-THC138    |         |                        |
  --------------------------------------------------------
  | Oregon-THC238     |         |                        |
  | Oregon-THC268     |         |                        |
  | Oregon-THN132N    |         |                        |
  | Oregon-THWR288A   | 0xEA4C  | Outside/Water Temp     |
  | Oregon-THRN122N   |         |                        |
  | Oregon-THN122N    |         |                        |
  | Oregon-AW129      |         |                        |
  | Oregon-AW131      |         |                        |
  --------------------------------------------------------
  | Oregon-THWR800    | 0xCA48  | Water Temp             |
  --------------------------------------------------------
  | Oregon-THGN122N   |         |                        |
  | Oregon-THGN123N   |         |                        |
  | Oregon-THGR122NX  | 0x1A2D  | Inside Temp-Hygro      |
  | Oregon-THGR228N   |         |                        |
  | Oregon-THGR238    |         |                        |
  | Oregon-THGR268    |         |                        |
  --------------------------------------------------------
  | Oregon-THGR810    | 0xFA28  | Inside Temp-Hygro      |
  | Oregon-RTGR328N   | 0x*ACC  | Outside Temp-Hygro     |
  | Oregon-THGR328N   | 0xCA2C  | Outside Temp-Hygro     |
  | Oregon-WTGR800    | 0xFAB8  | Outside Temp-Hygro     |
  --------------------------------------------------------
  | Oregon-THGR918    |         |                        |
  | Oregon-THGRN228NX | 0x1A3D  | Outside Temp-Hygro     |
  | Oregon-THGN500    |         |                        |
  --------------------------------------------------------
  | Huger - BTHR918   | 0x5A5D  | Inside Temp-Hygro-Baro |
  --------------------------------------------------------
  | Oregon-BTHR918N   |         |                        |
  | Oregon-BTHR968    | 0x5A6D  | Inside Temp-Hygro-Baro |
  --------------------------------------------------------
  | Oregon-RGR126     |         |                        |
  | Oregon-RGR682     | 0x2A1D  | Rain Gauge             |
  | Oregon-RGR918     |         |                        |
  --------------------------------------------------------
  | Oregon-PCR800     | 0x2A19  | Rain Gauge             |
  | Oregon-WTGR800    | 0x1A99  | Anemometer             |
  | Oregon-WGR800     | 0x1A89  | Anemometer             |
  --------------------------------------------------------
  | Huger-STR918      |         |                        |
  | Oregon-WGR918     | 0x3A0D  | Anemometer             |
  --------------------------------------------------------
  | Oregon-UVN128     |         |                        |
  | Oregon-UV138      | 0xEA7C  | UV sensor              |
  --------------------------------------------------------
  | Oregon-UVN800     | 0xDA78  | UV sensor              |
  | Oregon-RTGR328N   | 0x*AEC  | Date & RF_BIT_TIME            |
  --------------------------------------------------------
  | cent-a-meter      |         |                        |
  | OWL CM113         | 0xEAC0  | Ampere meter           |
  | Electrisave       |         |                        |
  --------------------------------------------------------
  | OWL CM119         | 0x1A*                            |
  |                   | 0x2A*   | Power meter            |
  |                   | 0x3A**  |                        |
  --------------------------------------------------------

*/
