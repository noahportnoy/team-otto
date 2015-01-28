Enter file contents here/*
 * uBlox UBX Protocol Reader (runs on Arduino Leonardo, or equivalent)
 *
 * Note: RX pad on 3DR Module is output, TX is input
 */
//#include <PString.h>
#include <Wire.h>
#include <stdio.h>
 
#define MAX_LENGTH 512

#define  POSLLH_MSG  0x02

#define LONG(X)    *(long*)(&data[X])
#define ULONG(X)   *(unsigned long*)(&data[X])
#define INT(X)     *(int*)(&data[X])
#define UINT(X)    *(unsigned int*)(&data[X])

long gps_long = 0;
long gps_lat = 0;

unsigned char  state, lstate, code, id, chk1, chk2, ck1, ck2;
unsigned int  length, idx, cnt;

unsigned char data[MAX_LENGTH];

long lastTime = 0;

void enableMsg (unsigned char id, boolean enable) {
  //               MSG   NAV   < length >  NAV
  byte cmdBuf[] = {0x06, 0x01, 0x03, 0x00, 0x01, id, enable ? 1 : 0};
  sendCmd(sizeof(cmdBuf), cmdBuf);
}
char receivedData;

void setup() {
   
  Serial.begin(9600);
  Serial1.begin(38400);
  
  Wire.begin(0x04);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // Modify these to control which messages are sent from module
  enableMsg(POSLLH_MSG, true);    // Enable position messages
}

void receiveData(int byteCount)
{
 //Serial.println("receiveData callback");

  while(Wire.available())
  {
    receivedData = Wire.read();
  }
     
   // Serial.print("Command received: 0x");
   // Serial.println(receivedData, HEX);
}

void sendData()
{
 // Serial.println("sendData callback");
  
  byte dataToSend[4];
  
  if(receivedData == 0)
  {
    memcpy(dataToSend, &gps_lat, 4);

  }else if(receivedData == 1)
  {
     memcpy(dataToSend, &gps_long, 4);
  }
  
  Wire.write(dataToSend, 4);
}

int counter = 0;
void loop() {
  if (Serial1.available()) {
    unsigned char cc = Serial1.read();
    switch (state) {
      case 0:    // wait for sync 1 (0xB5)
        ck1 = ck2 = 0;
        if (cc == 0xB5)
          state++;
        break;
      case 1:    // wait for sync 2 (0x62)
        if (cc == 0x62)
          state++;
        else
          state = 0;
        break;
      case 2:    // wait for class code
        code = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 3:    // wait for Id
        id = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 4:    // wait for length byte 1
        length = cc;
        ck1 += cc;
        ck2 += ck1;
        state++;
        break;
      case 5:    // wait for length byte 2
        length |= (unsigned int) cc << 8;
        ck1 += cc;
        ck2 += ck1;
        idx = 0;
        state++;
        if (length > MAX_LENGTH)
          state= 0;
        break;
      case 6:    // wait for <length> payload bytes
        data[idx++] = cc;
        ck1 += cc;
        ck2 += ck1;
        if (idx >= length) {
          state++;
        }
        break;
      case 7:    // wait for checksum 1
        chk1 = cc;
        state++;
        break;
      case 8:    // wait for checksum 2
        chk2 = cc;
        boolean checkOk = ck1 == chk1  &&  ck2 == chk2;
        if (checkOk) {
         
          switch (code) {
             // Serial.print("NAV-");
              case 0x01: 
              switch (id) 
              {
                case 0x02:  // NAV-POSLLH
                  gps_long = LONG(4);
                  gps_lat = LONG(8);
                
                  //Serial.print("Updated Long: ");
                 //Serial.println(counter);r
                 /* Serial.print(gps_long);
                  Serial.println(" ");
                  Serial.print("Updated Lat: ");
                  Serial.print(gps_lat);
                  Serial.println(" \n");*/
                 break;

                 
                default:
                  //printHex(id);
                  break;
              }
             // Serial.println();
              break;
            case 0x05:      // ACK-
              //Serial.print(F("ACK-"));
              switch (id) {
                case 0x00:  // ACK-NAK
                //Serial.print(F("NAK: "));
                break;
                case 0x01:  // ACK-ACK
                //Serial.print(F("ACK: "));
                break;
              }

              break;
          }
        }
        state = 0;
        break;
    }
  }
}


void sendCmd (unsigned char len, byte data[]) {
  Serial1.write(0xB5);
  Serial1.write(0x62);
  unsigned char chk1 = 0, chk2 = 0;
  for (unsigned char ii = 0; ii < len; ii++) {
    unsigned char cc = data[ii];
    Serial1.write(cc);
    chk1 += cc;
    chk2 += chk1;
  }
  Serial1.write(chk1);
  Serial1.write(chk2);
}

