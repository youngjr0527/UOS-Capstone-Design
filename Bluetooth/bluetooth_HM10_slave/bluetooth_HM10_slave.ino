#include <SoftwareSerial.h>

SoftwareSerial bluetooth(2,3); // RX, TX

void setup() {
  //기본 통신속도는 9600입니다.
  Serial.begin(9600);
  bluetooth.begin(9600);
  delay(1000); // 명령 실행에 충분한 시간을 줍니다.
  sendATCommand("AT+RENEW");
  delay(1000);
  // sendATCommand("AT+RESET");
  
  // delay(1000);
  sendATCommand("AT");
  delay(1000);
  sendATCommand("AT+ADDR?");
  delay(1000);
  sendATCommand("AT+BAUD?");
  delay(1000);
  
  // ////////////////////////////////////
  // sendATCommand("AT+BAUD3");
  // delay(1000);

  
  // sendATCommand("AT+RESET");
  
  // delay(1000);

  // Serial.begin(57600);
  // bluetooth.begin(57600);
  // ////////////////////////////////////////
  sendATCommand("AT+MODE2");
  delay(1000);
  sendATCommand("AT+ROLE0");
  delay(7000);
  // sendATCommand("AT+IMME1");
  // delay(1000);
  

}

void loop() {
  if (bluetooth.available()) {
    Serial.write(bluetooth.read());
  }
  if (Serial.available()) {
    bluetooth.write(Serial.read());
  }

  
}

void sendATCommand(String command) 
{
  bluetooth.print(command);
  delay(500); // 명령 실행에 충분한 시간을 줍니다.
  //Serial.print("hi1");
  while (bluetooth.available()) {
    char c = bluetooth.read();
    //Serial.print("hi2");
    Serial.print(c);
    
  }
  Serial.print("\r\n");
}