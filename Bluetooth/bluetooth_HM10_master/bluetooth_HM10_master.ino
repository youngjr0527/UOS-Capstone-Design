#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <ArduinoHardware.h>
SoftwareSerial bluetooth(2, 3); // RX, TX
ros::NodeHandle nh;
std_msgs::String rssi_msg;
ros::Publisher rssi_pub("rssi", &rssi_msg);

int flag;


void sendATCommand(String command) 
{
  bluetooth.print(command);
  delay(500);
  while (bluetooth.available()) {
  char c = bluetooth.read();
  Serial.print(c);
  

}
  Serial.print("\r\n");
  
// }
}

String readATResponse(String command) {
  String response = "";
  bluetooth.print(command);
  // 잘 안되면 500으로 변경
  delay(300);
  while (bluetooth.available()) {
    char c = bluetooth.read();
    response += c;
    Serial.print(c);
    // return "1" + response ;
  }
  char c = bluetooth.read();
  // response = c + "s";
  return response;
}

void setup() {
  // nh.getHardware()->setBaud(9600);
  flag = 0;
  nh.initNode();
  nh.advertise(rssi_pub);
  Serial.begin(9600);
  bluetooth.begin(9600);
  delay(1000);
  // ArduinoHardware arduinoHardware;
  // arduinoHardware.setBaud(9600);

  //////////////////////////////
  sendATCommand("AT+RENEW");
  delay(2000);
  //////////////////////////////


  sendATCommand("AT");
  delay(1000);
  sendATCommand("AT+ADDR?");
  delay(1000);
  sendATCommand("AT+BAUD?");
  delay(1000);
  sendATCommand("AT+ROLE1");
  delay(1000);
  sendATCommand("AT+IMME1");
  delay(3000);
  // sendATCommand("AT+CON10CEA9FCDF5B");
  

  // sendATCommand("AT+3CA308B57B47");

  sendATCommand("AT+CON4C249818564B");

  
  delay(1000);

  String rssiData = "start!!!";
  
  // Publish the RSSI data as a string to the "rssi" topic
  rssi_msg.data = rssiData.c_str();
  rssi_pub.publish(&rssi_msg);
 
}

void loop() {
  // sendATCommand("AT+RSSI?"); // cost 40ms
 

  // total 50ms

  // Read RSSI data from the Bluetooth module
  String rssiData = readATResponse("AT+RSSI?");
  delay(100);                 // cost 10ms
  // Publish the RSSI data as a string to the "rssi" topic


  
  if (!(rssiData.length() > 0) && flag < 2) 
  {
    String response_e = "";
    // String blank = "blank";
    char c_e = "b";
    response_e += c_e;
    rssi_msg.data = response_e.c_str();
    rssi_pub.publish(&rssi_msg);
    Serial.println(response_e);
    flag = flag + 1;
    // nh.spinOnce();

    // nh.spinOnce();
    // rssi_pub.publish(&rssi_msg);
    // delay(1000); 

  }


  else if (rssiData.length() > 0) {
  rssi_msg.data = rssiData.c_str();
  rssi_pub.publish(&rssi_msg);
  Serial.println(rssiData);

  // nh.spinOnce();

  flag = 0;

  }
  

  else {
    // If the received data is empty, send the connection command again
    // String myString = "blank!"; // 출력하고 싶은 문자열

    // 문자열을 시리얼 모니터에 출력
    // Serial.println(myString);
    // String response_e = "";
    // // String blank = "blank";
    // char c_e = "b";
    // response_e += c_e;
    // rssi_msg.data = response_e.c_str();
    // rssi_pub.publish(&rssi_msg);
    // Serial.println(response_e);
    // delay(100); 
    // nh.spinOnce();
    // rssi_pub.publish(&rssi_msg);
    // nh.initNode();
    // nh.advertise(rssi_pub);
    Serial.begin(9600);
    bluetooth.begin(9600);
    delay(1000);
    // ArduinoHardware arduinoHardware;
    // arduinoHardware.setBaud(9600);

    //////////////////////////////
    sendATCommand("AT+RENEW");
    delay(2000);
    //////////////////////////////


    sendATCommand("AT");
    delay(500);
    sendATCommand("AT+ADDR?");
    delay(500);
    sendATCommand("AT+BAUD?");
    delay(1000);
    sendATCommand("AT+ROLE1");
    delay(1000);
    sendATCommand("AT+IMME1");
    delay(3000);
    // sendATCommand("AT+CON10CEA9FCDF5B");
    

    // sendATCommand("AT+3CA308B57B47");

    sendATCommand("AT+CON4C249818564B");

    
    delay(1000);

  }

  nh.spinOnce();

}

////////////////////////////////////////////////////


// #include <SoftwareSerial.h>
// SoftwareSerial bluetooth(2, 3); // RX, TX


// void setup() {
//   Serial.begin(9600);
//   bluetooth.begin(9600);
//   delay(1000);


//   //////////////////////////////
//   sendATCommand("AT+RENEW");
//   delay(2000);
//   //////////////////////////////



//   sendATCommand("AT");
//   delay(1000);
//   sendATCommand("AT+ADDR?");
//   delay(1000);
//   sendATCommand("AT+BAUD?");
//   delay(1000);
//   sendATCommand("AT+ROLE1");
//   delay(1000);
//   sendATCommand("AT+IMME1");
//   delay(3000);


//   sendATCommand("AT+CON4C249818564B");

  
//   delay(1000);
 
// }



// void loop() {
//   sendATCommand("AT+RSSI?");
//   delay(10);
  
// }

// void sendATCommand(String command) {
//   bluetooth.print(command);
//   delay(500);
  
//   while (bluetooth.available()) {
//     char c = bluetooth.read();
//     Serial.print(c);
    

//   }
//   Serial.print("\r\n");
  
// }


/////////////////////////////////////////////
// #include <SoftwareSerial.h>        // 블루투스 시리얼 통신 라이브러리 추가
// #define BT_RXD 2
// #define BT_TXD 3
// SoftwareSerial bluetooth(BT_RXD, BT_TXD);        // 블루투스 설정 BTSerial(Tx, Rx)
// void setup() {
//   Serial.begin(9600);
//   bluetooth.begin(9600);                         // 블루투스 통신 시작
// }
// void loop() {
//   if (bluetooth.available()) {        // 블루투스에서 보낸 내용은 시리얼모니터로 전송
//     Serial.write(bluetooth.read());
//   }
//   if (Serial.available()) {           // 시리얼모니터에서 보낸 내용은 블루투스로 전송
//     bluetooth.write(Serial.read());
//   }
// }


////////////////////////////////////////////////

// #include <SoftwareSerial.h>
// SoftwareSerial bluetooth(2, 3); // RX, TX


// void setup() {
//   Serial.begin(9600);
//   bluetooth.begin(9600);
//   delay(1000);
//   // sendATCommand("AT+BAUD3");
//   // delay(1000);
//   // sendATCommand("AT+ADDR?");
//   // delay(1000);
  
//   // sendATCommand("AT+RESET");
//   // delay(1000);

//   //////////////////////////////
//   sendATCommand("AT+RENEW");
//   delay(2000);
//   //////////////////////////////

//   // ////////////////////////////////////
//   // sendATCommand("AT+BAUD3");
//   // delay(1000);

  
//   // sendATCommand("AT+RESET");
  
//   // delay(1000);

//   // Serial.begin(57600);
//   // bluetooth.begin(57600);
//   // ////////////////////////////////////////

//   sendATCommand("AT");
//   delay(1000);
//   sendATCommand("AT+ADDR?");
//   delay(1000);
//   sendATCommand("AT+BAUD?");
//   delay(1000);
//   sendATCommand("AT+ROLE1");
//   delay(1000);
//   sendATCommand("AT+IMME1");
//   delay(3000);
//   // sendATCommand("AT+CON10CEA9FCDF5B");
  

//   // sendATCommand("AT+3CA308B57B47");

//   sendATCommand("AT+CON4C249818564B");

  
//   delay(1000);
 
// }

// void loop() {
//   if (bluetooth.available()) {        // 블루투스에서 보낸 내용은 시리얼모니터로 전송
//     Serial.write(bluetooth.read());
//   }
//   if (Serial.available()) {           // 시리얼모니터에서 보낸 내용은 블루투스로 전송
//     bluetooth.write(Serial.read());
//   }
// }

// void sendATCommand(String command) {
//   bluetooth.print(command);
//   delay(500);
  
//   while (bluetooth.available()) {
//     char c = bluetooth.read();
//     Serial.print(c);
    

//   }
//   Serial.print("\r\n");
  
// }