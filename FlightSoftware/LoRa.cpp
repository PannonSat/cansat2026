

#include <Arduino.h>
#include "Settings.h"
#include "DataBank.h"
#include "LoRa.h"
#include "LED.h"
#include <SoftwareSerial.h>

struct __attribute__((packed)) LoRa_packet{
  bool Magnet_on;
  short OperationMode;

  double lat;
  double lng;

  float temperature;
  float pressure;
  float altitude;

  float tilt_x;
  float tilt_y;
  float course;

  float V_vert;
  float time;

  float spinrate;
  float speed;
  float predicted_distance;
};

bool init_esp = true;
LoRa_packet Packet;
unsigned long last_tx_time = 0;
int packet_count = 0;
String espBuffer = "";
String ESP_PACKET;
String Last_ESP_Packet;

void Fill_Packet(){
  // Call az sok?
  Packet.lat = MainBank.GPS.latitude;
  Packet.lng = MainBank.GPS.longitude;
  Packet.tilt_x = MainBank.IMU.tilt_x;
  Packet.tilt_y = MainBank.IMU.tilt_y;
  Packet.course = MainBank.GPS.course;
  Packet.speed = MainBank.GPS.speed;

  Packet.altitude = MainBank.BMP.altitude;
  Packet.pressure =  MainBank.BMP.pressure;
  Packet.temperature = MainBank.BMP.temperature;

  Packet.predicted_distance = MainBank.GPS.predicted_current_distance;

  Packet.V_vert = MainBank.IMU.V_vertical;
  Packet.spinrate = MainBank.IMU.spinrate;
  Packet.time = millis();
  Packet.OperationMode = MainBank.Operation_Mode;
  Packet.Magnet_on = MainBank.Magnet_on;
  Packet.lat = MainBank.GPS.latitude;
  Packet.lng = MainBank.GPS.longitude;
  Packet.tilt_x = MainBank.IMU.tilt_x;
  Packet.tilt_y = MainBank.IMU.tilt_y;
  Packet.course = MainBank.GPS.course;
  Packet.speed = MainBank.GPS.speed;

  Packet.altitude = MainBank.BMP.altitude;
  Packet.pressure =  MainBank.BMP.pressure;
  Packet.temperature = MainBank.BMP.temperature;

  Packet.predicted_distance = MainBank.GPS.predicted_current_distance;

  Packet.V_vert = MainBank.IMU.V_vertical;
  Packet.spinrate = MainBank.IMU.spinrate;
  Packet.time = millis();
  Packet.OperationMode = MainBank.Operation_Mode;
  Packet.Magnet_on = MainBank.Magnet_on;
}

String convertToHex(void* ptr, size_t size) {
  // x ms
  uint8_t* bytes = (uint8_t*)ptr;
  char buf[size * 2 + 1];
  
  for (size_t i = 0; i < size; i++) {
    sprintf(&buf[i * 2], "%02X", bytes[i]);
  }
  
  return String(buf);
}


void LoRa_send_command(String cmd) {
  // Send the command 
  Serial1.print(cmd + "\r\n");
  unsigned long timeout = millis() + 50; 
  
      while(millis() < timeout) {
        if (Serial1.available()) {
          String response = "";
          while (Serial1.available()) {
            response += (char)Serial1.read();
          }
          response.trim();
        }
      }

}
void read_ESP() {
  while (ESP.available() > 0) ESP.read();
  if(Status.picture ==1){}else return;
  ESP.print('R');
  // 3. SEEK START MARKER 's'
  unsigned long timeout = millis();
  bool foundStart = false;
  
  while (millis() - timeout < 500) {
    if (ESP.available() > 0 && ESP.read() == 's') {
      foundStart = true;
      break;
    }
  }
  
  if (!foundStart) {
    LOGln("Error: Connection Timeout (No 's')");
    return;
  }

  // 4. BLOCKING READ (256 BYTES)
  // readBytes handles the flow control from the 64-byte hardware buffer
  char packet[257]; 
  int bytesRead = ESP.readBytes(packet, 256);

  if (bytesRead == 256) {
    // 5. VERIFY END MARKER 'e'
    unsigned long eWait = millis();
    while(ESP.available() == 0 && millis() - eWait < 50); 
    
    if (ESP.peek() == 'e') {
      ESP.read(); // Consume 'e'
      packet[256] = '\0'; 
      
      // Update global packet variable
      ESP_PACKET = String(packet); 
      Status.picture = false;


      // 6. MANDATORY COOL-DOWN (POST-READ)
      // Drains any remaining bits and creates a 50ms silence gap
      while(ESP.available() > 0) ESP.read();   
    } else {
      LOGln("Error: Frame Mismatch (No 'e')");
    }
  } else {
    LOGln("Error: Incomplete Packet");
  }
}

void LoRa_init(){
  Serial1.setFIFOSize(256);
  Serial1.begin(115200); // could try 460,800 or higher
  ESP.begin(115200);
  LOG("LoRa init success!");
  LED_beep(100, 1);
  // 1. WAKE UP / AUTO-BAUD
  // Send a dummy character and wait

  // 3. THE RESET (Crucial)
  // Instead of LoRa_send_command, do it manually to see where it hangs
  Serial1.print("sys reset");

  LOG("After sys reset!");
  LOGln("MARK");
  delay(1000); // The module takes a full second to reboot
  LoRa_send_command("radio set sf sf7");
  LoRa_send_command("radio set mod lora");
  LoRa_send_command("radio set freq 869100000");
  LoRa_send_command("radio set pwr 20");
  LoRa_send_command("radio set pa on");
  LoRa_send_command("radio set bw 250");
  LoRa_send_command("radio set cr 4/5");
  LoRa_send_command("radio set crc on");
  LoRa_send_command("radio set wdt 0");
  LoRa_send_command("radio set prlen 8");
  
  espBuffer = "";
  // Turn ESP on
  //removed, see main.
}

void send_ESP(){
  if(ESP_PACKET.length() == 256 && Last_ESP_Packet != ESP_PACKET){
    // Send every chunk twice just in case
    Serial1.print("radio tx "+ ESP_PACKET + " 1\r\n");
    //Serial.println("SENT ESP");
    Status.picture = true;
    Last_ESP_Packet = ESP_PACKET;
  }
}
unsigned long last_lora_time = 0;
int false_count = 0;

void LoRa_run(){
  // Every fifteenth packet sends an image part
  unsigned long current_time = millis();
  
  // Only execute every 74ms using non-blocking timing
  if (current_time - last_lora_time < 74) {
    return;  // Exit early, don't block
  }
  last_lora_time = current_time;
  

  if (packet_count < 15){
    Fill_Packet();
    String Full_Packet = convertToHex(&Packet, sizeof(Packet));
    //Serial.println(Full_Packet);
    if(Full_Packet.length() == 126){
      Serial1.print("radio tx " + Full_Packet + " 1\r\n");
    }else{
      LOGln("INVALID TELEMETRY " + String(Full_Packet.length()));
    }
  }else{
    send_ESP();
    packet_count = 0;
  }

  last_tx_time = millis();
  packet_count++;

  if(Status.picture == 0){
    false_count++;
  }else{
    false_count = 0;
  }

  if(false_count >= 35){
    Status.picture = true;
    false_count = 0;
  }
  //Serial.println("Added packet");
  //Serial.print("picture: ");
  //Serial.println(String(Status.picture));
}
