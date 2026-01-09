#include <Arduino.h>

#include "Settings.h"
#include "DataBank.h"
#include "LoRa.h"

UART Lora(LoRa_RX, LoRa_TX);

struct LoRa_packet{
  bool MagnetOn;
  short int OperationMode;

  float lat;
  double lng;

  float temperature;
  float pressure;
  float altitude;

  float gx;
  float gy;
  float gz;

  float ax;
  float az;
  float ay;

  float course;
  float speed;
  float predicted_distance;
}

LoRa_packet Packet;

void LoRa_send_command(String cmd) {
  // Send the command (if cmd is empty, it just sends \r\n)
  LoRa.print(cmd + "\r\n");
  
  // Brief window to capture the immediate response
  unsigned long timeout = millis() + 500; 
  while (millis() < timeout) {
    if (LoRa.available()) {
      String response = LoRa.readStringUntil('\n');
      if (response.indexOf("ok") != -1 || response.indexOf("radio_tx_ok") != -1) break;
    }
  }
}

String convertToHex(void* ptr, size_t size) {
  uint8_t* bytes = (uint8_t*)ptr; // Treat the data as an array of bytes
  String hexOutput = "";

  for (size_t i = 0; i < size; i++) {
    if (bytes[i] < 0x10) hexOutput += "0"; // Add leading zero for 0-F
    hexOutput += String(bytes[i], HEX);
  }
  
  hexOutput.toUpperCase();
  return hexOutput;
}

void Fill_Packet(){

  Packet.MagnetOn = MainBank.vmi;
  Packet.lat = MainBank.GPS.latitude;
  Packet.lng = MainBank.GPS.longitude;
  Packet.course = MainBank.GPS.course;
  Packet.speed = ManiBank.GPS.speed;

  Packet.altitude = MainBank.BMP.altitude;
  Packet.pressure =  MainBank.BMP.pressure;
  Packet.temperature = MainBank.BMP.temperature;

  Packet.predicted_distance = MainBank.GPS.predicted_current_distance;
  Packet.Operation_Mode = MainBank.vmi;

  Packet.gx = MainBank.IMU.gx;
  Packet.gy = MainBank.IMU.gy;
  Packet.gz = MainBank.IMU.gz;

  Packet.ax = MainBank.IMU.ax;
  Packet.ay = MainBank.IMU.ay;
  Packet.az = MainBank.IMU.az;
}

void LoRa_init(){
  LoRa.begin(57600);

  LoRa_send_command("");              // Garbage command
  LoRa_send_command("sys reset");     // Hardware reboot
  delay(500);                // Wait for reboot
  LoRa_send_command("mac pause");     // Allow P2P mode
  LoRa_send_command("radio set sf sf7");
  LoRa_send_command("radio set mode lora");
  LoRa_send_command("radio set freq 868100000");
  LoRa_send_command("radio set pwr 15");
  LoRa_send_command("radio set bw 125");
  LoRa_send_command("radio set cr 4/6");
  LoRa_send_command("radio set crc on");
  LoRa_send_command("radio set wdt 0");
}

void LoRa_run(){

  // Filling the packet from our databank
  Fill_Packet();

  String Full_Packet = convertToHex(&Packet, sizeof(Packet));
  
  LoRa_send_command("radio tx" + Full_Packet);
  // Kéne még acceleration is!
}