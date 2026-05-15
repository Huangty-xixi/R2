#include "bsp_uart.hpp"
#include <stdio.h>
#include <SoftwareSerial.h>

void parseDistanceData(String &data) {
  // 去除首尾空白字符  Remove the first and last whitespace characters
  data.trim();
  
  // 查找逗号位置 Find the position of the comma
  int commaPos = data.indexOf(',');
  if (commaPos == -1) {
    return;
  }
  
  // 提取距离  Extraction distance
  String distanceStr = data.substring(0, commaPos);
  distanceStr.trim();
  
  // 提取置信度  Extract the confidence level section
  String confidenceStr = data.substring(commaPos + 1);
  confidenceStr.trim();
  
  // 转换为整数 Convert to an integer 
  long distance = distanceStr.toInt();  
  long confidence = confidenceStr.toInt();
  
  if ((distance <=20 || distance>=4000) && distanceStr != "0") {
    return;
  }
  if (confidence == 0 && confidenceStr != "0") {
    return;
  }
  
  // 输出结果 output result
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.print(" mm, Confidence: ");
    Serial.println(confidence);
}



