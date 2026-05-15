import serial
import time

class DistanceDataParser:
    #默认构造函数 default constructor 
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):  
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_com = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
            )
    
    def parse_data_format(self, data_bytes):
        try:
            # 查找逗号分隔符的位置 (0x2C) Find the position of the comma separator (0x2C)
            comma_index = -1
            for i, b in enumerate(data_bytes):
                if b == 0x2C:
                    comma_index = i
                    break
            
            if comma_index == -1:
                return None, None
            
            # 提取距离部分 (逗号前的数据)  Extract the distance part (the data before the comma)
            # 跳过开头的空格(0x20)  Skip the initial space (0x20)
            start_index = 0
            while start_index < comma_index and data_bytes[start_index] == 0x20:
                start_index += 1
            # 距离数据长度 (2-5字节) Distance data length (2-5 bytes) :
            distance_bytes = data_bytes[start_index:comma_index]  
            
            # 提取置信度部分 (逗号后的数据) Extract the confidence part (the data after the comma)
            # 跳过逗号后的空格(0x20) The space after the comma (0x20)
            confidence_start = comma_index + 1
            while confidence_start < len(data_bytes) and data_bytes[confidence_start] == 0x20:
                confidence_start += 1
            
            # 置信度数据应为1-2字节:原始数据 The confidence data should be 1-2 bytes: raw data
            confidence_bytes = data_bytes[confidence_start:len(data_bytes)-1]
            
            # 转换距离数据 Convert distance data
            distance_str = distance_bytes.decode('ascii')
            distance = int(distance_str)
            if distance >4000:
            	return None,None
            	
            # 转换置信度数据 Convert the confidence level data
            confidence_str = confidence_bytes.decode('ascii')
            confidence = int(confidence_str)
            
            return distance, confidence
        except Exception as e:
            print(f"parse_error: {e}")
            return None, None
    
    def start_processing(self):
        if not self.serial_com:
            return
        try:
            while True:
                if self.serial_com.in_waiting > 0:
                    # 读取一行数据 Read a line of data
                    data = self.serial_com.readline()
                    if data:
                        # 解析数据 Read a line of data
                        distance, confidence = self.parse_data_format(data)
                        # 打印结果 Read a line of data
                        if distance is not None and confidence is not None:
                            print(f"Distance: {distance} mm, Confidence: {confidence}\r")
                
        except KeyboardInterrupt:
            print("\nExit!")
        finally:
            if self.serial_com:
                self.serial_com.close()


if __name__ == "__main__":
    # 根据设备实际串口号进行替换  Replace according to the actual serial port number of the device
    SERIAL_PORT = "/dev/ttyAMA0" 
    BAUD_RATE = 115200
    parser = DistanceDataParser(serial_port=SERIAL_PORT, baud_rate=BAUD_RATE)
    parser.start_processing()
