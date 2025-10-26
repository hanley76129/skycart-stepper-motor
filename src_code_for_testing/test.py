from Adafruit_PureIO.smbus import SMBus 
import time
# This is the address we gave in the ESP32 - see code
ESP_I2C_address = 0x04
# open I2C bus 1 = RPi3/4
bus = SMBus(1)
# 2 classes to easy the use of I2C data streams
# encode data to be sent
class I2C_Encoder:
 # because ESP Slave I2C library wait for buffer[128] size
  PACKER_BUFFER_LENGTH = (128)
  def __init__(self):
    self._buffer = [0] * self.PACKER_BUFFER_LENGTH
    self._frame_start = 0x02
    self._frame_end = 0x04
    self.reset()
  def reset(self):
    # Reset the packing process.
    self._buffer[0] = self._frame_start
    # field for total lenght on index 1. data starts on field 2 
    self._index = 2 
    self._is_written = False
  def write(self, data: int):
 # write data in prepared buffer
 # do not allow write after .end()
    if self._is_written:
      raise Exception("ERROR: You need to restart process by using .reset() method before writing to buffer")
    self._buffer[self._index] = data
    self._index += 1
  def end(self):
    # Closes the packet by adding crc8 and length 
    # After that, use read()
    # skip field for CRC byte
    self._index += 1 
    # add frame end
    self._buffer[self._index] = self._frame_end
    # calc and write total length
    self._index += 1
    self._total_length = self._index
    self._buffer[1] = self._total_length
    # ignore crc and end byte
    payload_range = self._total_length - 2
    # ignore start and length byte [2:payload_range]
    self._buffer[self._index - 2] = crc8(self._buffer[2:payload_range])
    self._is_written = True
    return self._buffer
  def read(self):
    # Read the packet
    if not self._is_written:
      raise Exception("ERROR: You need to finish process by using .end() method before read buffer")
    return self._buffer

# decodes data received from I2C data stream
class I2C_Decoder:
  error_codes = {
    "INVALID_CRC": 1,
    "INVALID_LENGTH": 2,
    "INVALID_START": 3,
    "INVALID_END": 4,
  }
  error_decodes = {
    1: "INVALID_CRC",
    2: "INVALID_LENGTH",
    3: "INVALID_START",
    4: "INVALID_END",
  }
  def __init__(self):
    self._debug = False
    self._frame_start = 0x02
    self._frame_end = 0x04
  
  def write(self, stream):
    # get the i2c data from slave
    # clear any previous
    self._buffer = []
    self._last_error = None
    data = list(stream)
    # check if start and end bytes are correct
    if data[0] != self._frame_start:
      self._last_error = self.error_codes["INVALID_START"]
      raise Exception("ERROR: invalid start byte")
    
    self._length = data[1]
    if data[self._length-1] != self._frame_end:
      self._last_error = self.error_codes["INVALID_END"]
      raise Exception("ERROR: invalid end byte")
    
    # check if provided crc8 is good
    # ignore start, length, crc and end byte
    crc = crc8(data[2:self._length-2])
    if crc != data[self._length-2]:
      self._last_error = self.error_codes["INVALID_CRC"]
      raise Exception("ERROR: Unpacker invalid crc8")
    self._data = data[2:self._length-2]
    # create string
    answer = ""
    for c in self._data:
      answer = answer + chr(c)
    return answer
 
  def get_last_error(self):
    """
    @brief get the last error code and message
    @return list [error_code, error_text]
    """
    return self._last_error, self.error_decodes[self._last_error]

# routine to calculate CRC8
def crc8(data: list):
  crc = 0
  for _byte in data:
    extract = _byte
    for j in range(8, 0, -1):
      _sum = (crc ^ extract) & 0x01
      crc >>= 1
      if _sum:
        crc ^= 0x8C
      extract >>= 1
  return crc

# read data from EPS32
def read_from_esp32(i2caddress: hex, size: int):
 
  decoder = I2C_Decoder()
 
  try:
    # get data sent by ESP32, in raw format.
    # read 25 bytes
    stream = bus.read_bytes(i2caddress, size)
    # convert to a list to ease handling
    data = decoder.write(stream) 
    return data
  except Exception as e:
    print("ERROR: {}".format(e))
 
# write data to ESP32
def write_to_esp32(i2caddress: hex, data: str):
  encoder = I2C_Encoder()
  try:
  # only if there is data 
    if len(data) > 0:
      for c in data:
        encoder.write(ord(c))
    stream = encoder.end()
 
    # send stream to slave 
    bus.write_bytes(i2caddress, bytearray(stream))
 
  except Exception as e:
    print("{}".format(e)) 

if __name__ == "__main__":
  while True:
    # request ESP32 to get data
    write_to_esp32(ESP_I2C_address, "")
    # wait to process the request
    time.sleep(0.1) 
    # Get the value from the ESP32
    data = read_from_esp32(ESP_I2C_address, 25)
    print(data)
    time.sleep(2)