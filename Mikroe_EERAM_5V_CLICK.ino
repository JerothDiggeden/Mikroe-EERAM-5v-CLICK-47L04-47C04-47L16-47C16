#include <Wire.h>

const uint8_t deviceAddr = 0x50; // Adjust based on your SRAM chip configuration
uint16_t writeAddr = 0x0000;      // Starting address to write/read
uint16_t valueToWrite = 0xAA55;    // Initial value to write to SRAM (16-bit word)

void setup() {
  Serial.begin(9600); // Initialize serial communication
  Wire.begin();       // Initialize I2C (Wire) library
  pinMode(9, OUTPUT); // Set pin 9 as PWM output

  // Write 0x0000 to reset the memory before the main loop
  writeSRAM(deviceAddr, writeAddr, 0x0000);
  Serial.println("Setup complete.");
}

void loop() {
  // Write data to SRAM
  writeSRAM(deviceAddr, writeAddr, valueToWrite);
  Serial.print("Data written at address 0x");
  Serial.print(writeAddr, HEX);
  Serial.print(": 0x");
  Serial.println(valueToWrite, HEX);

  // Read data from SRAM immediately after writing
  uint16_t data = readCurrentAddressSRAM(deviceAddr);
  Serial.print("Data read after write: 0x");
  Serial.println(data, HEX); // Print the read value

  // Adjust the PWM signal based on the read value (lower 8 bits)
  analogWrite(9, data & 0xFF); // Write the lower byte as PWM to pin 9

  // Evolve the valueToWrite and writeAddr
  valueToWrite += 1; // Increment the value by 1
  if (valueToWrite > 0xFFFF) { // Reset if it exceeds 16 bits
    valueToWrite = 0x0000; 
  }

  writeAddr += 2; // Increment the address by 2 (to write the next word)
  if (writeAddr > 0xFFFF) { // Reset if it exceeds 16 bits
    writeAddr = 0x0000; 
  }

  // Wait for 2 seconds before the next iteration
  delay(2000);
}

void writeSRAM(uint8_t deviceAddr, uint16_t memAddr, uint16_t data) {
  Wire.beginTransmission(deviceAddr); // Begin transmission to the device
  Wire.write((memAddr >> 8) & 0xFF);  // Send high byte of address
  Wire.write(memAddr & 0xFF);          // Send low byte of address
  Wire.write((data >> 8) & 0xFF);      // Send high byte of data
  Wire.write(data & 0xFF);             // Send low byte of data
  Wire.endTransmission();              // End transmission
  
  delay(20); // Additional delay to ensure data is latched properly
}

uint16_t readCurrentAddressSRAM(uint8_t deviceAddr) {
  Wire.beginTransmission(deviceAddr);  // Begin transmission to the device
  Wire.write((writeAddr >> 8) & 0xFF); // Send high byte of address
  Wire.write(writeAddr & 0xFF);         // Send low byte of address
  Wire.endTransmission();               // End transmission

  Wire.requestFrom(deviceAddr, 2);     // Request two bytes from the device
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();    // Read the high byte
    uint16_t lowByte = Wire.read();     // Read the low byte
    return (highByte << 8) | lowByte;   // Combine into a 16-bit word
  }
  return 0;                             // Return 0 if no bytes were read
}