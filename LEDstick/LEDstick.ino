LED LEDStick;

void setup() {
  // put your setup code here, to run once:
  bool begin(uint8_t address, TwoWire &wirePort);

  
  Wire.begin();
  Serial.begin(115200);

  if (LEDStick.begin() == false){
    Serial.println("Qwiic LED Stick failed to begin. Please check wiring and try again!);
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
