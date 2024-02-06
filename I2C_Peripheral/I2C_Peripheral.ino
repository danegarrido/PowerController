#include <Wire.h>
#include <ACS712.h>

#define FREQUENCY_HZ    5
#define INTERVAL_MS     (1000 / (FREQUENCY_HZ + 1))

#define RELAY1          2
#define RELAY2          3

int x = 0;

static unsigned long last_interval_ms = 0;

ACS712  ACS(A0, 5.0, 1023, 66);

static union FloatI2C
{
	float m_float;
	uint8_t m_bytes[sizeof(float)];
} data;

void setup() {
  // Serial.begin(115200);
  uint16_t average = (ACS.getMinimum(20) + ACS.getMaximum(20)) / 2;
  ACS.setMidPoint(average);

  pinMode (RELAY1,OUTPUT);
  pinMode (RELAY2,OUTPUT);
  // Start the I2C Bus as Slave on address 0x9a
  Wire.begin(0x9a); 
  // Attach a function to trigger when something is received
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  digitalWrite(RELAY1,LOW);
  digitalWrite(RELAY2,LOW);
}

void receiveEvent(int bytes) {
  x = Wire.read();    // read one character from the I2C
  if (x == 0) {
    digitalWrite(RELAY1,HIGH);
    digitalWrite(RELAY2,HIGH);
  }
  else if (x == 1) {
    digitalWrite(RELAY1,LOW);
    digitalWrite(RELAY2,LOW);
  }
}

void requestEvent(int bytes) {
  Wire.write(data.m_bytes,4);
}

void loop() {
  if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();
        float mA = ACS.mA_AC(50,10);
        data.m_float = mA;
        //Serial.println(mA);
  }
}