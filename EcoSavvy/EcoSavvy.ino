#include <SCServo.h>
#include <SoftwareSerial.h>

// Servo init
SCServo SERVO;
int SrvH = 335, SrvL = 60, SrvR = 620;

// Ultrasonic init
int pinRX = 10;
int pinTX = 11;
unsigned char data_buffer[4] = {0};
int distance = 0;
unsigned char CS;
SoftwareSerial mySerial(pinRX, pinTX);

// Mat sensor
int ifmPin = 4, ifmVal;

unsigned long previousTime = 0;
unsigned long delayInterval = 1000; // Delay interval of 1 second
 
void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  // Set up software serial port
  mySerial.begin(9600);

  // Servo initialization
  Serial1.begin(1000000);
  SERVO.pSerial = &Serial1;
  delay(500);
  SERVO.EnableTorque(0xfe,1);
  SERVO.WritePos(0xfe, SrvH, 1000);
  delay(1000);

  // IFM init
  pinMode(ifmPin,INPUT);

  previousTime = millis();
}
 
void loop() {
  // Run if data available
  if (mySerial.available() > 0) {
    delay(4);
    // Check for packet header character 0xff
    if (mySerial.read() == 0xff) {
      // Insert header into array
      data_buffer[0] = 0xff;
      // Read remaining 3 characters of data and insert into array
      for (int i = 1; i < 4; i++) {
        data_buffer[i] = mySerial.read();
      }
 
      //Compute checksum
      CS = data_buffer[0] + data_buffer[1] + data_buffer[2];

      if (data_buffer[3] == CS) {
        distance = (data_buffer[1] << 8) + data_buffer[2];
        // Print to serial monitor
        Serial.print("distance: ");
        Serial.print(distance);
        Serial.println(" mm");

        if (distance < 100) {
          unsigned long currentTime = millis();
          if (currentTime - previousTime >= delayInterval) {
            sort();
            previousTime = currentTime; // Reset the timer
            while (mySerial.available() > 0) {
              mySerial.read();
            }
          }
        }
      }
    }
  }
}

void sort() {
  ifmVal = digitalRead(ifmPin);
  Serial.print("IFM: ");
  Serial.println(ifmVal);
  if (ifmVal == 0) {
    SERVO.WritePos(0xfe, SrvL, 1000);
  } else {
    SERVO.WritePos(0xfe, SrvR, 1000);
  }
  delay(2000);
  SERVO.WritePos(0xfe, SrvH, 1000);
  delay(2000);
}
