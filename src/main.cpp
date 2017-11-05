#include "Arduino.h"
#include "Servo.h"

 int main(){
    init();

#if defined(USBCON)
    USB.attach();
#endif

    setup();

    while(1) {
        loop();
        if (serialEventRun) serialEventRun();
    }

    return 0;
}


uint16_t calculateDistance();
const uint8_t trigPin = 7;
const uint8_t echoPin = 8;
// Variables for the duration and the distance
uint32_t duration;
uint16_t distance;
int16_t baseAngle = 0;
Servo baseServo;
Servo topServo;// Creates a servo object for controlling the servo motor

void setup() {
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(9600);
    baseServo.attach(9); // Defines on which pin is the servo motor attached
    topServo.attach(11, -90, 90);
}
void loop() {

    // rotates the servo motor from 15 to 165 degrees
    for(int i=15;i<=165;i++){
        baseServo.write(i);
        delay(30);
        distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree

        //Serial.print(i); // Sends the current degree into the Serial Port
        //Serial.print(", "); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
        //Serial.print(distance); // Sends the distance value into the Serial Port
        //Serial.println("."); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    }

    if(baseAngle ==90){
        baseAngle=0;
    }

    baseAngle+=5;
    Serial.println(baseAngle);
    topServo.write(baseAngle);

    // Repeats the previous lines from 165 to 15 degrees
    for(int i=165;i>15;i--){
        baseServo.write(i);
        delay(30);
        distance = calculateDistance();
        //Serial.print(i);
        //Serial.print(", ");
        //Serial.print(distance);
        //Serial.println(".");
    }
}
// Function for calculating the distance measured by the Ultrasonic sensor
uint16_t calculateDistance(){

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH); // Reads the echoPin, returns the sound wave travel time in microseconds
    distance= duration*0.034/2;
    return distance;
}