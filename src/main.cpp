#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
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

void ping_rising();
void ping_falling();

uint16_t calculateDistance();
const uint8_t trigPin = 7;
const uint8_t echoPin = 2;
// Variables for the duration and the distance
volatile uint32_t int_duration;
volatile uint32_t duration;
volatile uint32_t distance;
volatile uint8_t still_pulsed;

int16_t baseAngle = 0;
Servo baseServo;
Servo topServo;// Creates a servo object for controlling the servo motor

void setup() {
    //pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(115200);
    baseServo.attach(9); // Defines on which pin is the servo motor attached
    topServo.attach(11, -90, 90);

    //DDRD &= ~(1 << DDD2);     // Clear the PD2 pin

    //attachInterrupt(interrupt, ISR, mode) PD2 is now an input with pull-up enabled
    //attachInterrupt(digitalPinToInterrupt(2), ping_rising, CHANGE);
    //attachInterrupt(1, ping_falling, FALLING);

    cli();
    EICRA |= (1 << ISC00);
    EIMSK |= (1 << INT0);     // Turns on INT0
    sei();
    pinMode(3, INPUT);
    pinMode(echoPin, INPUT_PULLUP);
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
}
void loop() {

    // rotates the servo motor from 15 to 165 degrees
    for(int i=15;i<=165;i++){
        baseServo.write(i);
        //delay(30);
        distance = calculateDistance();// Calls a function for calculating the distance measured by the Ultrasonic sensor for each degree

        //Serial.print(i); // Sends the current degree into the Serial Port
        //Serial.print(", "); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
        //Serial.print(distance); // Sends the distance value into the Serial Port
        //Serial.println("."); // Sends addition character right next to the previous value needed later in the Processing IDE for indexing
    }
    delay(1000);
    if(baseAngle ==90){
        baseAngle=0;
    }

    baseAngle+=5;
    //Serial.println(baseAngle);
    topServo.write(baseAngle);

    // Repeats the previous lines from 165 to 15 degrees
    for(int i=165;i>15;i--){
        baseServo.write(i);
        //delay(30);
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
}

void ping_rising(){
    if (!(PIND && PD2))
        Serial.println(PIND & PD2);
    int_duration = millis();
}

ISR(INT0_vect){
    int value = PIND & PD2;
    //duration = pulseIn(echoPin, LOW); // Reads the echoPin, returns the sound wave travel time in microseconds
    //distance= duration*0.034/2;
    if ((value == 0) && !still_pulsed){
        //Serial.println("test1");
        int_duration = millis();
        still_pulsed = 1;
    }
    else if ((value > 0) && still_pulsed){
        duration = millis() - int_duration;
        still_pulsed = 0;
        distance = duration *0.034/2;
        //Serial.println("Winner!");
        //Serial.println(distance);
    }
    Serial.print(PIND & 0x03);
}