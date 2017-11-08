#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Servo.h"
#include "math.h"

#define NUMBER_OF_SAMPLES 50

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

const uint8_t trigPin = 8;
const uint8_t echoPin = 7;
uint32_t last_servo = 0;
uint8_t rotation = 0;
uint8_t reverse = 0;
uint8_t counter = 0;
uint32_t distance_array[NUMBER_OF_SAMPLES];
volatile uint32_t distance;
volatile uint32_t int_duration;
volatile uint32_t duration;
volatile uint8_t still_pulsed;

int16_t baseAngle = 0;
Servo baseServo;
Servo topServo;// Creates a servo object for controlling the servo motor

void setup() {
    //pinMode(echoPin, INPUT); // Sets the echoPin as an Input
    Serial.begin(115200);
    baseServo.attach(9); // Defines on which pin is the servo motor attached
    topServo.attach(11, -90, 90);
    topServo.write(0);

    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output

    cli();
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT23);
    sei();
}

void calculateDistance(){

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

// Need for this function varies
static int32_t uint32Compare(const void * a,const void * b)
{
    if (*(uint32_t*)a==*(uint32_t*)b)
        return 0;
    else if (*(uint32_t*)a < *(uint32_t*)b)
        return -1;
    else
        return 1;
}

void loop() {

    if (20 < millis() - last_servo) {
        last_servo = (uint32_t) millis();
        baseServo.write(rotation);
        if (reverse)
            rotation--;
        else
            rotation++;
    }

    if (rotation == 140 && reverse == 0) {
        reverse = 1;
        if(baseAngle==90)
            baseAngle=0;
        baseAngle+=5;
        topServo.write(baseAngle);

    }
    else if (rotation == 0 && reverse == 1){
        reverse = 0;
    }

    if (!still_pulsed)
        //counter++;
        //distance_array[counter%=NUMBER_OF_SAMPLES] = duration *0.034/2;
        calculateDistance();

    //uint32_t tmp[NUMBER_OF_SAMPLES];
    //memcpy(&tmp[0], &distance_array[0] , NUMBER_OF_SAMPLES*sizeof(uint32_t));
    //qsort(tmp, NUMBER_OF_SAMPLES, sizeof(uint32_t), uint32Compare);
    // distance = tmp[NUMBER_OF_SAMPLES/2]
    if ((millis()%120) == 0) {
        Serial.println(distance);
        uint16_t distance_x = (uint16_t) (distance * cos(baseAngle * PI / 180));
        uint16_t distance_y = (uint16_t) (distance * sin(baseAngle * PI / 180));
        Serial.print("x, y distance: ");
        Serial.print(distance_x);
        Serial.print(", ");
        Serial.print(distance_y);
        Serial.print(", ");
        Serial.println(baseAngle);
    }
}


ISR(PCINT2_vect){
    int value = PIND & (1 << PINB7);
    //duration = pulseIn(echoPin, LOW); // Reads the echoPin, returns the sound wave travel time in microseconds
    //distance= duration*0.034/2;
    if ((value > 0) && !still_pulsed){
        //Serial.println("test1");
        int_duration = (uint32_t) micros();
        still_pulsed = 1;
    }
    else if ((value == 0) && still_pulsed){
        duration = (uint32_t) (micros() - int_duration);
        still_pulsed = 0;
        distance = (uint32_t) (duration * 0.034 / 2);
    }
    //Serial.println(PIND & (1 << PINB7));
}