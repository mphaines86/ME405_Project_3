#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Servo.h" //For controlling the servoe
#include "math.h" //For calculating angles

#define NUMBER_OF_SAMPLES 50 //will be used later, could be any number within bounds, really

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

const uint8_t trigPin = 8; //set pins and starting points
const uint8_t echoPin = 7;
uint32_t last_servo = 0; //the last time the base servo was moved
uint8_t rotation = 20; //The base servo angle will be called rotation and start at 20 degrees, yaw
uint8_t reverse = 0; //sweeping backwards?
uint8_t counter = 0; //used to fill distance array
uint32_t distance_array[NUMBER_OF_SAMPLES];
volatile uint32_t distance;
volatile uint32_t int_duration;
volatile uint32_t duration;
volatile uint8_t still_pulsed;
int16_t distance_old = 0;

int16_t topAngle = 50; //angle corresponding to topServo (yeah, not the best naming convetion), pitch/roll
Servo baseServo; //Creates a servo object for controlling the servo motor
Servo topServo;

void setup() {
    //pinMode(echoPin, INPUT); //Sets the echoPin as an Input
    Serial.begin(115200);
    baseServo.attach(9); //Defines on which pin is the servo motor attached
    topServo.attach(11); //pin 11, goes between 90 and -90 degrees
    baseServo.write(topAngle); //Set initial position to 0, sensor is flat with the horizontal, speakers facing straight down
    //delay(50000);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output

    cli(); //external interrupt
    PCICR |= (1 << PCIE2); //activates interrupt if pin 7 goes changes state
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
static int32_t uint32Compare(const void * a,const void * b){//used with qsort; none of us really understand what this part does
    if (*(uint32_t*)a==*(uint32_t*)b)
        return 0;
    else if (*(uint32_t*)a < *(uint32_t*)b)
        return -1;
    else
        return 1;
}

void loop() {

    if (45 < millis() - last_servo) { //Move every 20 (or more) milliseconds
        last_servo = (uint32_t) millis();
        baseServo.write(rotation); //Go to location
        if (reverse) //increment location up or down as needed
            rotation--;
        else
            rotation++;
    }

    if (rotation == 70 && reverse == 0) { //once you reach 160 (140 degree sweep), start going backwards
        reverse = 1;
        if(topAngle==140) //if now speakers are level with the horizontal, start over
            topAngle=50;
        topAngle+=5; //otherwise, just incrememnt up angle between grounda and speaker
        topServo.write(topAngle);

    }
    else if (rotation == 20 && reverse == 1){ //switch back to rotating forward 
        reverse = 0;
    }

    if (!still_pulsed) {//got return pusle, get distances and fill distance array
        counter++;
        distance_array[counter%=NUMBER_OF_SAMPLES] = duration *0.034/2;
        calculateDistance();
    }


    uint32_t tmp[NUMBER_OF_SAMPLES]; //Sort distances values and take the median; there's generally a lot of noise and this is the best quick/simple way we had to filter it
    memcpy(&tmp[0], &distance_array[0] , NUMBER_OF_SAMPLES*sizeof(uint32_t));
    qsort(tmp, NUMBER_OF_SAMPLES, sizeof(uint32_t), uint32Compare); 
    distance = tmp[NUMBER_OF_SAMPLES/2];
    int16_t changeDistance = distance_old - distance;
    if ((millis()%480) == 0) {//only reports every 120 milliseconds
        distance_old = distance;
        //if (changeDistance > 75)
        //    Serial.println("Maybe Detected an Object at distance ");
        Serial.print("d = ");
        Serial.print(distance);
        int16_t distance_floor = (uint16_t) (distance * sin(topAngle * PI / 180)); //see attached PDF for geometric analysis
        int16_t distance_x = (uint16_t) (distance_floor * cos((rotation + 40) * PI / 180)); //parallel to setup
        int16_t distance_y = (uint16_t) (distance_floor * sin((rotation + 40) * PI / 180)); //perpendicular to setup
        Serial.print(", x = ");
        Serial.print(distance_x);
        Serial.print("  y = ");
        Serial.println(distance_y);
        //Serial.print(", ");
        //Serial.println(topAngle);
    }
}


ISR(PCINT2_vect){ //Checks to see if the speaker is still waiting for a response (pulsed) by comparing the output to a value of 10000000, which only be true if its still waiting
    int value = PIND & (1 << PIND7);
    if ((value > 0) && !still_pulsed){
        int_duration = (uint32_t) micros();
        still_pulsed = 1;
    }
    else if ((value == 0) && still_pulsed){
        duration = (uint32_t) (micros() - int_duration);
        still_pulsed = 0;
    }
}
