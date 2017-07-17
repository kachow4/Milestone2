//generalized wave freq detection with 38.5kHz sampling rate and interrupts
//by Amanda Ghassaei
//https://www.instructables.com/id/Arduino-Frequency-Detection/
//Sept 2012

/*
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

*/

#include <Servo.h>
Servo servo;
int position = 90;
int currentFrequency = 0;
int nextFrequency = 0;

//clipping indicator variables
boolean clipping = 0;

//data storage variables
byte newData = 0;
byte prevData = 0;
unsigned int time = 0;//keeps time and sends vales to store in timer[] occasionally
int timer[10];//sstorage for timing of events
int slope[10];//storage for slope of events
unsigned int totalTimer;//used to calculate period
unsigned int period;//storage for period of wave
byte index = 0;//current storage index
int frequency;//storage for frequency calculations
int maxSlope = 0;//used to calculate max slope as trigger point
int newSlope;//storage for incoming slope data

//variables for decided whether you have a match
byte noMatch = 0;//counts how many non-matches you've received to reset variables if it's been too long
byte slopeTol = 2;//slope tolerance- adjust this if you need
int timerTol = 2;//timer tolerance- adjust this if you need

//variables for amp detection
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 50;//raise if you have a very noisy signal

void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT); //led indicator pin
  pinMode(11, OUTPUT); //output pin
  servo.attach(6);

  cli();//diable interrupts

  //set up continuous sampling of analog pin 0 at 38.5kHz

  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); //set reference voltage
  ADMUX |= (1 << ADLAR); //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
  ADCSRA |= (1 << ADATE); //enabble auto trigger
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADSC); //start ADC measurements

  sei();//enable interrupts
}

ISR(ADC_vect) {//when new ADC value ready

  PORTB &= B11101111;//set pin 12 low
  prevData = newData;//store previous value
  newData = ADCH;//get value from A0
  if (prevData < 127 && newData >= 127) { //if increasing and crossing midpoint
    newSlope = newData - prevData;//calculate slope
    if (abs(newSlope - maxSlope) < slopeTol) { //if slopes are ==
      //record new data and reset time
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0) { //new max slope just reset
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
        index++;//increment index
      }
      else if (abs(timer[0] - timer[index]) < timerTol && abs(slope[0] - newSlope) < slopeTol) { //if timer duration and slopes match
        //sum timer values
        totalTimer = 0;
        for (byte i = 0; i < index; i++) {
          totalTimer += timer[i];
        }
        period = totalTimer;//set period
        //reset new zero index values to compare with
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1;//set index to 1
        PORTB |= B00010000;//set pin 12 high
        noMatch = 0;
      }
      else { //crossing midpoint but not match
        index++;//increment index
        if (index > 9) {
          reset();
        }
      }
    }
    else if (newSlope > maxSlope) { //if new slope is much larger than max slope
      maxSlope = newSlope;
      time = 0;//reset clock
      noMatch = 0;
      index = 0;//reset index
    }
    else { //slope not steep enough
      noMatch++;//increment no match counter
      if (noMatch > 9) {
        reset();
      }
    }
  }

  if (newData == 0 || newData == 1023) { //if clipping
    PORTB |= B00100000;//set pin 13 high- turn on clipping indicator led
    clipping = 1;//currently clipping
  }

  time++;//increment timer at rate of 38.5kHz

  ampTimer++;//increment amplitude timer
  if (abs(127 - ADCH) > maxAmp) {
    maxAmp = abs(127 - ADCH);
  }
  if (ampTimer == 1000) {
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}

void reset() { //clea out some variables
  index = 0;//reset index
  noMatch = 0;//reset match couner
  maxSlope = 0;//reset slope
}

void checkClipping() { //manage clipping indicator LED
  if (clipping) { //if currently clipping
    PORTB &= B11011111;//turn off clipping indicator led
    clipping = 0;
  }
}

//int C4frequencies[14] = {267, 265, 263, 261, 159, 158, 132, 131, 77, 76, 46, 32, 29, 13};  //261.6 Hz 265.26
//int G4frequencies[14] = {801, 784, 400, 396, 392, 272, 198, 196, 98, 97, 78, 65, 51, 43};  //392.0 Hz 396.52
//int C5frequencies[14] = {534, 526, 519, 259, 175, 174, 139, 123, 110, 104, 74, 52, 47, 40}; //523.3 Hz 526.88

int C4frequencies[6] = {265, 263, 261, 131, 88, 76};  //261.6 Hz 265.26
int G4frequencies[6] = {400, 396, 392, 196, 65, 43};  //392.0 Hz 396.52
int E4frequencies[6] = {331, 328, 164, 109, 82, 66}; //523.3 Hz 526.88

void loop() {

  checkClipping();

  if (checkMaxAmp > ampThreshold) {
    frequency = 38462 / float(period); //calculate frequency timer rate/period
    //print results
    //Serial.print(frequency);
    //Serial.println(" hz");
    /*
        if (currentFrequency == 0) {
          if (frequency >= 260 && frequency <= 265) {
            currentFrequency = frequency;
          }
        }
        if (currentFrequency >= 260 && currentFrequency <= 265) {
          if (frequency >= 328 && frequency <= 331) {
            currentFrequency = frequency;
          }
        }
        if (currentFrequency >= 328 && currentFrequency <= 331) {
          if (frequency >= 392 && frequency <= 396) {
            currentFrequency = frequency;
          }
        }
        if (currentFrequency >= 392 && currentFrequency <= 396) {
          if (frequency >= 519 && frequency <= 539) {
            servo.write(position += 90);
            currentFrequency = frequency;
          }
        }
        if (currentFrequency >= 519 && currentFrequency <= 539) {
          if (frequency >= 261 && frequency <= 265) {
            servo.write(position -= 90);
            currentFrequency = 0;
          }
        }
      }
      delay(200);
      }
    */

    if (currentFrequency == 0) {
      if (frequency >= 310 && frequency <= 312) {
        digitalWrite(7, HIGH);
        currentFrequency = frequency;
        Serial.println(currentFrequency);
      }
    }
    if (currentFrequency >= 310 && currentFrequency <= 312) {
      if (frequency >= 260 && frequency <= 265) {
        digitalWrite(8, HIGH);
        currentFrequency = frequency;
        Serial.println(currentFrequency);
      }
    }
    if (currentFrequency >= 260 && currentFrequency <= 265) {
      if (frequency >= 310 && frequency <= 312) {
        if (nextFrequency == 0) {
          digitalWrite(9, HIGH);
          nextFrequency = frequency;
          Serial.println(nextFrequency);
        }
      }
    }
    if (nextFrequency >= 310 && nextFrequency <= 312) {
      if (frequency >= 392 && frequency <= 396) {
        digitalWrite(10, HIGH);
        nextFrequency = frequency;
        Serial.println(nextFrequency);
      }
    }
    if (nextFrequency >= 392 && nextFrequency <= 396) {
      if (frequency >= 519 && frequency <= 539) {
        digitalWrite(11, HIGH);
        nextFrequency = frequency;
        Serial.println(nextFrequency);
        servo.write(position -= 90);
      }
    }
    if (nextFrequency >= 519 && nextFrequency <= 539) {
      if (frequency >= 260 && frequency <= 265) {
        digitalWrite(7, LOW);
        digitalWrite(8, LOW);
        digitalWrite(9, LOW);
        digitalWrite(10, LOW);
        digitalWrite(11, LOW);
        servo.write(position += 90);
        currentFrequency = 0;
        nextFrequency = 0;
        Serial.println(currentFrequency);
        Serial.println(nextFrequency);
      }
    }
  }
  delay(200);
}




/*for(int i = 0; i < 6; i++){
  if(frequency == C4frequencies[i]){
    Serial.println("C4");
  }
  if(frequency == G4frequencies[i]){
    Serial.println("G4");
  }
  if(frequency == E4frequencies[i]){
    Serial.println("E4");
  }
  }*/
/*if(frequency >= 260 && frequency <= 265){
      Serial.println("C4");
      digitalWrite(11, HIGH);
      Serial.println("LET THERE BE LIGHT!");
    }
    if(frequency >= 328 && frequency <= 331){
      Serial.println("E4");
      digitalWrite(10, HIGH);
    }
    if(frequency >= 392 && frequency <= 396){
      Serial.println("G4");
      digitalWrite(9, HIGH);
    }
    if(frequency >= 519 && frequency <= 539){
      Serial.println("C5");
      digitalWrite(8, HIGH);
    }*/

/*if (frequency >= 260 && frequency <= 265) {
  Serial.println("C4");
  digitalWrite(11, HIGH);
  }
  if (digitalRead(11) == HIGH) {
  if (frequency >= 328 && frequency <= 331) {
    Serial.println("E4");
    digitalWrite(10, HIGH);
  }
  else if (frequency < 328 || frequency > 331) {
    if (frequency < 260 || frequency > 265) {
      if (frequency > 100) {
        digitalWrite(11, LOW);
      }
    }
  }
  }
  if (digitalRead(10) == HIGH) {
  if (frequency >= 392 && frequency <= 396) {
    Serial.println("G4");
    digitalWrite(9, HIGH);
  }
  else if (frequency < 392 || frequency > 396) {
    if (frequency < 328 || frequency > 331) {
      if (frequency < 260 || frequency > 265) {
        if (frequency > 100) {
          digitalWrite(11, LOW);
          digitalWrite(10, LOW);
        }
      }
    }
  }
  }
  if (digitalRead(9) == HIGH) {
  if (frequency >= 519 && frequency <= 539) {
    Serial.println("C5");
    digitalWrite(8, HIGH);
    servo.write(position += 180);
  }
  else if (frequency < 519 || frequency > 539) {
    if (frequency < 392 || frequency > 396) {
      if (frequency < 328 || frequency > 331) {
        if (frequency < 260 || frequency > 265) {
          if (frequency > 100) {
            digitalWrite(11, LOW);
            digitalWrite(10, LOW);
            digitalWrite(9, LOW);
          }
        }
      }
    }
  }
*/
/*
  if (frequency >= 310 && frequency <= 312) {
  digitalWrite(11, HIGH);
  }
  if (digitalRead(11) == HIGH) {
  if (frequency >= 260 && frequency <= 265) {
    digitalWrite(10, HIGH);
  }
  }
  if (digitalRead(10) == HIGH) {
  if (frequency >= 310 && frequency <= 312) {
    digitalWrite(9, HIGH);
  }
  }
  if (digitalRead(9) == HIGH) {
  if (frequency >= 392 && frequency <= 396) {
    digitalWrite(8, HIGH);
  }
  }
  if (digitalRead(8) == HIGH) {
  if (frequency >= 519 && frequency <= 539) {
    servo.write(position += 180);
  }
  }
  if (digitalRead(11) == HIGH && digitalRead(10) == HIGH && digitalRead(9) == HIGH && digitalRead(8) == HIGH) {
  if (frequency >= 260 && frequency <= 265) {
  }
  servo.write(position = 0);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  }
  }*/
/*
  if (digitalRead(11) == LOW) {
  if (digitalRead(10) == HIGH) {
    if (frequency >= 620 && frequency <= 630) {
      digitalWrite(10, LOW);
    }
  }
  }
  if (digitalRead(10) == LOW) {
  if (digitalRead(9) == HIGH) {
    if (frequency >= 582 && frequency <= 591) {
      digitalWrite(9, LOW);
    }
  }
  }
  if (digitalRead(9) == LOW) {
  if (digitalRead(8) == HIGH) {
    if (frequency >= 519 && frequency <= 539) {
      digitalWrite(8, LOW);
    }
  }
  }
  if (digitalRead(8) == LOW) {
  if (frequency == 493) {
    digitalWrite(11, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(8, HIGH);
  }
  }*/
/*else{
  digitalWrite(ledPin, LOW);
  }*/


