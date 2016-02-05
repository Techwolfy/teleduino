//Teleduino: Arduino Uno I/O device for Telepresence

//Protocol definition:
//Command: [command byte, channel byte, data byte, data byte]
//Response: [data byte, data byte, data byte, data byte]
//Command 'A': return selected analog input (range is 0 to 1023)
//		Input: ignored
//		Output: low byte, high byte, 0x00
//Command 'C': return selected encoder count
//		Input: ignored
//		Output: low byte, low mid byte, high mid byte, high byte
//Command 'D': set digital output value
//		Input: set high if first data byte > 0, low otherwise
//		Output: none
//Command 'E': return selected encoder speed
//		Input: ignored
//		Output: 32-bit IEEE 754 float
//Command 'P': set PWM output value (range is 1000 to 2000, 1500 neutral)
//		Input: first data byte is low byte of value, second is high byte
//		Output: none

//Port map:
//0: Serial RX
//1: Serial TX
//2: Encoder input 0-0
//3: Encoder input 1-0
//4: Encoder input 0-1
//5: Servo output 1
//6: Servo output 2
//7: Encoder input 0-2
//8: Digital output 0
//9: Servo output 3
//10: Servo output 4
//11: Servo output 5
//12: Digital output 1
//13: Digital output 2
//A0: Analog input 0
//A1: Analog input 1
//A2: Analog input 2
//A3: Analog input 3
//A4: Analog input 4
//A5: Analog input 5


//Board-specific configuration
//PWM outputs
const int numServoOutputs = 5;
const int servoPorts[numServoOutputs] = {5, 6, 9, 10, 11};
//Digital outputs
const int numDigitalOutputs = 3;
const int digitalPorts[numDigitalOutputs] = {8, 12, 13};
//Analog inputs
const int numAnalogInputs = 6;
const int analogPorts[numAnalogInputs] = {A0, A1, A2, A3, A4, A5};
//Encoder inputs
const int numEncoderInputs = 2;
const int encoderPorts[numEncoderInputs][2] = {{2, 4}, {3, 7}};


//Includes
#include <Arduino.h>
#include <Servo.h>
#include <Encoder.h>	//https://www.pjrc.com/teensy/td_libs_Encoder.html

//Union for serial transmission
typedef union {
  int32_t asInt32;
  float asFloat;
  unsigned char asBytes[4];
} packet;

//Global variables
//Serial data data
unsigned char data[4] = {0};
packet out = {0};
//Servos and servo states
Servo servos[numServoOutputs];
int servoStates[numServoOutputs] = {0};
//Digital I/O states
bool digitalStates[numDigitalOutputs] = {0};
//Encoders and encoder states
Encoder *encoders[numEncoderInputs];
int32_t lastEncoderInput[numEncoderInputs];

//Initial setup
void setup() {
	//Initialize encoders
	for(int i = 0; i < numEncoderInputs; i++) {
		//TODO: Initialize encoders statically, if possible
		//Note: Dynamic initialization; encoders are used until arduino is unplugged, so no destructor is necessary
		encoders[i] = new Encoder(encoderPorts[i][0], encoderPorts[i][1]);
	}

	//Initialize servos
	for(int i = 0; i < numServoOutputs; i++) {
		servos[i].attach(servoPorts[i]);
	}

	//Initialize digital outputs
	for(int i = 0; i < numDigitalOutputs; i++) {
		pinMode(digitalPorts[i], OUTPUT);
	}

	//Start serial connection: 115200 8 N 1, full duplex
	//Note: Uses both USB and pins 0 and 1 for communication
	Serial.begin(115200);
	while(!Serial) {
		delay(10);  //Wait for a serial connection
	}
}

//Main program loop
void loop() {
	//For each command data:
	//Note: Uses logical AND short-circuit to prevent errors
	if(Serial.available() >= 4 && Serial.readBytes(data, 4) == 4) {
		switch(data[0]) {
			case 'A':	//Get analog input value
				if(data[1] < numAnalogInputs) {
					//Write 32-bit IEEE 754 float directly to serial output
					//Convert analogRead's (0, 1023) to physical (0.0V, 5.0V)
					out = {.asFloat = analogRead(analogPorts[data[1]]) * (5.0f / 1023.0f)};
					Serial.write(out.asBytes, 4);
				}
				break;
			case 'C':	//Get encoder count
				if(data[1] < numEncoderInputs) {
					//Write 32-bit signed int directly to serial output
					out = {.asInt32 = encoders[numEncoderInputs]->read()};
					Serial.write(out.asBytes, 4);
				}
				break;
			case 'D':	//Set digital output value
				if(data[1] < numDigitalOutputs) {
					digitalStates[data[1]] = data[2] > 0;
				}
				break;
			case 'E':	//Get encoder speed
				if(data[1] < numEncoderInputs) {
					//Calculate encoder speed: currentPos - lastPos / deltaTime (10ms in seconds)
					out = {.asFloat = encoders[numEncoderInputs]->read() - lastEncoderInput[numEncoderInputs] / 0.01f};
					//Write 32-bit IEEE 754 float directly to serial output
					Serial.write(out.asBytes, 4);
				}
				break;
			case 'P':	//Set PWM output value
				if(data[1] < numServoOutputs) {
					servoStates[data[1]] = (data[3] << 8) | data[2];
				}
				break;
			default:
				//Invalid command; do nothing
				break;
		}
	}

	//Run servos
	for(int i = 0; i < numServoOutputs; i++) {
		servos[i].writeMicroseconds(servoStates[i]);
	}

	//Run digital outputs
	for(int i = 0; i < numDigitalOutputs; i++) {
		digitalWrite(digitalPorts[i], digitalStates[i]);
	}

	for(int i = 0; i < numEncoderInputs; i++) {
		lastEncoderInput[i] = encoders[numEncoderInputs]->read();
	}

	//No delay; aside from encoder interrupts, program runs in a tight loop
}
