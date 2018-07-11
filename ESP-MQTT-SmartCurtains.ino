//ESP-MQTT-SmartCurtains
//Version 1.0
//July 10 2018
//https://github.com/DrCoolHands/ESP-MQTT-SmartCurtains

#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <Arduino.h>
#include <Chrono.h>


/************ WIFI and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
#define wifi_ssid "WifiSSID" //type your WIFI information inside the quotes
#define wifi_password "WIFIPassword"
#define mqtt_server "mqtt.domain.com"
#define mqtt_user "mqttuser" 
#define mqtt_password "mqttpassword"
#define mqtt_port 1883

/************* MQTT TOPICS (change these topics as you wish)  **************************/
#define command_topic "smartcurtains/curtains/set"
#define state_topic "smartcurtains/curtains/state"
#define position_topic "smartcurtains/curtains/position"

const char* open_cmd = "OPEN";
const char* close_cmd = "CLOSE";
const char* stop_cmd = "STOP";
const char* opened_state = "OPENED";
const char* closed_state = "CLOSED";

/************* HARDWARE PIN DEFINITIONS (Set to match your hardware)***********************/
#define stepPin D0
#define dirPin D1
#define enablePin D2
#define endstopPin D3

/************************* MOVEMENT DEFINITIONS ******************************************/
#define travelDistance 150 //Travel distance in mm
#define stepsPerMM 80 //8th step
#define travelSpeed 200 //mm per second
#define acceleration 1500 //Accel and Decell rates
#define homingRatio 0.5 //Speed ratio of homing compared to regular movements
#define motorTimeout 300 //300 second time out before disabeling the motors

/************************* STEPPER DEFINITIONS ******************************************/
#define dirInvert false
#define stepInvert false
#define enableInvert true

/**************************** GLOBALS ********************************************/
/*********** Shouldn't need to change anything beyond here ***********************/
#define MQTT_MAX_PACKET_SIZE 512
const float stepSpeed = stepsPerMM * travelSpeed;
const long maxSteps = travelDistance * stepsPerMM;
long finalDestination = 0;
bool knowsHome, moving;

WiFiClient espClient;
PubSubClient client(espClient);
AccelStepper stepper(1,stepPin,dirPin);
Chrono motorTimer(Chrono::SECONDS);

/********************************** START SETUP*****************************************/
void setup() {
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  while (!Serial)  // Wait for the serial connection to be establised.
    delay(50);
  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  stepper.setPinsInverted( dirInvert, stepInvert, enableInvert );
  stepper.setEnablePin(enablePin);
  stepper.setMaxSpeed(stepSpeed);
  stepper.setAcceleration(acceleration);

  pinMode(endstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(endstopPin), endstopInterrupt, FALLING);

  Serial.println();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());
  reconnect();
  Serial.println("ESP-MQTT-SmartCurtains is now running....");

  goTo(0); //Home to endstop

}

/**********************************Endstop goes low, stop, publish state, set home*****************************************/
void endstopInterrupt(){
  stepper.setCurrentPosition(0);
  knowsHome = true;
  
  if (moving && stepper.targetPosition() == 0) {
    Serial.println("Endstop triggered.");
    moving = false;
    stepper.stop();
    stepper.setSpeed(stepSpeed); //Go Full Speed!    
    publishPosition();
    stepper.setAcceleration(acceleration); //Resume regular acceleration
    if (finalDestination != 0)
      goTo(finalDestination);
  }
  
}

/********************************** START SETUP WIFI*****************************************/
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/********************************** START MQTT CALLBACK*****************************************/

void callback(char* topic, byte* payload, unsigned int length) {

  char* payload_str;
  payload_str = (char*) malloc(length + 1);
  memcpy(payload_str, payload, length);
  payload_str[length] = '\0';
  Serial.println(String(payload_str));
  
  if ( String(topic) == command_topic ) {
    if (String(payload_str) == open_cmd ) {
      goTo(maxSteps);
      Serial.println("Received Open Command - publishing open state");        
    } else if ( String(payload_str) == close_cmd ) {
      goTo(0);
      Serial.println("Received close command");
    } else if ( String(payload_str) == stop_cmd ) {
      stepper.setAcceleration(80000); //stop REALLY fast
      stepper.stop();
      stepper.runToPosition();
      stepper.setAcceleration(acceleration);
      Serial.println("Received stop Command");
    } else {
      Serial.print("I do not know what to do with ");
      Serial.print(String(payload_str));
      Serial.print(" on topic ");
      Serial.println( String(topic));
    }
  }
}

/**********************Move to destination, or home first if requried***************************/
void goTo(long destination){
  finalDestination = destination;
  stepper.enableOutputs();
  
  if (!knowsHome && digitalRead(endstopPin) == HIGH){
    Serial.println("Homing....");
    stepper.setCurrentPosition(maxSteps*1.25); //Expect we're fully open and a bit more.
    stepper.setSpeed(stepSpeed * homingRatio); // homing speed
    stepper.moveTo(0); //Go Home!
  } else if (!knowsHome && digitalRead(endstopPin) == LOW)
    endstopInterrupt();      
  else stepper.moveTo(destination);

  moving = true;
}

/**********************Print Debug info***************************/
void printDebug(){
  Serial.print("Destination: ");
  Serial.println(finalDestination);
  Serial.print("Moving var: ");
  Serial.println(moving);
  Serial.print("Knows home: ");
  Serial.println(knowsHome);
  Serial.print("Endstop status: ");
  Serial.println(digitalRead(endstopPin));
  Serial.print("Current Position: ");
  Serial.println(stepper.currentPosition());
}

/**********************************Calc location as % and publish *****************************************/
void publishPosition(){
  String percentage = String( (int)((float(stepper.currentPosition())/float(maxSteps))*100) );
  char buffer[percentage.length()+1];
  percentage.toCharArray(buffer,sizeof(buffer));
  client.publish(position_topic, buffer);
  
  Serial.print("Stopped at percentage: ");
  Serial.println(percentage);

  if (stepper.currentPosition() == 0){
    client.publish(state_topic, closed_state, true);
    Serial.println("At closed position. Publishing state.");
  } else {
    client.publish(state_topic, opened_state, true); //cover is concidered opened even at 1%
    Serial.println("Publishing open state");
  }

  motorTimer.restart();
  motorTimer.resume();
 
}

/********************************** START RECONNECT*****************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SmartCurtain", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(command_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/********************************** START MAIN LOOP***************************************/
void loop() {
  if (!client.connected())
    software_Reset();
  client.loop();

  if (moving && stepper.distanceToGo() == 0){
    publishPosition();
    moving = false;
  }

  if (motorTimer.hasPassed(motorTimeout,true)){
    motorTimer.stop();
    stepper.disableOutputs();
    knowsHome = false;
    Serial.println("Motor timeout triggered");
  }

  stepper.run(); //Run the stepper motor if a step is pending.

}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
  Serial.print("resetting");
  ESP.reset(); 
}