// All of the LED code contained within comes from:

/*
 * ESP8266 MQTT Lights for Home Assistant.
 *
 * Created DIY lights for Home Assistant using MQTT and JSON.
 * This project supports single-color, RGB, and RGBW lights.
 *
 * Copy the included `config-sample.h` file to `config.h` and update
 * accordingly for your setup.
 *
 * See https://github.com/corbanmailloux/esp-mqtt-rgb-led for more information.
 */

 // My code includes the overall structure, DHT logic,  and the pushbutton functions

#include<cmath>

#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>         //MQTT
#include <ESP8266mDNS.h>          // mDNS
#include <ESP8266HTTPUpdateServer.h> // OTA Updates
#include "DHT.h"


// Set configuration options for LED type, pins, WiFi, and MQTT in the following file:
#include "config.h"

// https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>

unsigned long last_reconnect = 0;
const unsigned long reconnect_wait = 3000;

// app specific variables


const bool rgb = (CONFIG_STRIP == RGB) || (CONFIG_STRIP == RGBW);
const bool includeWhite = (CONFIG_STRIP == BRIGHTNESS) || (CONFIG_STRIP == RGBW);

const int BUFFER_SIZE = JSON_OBJECT_SIZE(20);

// Maintained state for reporting to HA
byte red = 255;
byte green = 255;
byte blue = 255;
byte white = 255;
byte brightness = 255;
int color_temp = 0;

// Last state
bool last_stateOn = false;
byte last_red = 0;
byte last_green = 0;
byte last_blue = 0;
byte last_white = 0;
byte last_brightness = 0;
int last_color_temp = 0;

// Real values to write to the LEDs (ex. including brightness and state)
byte realRed = 0;
byte realGreen = 0;
byte realBlue = 0;
byte realWhite = 0;

bool stateOn = false;

// Globals for fade/transitions
bool startFade = false;
unsigned long lastLoop = 0;
int transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB, stepW;
int redVal, grnVal, bluVal, whtVal;

// Globals for flash
bool flash = false;
bool startFlash = false;
int flashLength = 0;
unsigned long flashStartTime = 0;
byte flashRed = red;
byte flashGreen = green;
byte flashBlue = blue;
byte flashWhite = white;
byte flashBrightness = brightness;

// Globals for colorfade
bool colorfade = false;
int currentColor = 0;
// {red, grn, blu, wht}
const byte colors[][4] = {
  {255, 0, 0, 0},   // red
  {0, 255, 0, 0},   // green
  {255, 192, 203,0},// pink
  {0, 0, 255, 0},   // blue
  {255, 80, 0, 0},  // orange
  {163, 0, 255, 0}, // purple
  {0, 255, 255, 0}, // auqua
  {255, 255, 0, 0}  // yellow
};
const int numColors = 8;


// button press variables

long buttonTimer = 0;
long debounceTime = 50;
long longPressTime = 750;

boolean buttonActive = false;
boolean longPressActive = false;

const byte presets[][5] = {
  // {red, green, blue, white, brightness}
  {255, 255, 255, 255, 255},      // Bright white
  
//  {255, 255, 255, 0, 250},  // Sunlight
//  {255, 244, 229, 0, 250},  // Warm white
//  {212, 235, 255, 0, 250},  // Cool white
//  {255, 239, 247, 0, 200},  // Grow light
//  {255, 197, 143, 0, 150},  // tungsten

//  {255,179,186, 0, 200},           //  pastel red
//  {255,223,186, 0, 200},           //  pastel orange
//  {255,255,186, 0, 200},           //  pastel yellow
//  {186,255,201, 0, 200},           //  pastel green
//  {186,225,255, 0, 200},           //  pastel blue

  {51,255,153, 0, 255},           //  medium green
  {255,51,153, 0, 255},           //  pink
  {51,153,255, 0, 255},           //  medium blue
  {255,153,51, 0, 255},           //  orange
  {153,51,255, 0, 255},           //  purple
  {255,255,102, 0, 255},          //  goldenrod
  
  {255, 49, 0, 6, 255},       // nightlight
  {0, 0, 0, 0}                //  Off
};
const int numPresets = 9;

int currentPreset = 0;

// DHT 
#define DHTPIN D3
#define DHTTYPE DHT22
long lastTempReport = 0;

DHT dht(DHTPIN, DHTTYPE);


//Setup the web server for http OTA updates. 
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient espClient;

//Initialize MQTT
PubSubClient client(espClient);

//Wifi Manager will try to connect to the saved AP. If that fails, it will start up as an AP
//which you can connect to and setup the wifi
WiFiManager wifiManager;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Begin setup");

  //Set the wifi config portal to only show for 3 minutes, then continue.
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(CONFIG_OTA_HOST);
  
  //sets up the mqtt server, and sets callback() as the function that gets called
  //when a subscribed topic has data
  client.setServer(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT);
  client.setCallback(callback); //callback is the function that gets called for a topic sub
  
//  while (!client.connected()) {
    reconnect();
//  }

  //setup http firmware update page.
  MDNS.begin(CONFIG_OTA_HOST);
  httpUpdater.setup(&httpServer, CONFIG_OTA_PATH, CONFIG_OTA_USER, CONFIG_OTA_PASS);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and your password\n", CONFIG_OTA_HOST, CONFIG_OTA_PATH, CONFIG_OTA_USER);

  // do setup stuff
  if (rgb) {
    pinMode(CONFIG_PIN_RED, OUTPUT);
    pinMode(CONFIG_PIN_GREEN, OUTPUT);
    pinMode(CONFIG_PIN_BLUE, OUTPUT);
  }
  if (includeWhite) {
    pinMode(CONFIG_PIN_WHITE, OUTPUT);
  }

  // Set the BUILTIN_LED based on the CONFIG_BUILTIN_LED_MODE
  switch (CONFIG_BUILTIN_LED_MODE) {
    case 0:
      pinMode(BUILTIN_LED, OUTPUT);
      digitalWrite(BUILTIN_LED, LOW);
      break;
    case 1:
      pinMode(BUILTIN_LED, OUTPUT);
      digitalWrite(BUILTIN_LED, HIGH);
      break;
    default: // Other options (like -1) are ignored.
      break;
  }

  analogWriteRange(255);

  strip_test();
  next_preset();

  dht.begin();
}

void strip_test(){
  // turn all colors off
  setColor(0, 0, 0, 0);
  digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);
  delay(500);

  if(rgb){    
    // just red
    setColor(255, 0, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_ON);
    delay(500);
    setColor(0, 0, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);
    delay(500);
    // just green
    setColor(0, 255, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_ON);
    delay(500);
    setColor(0, 0, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);
    delay(500);
    // just blue
    setColor(0, 0, 255, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_ON);
    delay(500);
    setColor(0, 0, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);
    delay(500);
  }

  if(includeWhite){
    setColor(0, 0, 0, 255);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_ON);
    delay(500);
    setColor(0, 0, 0, 0);
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);
    delay(500);
  }
}

void loop() {
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }

  // do stuff
  handle_led_functions();
  handlePushButton();
  handle_dht();

  client.loop(); //the mqtt function that processes MQTT messages
  httpServer.handleClient(); //handles requests for the firmware update page
}

  /*
  SAMPLE PAYLOAD (BRIGHTNESS):
    {
      "brightness": 120,
      "flash": 2,
      "transition": 5,
      "state": "ON"
    }
  SAMPLE PAYLOAD (RGBW):
    {
      "brightness": 120,
      "color": {
        "r": 255,
        "g": 100,
        "b": 100
      },
      "white_value": 255,
      "flash": 2,
      "transition": 5,
      "state": "ON",
      "effect": "colorfade_fast"
    }
  */

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (!processJson(message)) {
    return;
  }

  if (stateOn) {
    // Update lights
    if(color_temp > 0){
      realRed = map(mired_to_red(color_temp), 0, 255, 0, brightness);
      realGreen = map(mired_to_green(color_temp), 0, 255, 0, brightness);
      realBlue = map(mired_to_blue(color_temp), 0, 255, 0, brightness);
    } else {
      realRed = map(red, 0, 255, 0, brightness);
      realGreen = map(green, 0, 255, 0, brightness);
      realBlue = map(blue, 0, 255, 0, brightness);
    }
    realWhite = map(white, 0, 255, 0, brightness);
  }
  else {
    realRed = 0;
    realGreen = 0;
    realBlue = 0;
    realWhite = 0;
  }

  startFade = true;
  inFade = false; // Kill the current fade

  sendState();
}

bool processJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);

  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }

  if (root.containsKey("state")) {
    if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_ON) == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], CONFIG_MQTT_PAYLOAD_OFF) == 0) {
      stateOn = false;
      last_stateOn = stateOn;
    }
  }

  // If "flash" is included, treat RGB and brightness differently
  if (root.containsKey("flash") ||
       (root.containsKey("effect") && strcmp(root["effect"], "flash") == 0)) {

    if (root.containsKey("flash")) {
      flashLength = (int)root["flash"] * 1000;
    }
    else {
      flashLength = CONFIG_DEFAULT_FLASH_LENGTH * 1000;
    }

    if (root.containsKey("brightness")) {
      flashBrightness = root["brightness"];
    }
    else {
      flashBrightness = brightness;
    }

    if (rgb && root.containsKey("color")) {
      flashRed = root["color"]["r"];
      flashGreen = root["color"]["g"];
      flashBlue = root["color"]["b"];
    }
    else {
      flashRed = red;
      flashGreen = green;
      flashBlue = blue;
    }

    if (includeWhite && root.containsKey("white_value")) {
      flashWhite = root["white_value"];
    }
    else {
      flashWhite = white;
    }

    flashRed = map(flashRed, 0, 255, 0, flashBrightness);
    flashGreen = map(flashGreen, 0, 255, 0, flashBrightness);
    flashBlue = map(flashBlue, 0, 255, 0, flashBrightness);
    flashWhite = map(flashWhite, 0, 255, 0, flashBrightness);

    flash = true;
    startFlash = true;
  }
  else if (rgb && root.containsKey("effect") &&
      (strcmp(root["effect"], "colorfade_slow") == 0 || strcmp(root["effect"], "colorfade_fast") == 0)) {
    flash = false;
    colorfade = true;
    currentColor = 0;
    if (strcmp(root["effect"], "colorfade_slow") == 0) {
      transitionTime = CONFIG_COLORFADE_TIME_SLOW;
    }
    else {
      transitionTime = CONFIG_COLORFADE_TIME_FAST;
    }
  }
  else if (colorfade && !root.containsKey("color") && root.containsKey("brightness")) {
    // Adjust brightness during colorfade
    // (will be applied when fading to the next color)
    brightness = root["brightness"];
  }
  // super_white effect turns on all 4 LEDs, if present
  else if (root.containsKey("effect") && strcmp(root["effect"], "super_white") == 0 ){
    flash = false;
    colorfade = false;

    if (rgb) {
      red = 255;
      green = 255;
      blue = 255;
    }
    
    if (includeWhite) {
      white = 255;
    }

    brightness = 255;

    color_temp = 0;

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else {
      transitionTime = 0;
    }
    
  }
  else { // No effect
    flash = false;
    colorfade = false;

    if (rgb && root.containsKey("color")) {
      red = root["color"]["r"];
      green = root["color"]["g"];
      blue = root["color"]["b"];
    } else {
      red = 0;
      green = 0;
      blue = 0;
    }

    if (includeWhite && root.containsKey("white_value")) {
      white = root["white_value"];
    } else {
      white = 0;
    }

    if (root.containsKey("brightness")) {
      brightness = root["brightness"];
    } 

    if (root.containsKey("color_temp")) {
      color_temp = root["color_temp"];
    } else {
      color_temp = 0;
    }

    if (root.containsKey("transition")) {
      transitionTime = root["transition"];
    }
    else {
      transitionTime = 0;
    }

    Serial.print("last_stateOn ");
    Serial.println(last_stateOn);

    if(stateOn && last_stateOn == false){
      get_last_state();
      last_stateOn = stateOn;
    }

    // save last state
    if(stateOn){
      last_red = red;
      last_green = green;
      last_blue = blue;
      last_white = white;
      last_brightness = brightness;
      last_color_temp = color_temp;
    }
  }

  return true;
}

void get_last_state(){

  Serial.println("setting, last, current");
  
  Serial.print("red        ");
  Serial.print(last_red);
  Serial.print(", ");
  Serial.println(red);
  
  Serial.print("green      ");
  Serial.print(last_green);
  Serial.print(", ");
  Serial.println(green);
  
  Serial.print("blue       ");
  Serial.print(last_blue);
  Serial.print(", ");
  Serial.println(blue);
  
  Serial.print("white      ");
  Serial.print(last_white);
  Serial.print(", ");
  Serial.println(white);
  
  Serial.print("color temp ");
  Serial.print(last_color_temp);
  Serial.print(", ");
  Serial.println(color_temp);
  
  Serial.print("brightness ");
  Serial.print(last_brightness);
  Serial.print(", ");
  Serial.println(brightness);  
  
  if(rgb && red==0 && green==0 && blue==0 && color_temp == 0){
    red = last_red;
    green = last_green;
    blue = last_blue;
    color_temp = last_color_temp;
  }

  if(includeWhite && white==0){
    white = last_white;
  }

  if(brightness == 0){
    brightness = last_brightness;
  }
}

void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["state"] = (stateOn) ? CONFIG_MQTT_PAYLOAD_ON : CONFIG_MQTT_PAYLOAD_OFF;
  if (rgb) {
    JsonObject& color = root.createNestedObject("color");
    color["r"] = red;
    color["g"] = green;
    color["b"] = blue;
  }

  root["brightness"] = brightness;
  root["color_temp"] = color_temp;

  if (includeWhite) {
    root["white_value"] = white;
  }

  if (rgb && colorfade) {
    if (transitionTime == CONFIG_COLORFADE_TIME_SLOW) {
      root["effect"] = "colorfade_slow";
    }
    else {
      root["effect"] = "colorfade_fast";
    }
  }
  else {
    root["effect"] = "null";
  }

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));
  
  // Check for MQTT connection and reconnect if necessary
  if (!client.connected()) {
    reconnect();
  }

  client.publish(CONFIG_MQTT_TOPIC_STATE, buffer, true);
}

void setColor(int inR, int inG, int inB, int inW) {

  if((rgb && inR + inG + inB > 0) || (includeWhite && inW > 0)){
    // Update builtin LED
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_ON);
  } else {
    // Update builtin LED
    digitalWrite(BUILTIN_LED, CONFIG_BUILTIN_LED_OFF);    
  }

  /* Simple color correction */
  inR = inR * CONFIG_RED_ADJUST/255;
  inG = inG * CONFIG_GREEN_ADJUST/255;
  inB = inB * CONFIG_BLUE_ADJUST/255;
  
  if (CONFIG_INVERT_LED_LOGIC) {
    inR = (255 - inR);
    inG = (255 - inG);
    inB = (255 - inB);
    inW = (255 - inW);
  }

  if (rgb) {
    analogWrite(CONFIG_PIN_RED, inR);
    analogWrite(CONFIG_PIN_GREEN, inG);
    analogWrite(CONFIG_PIN_BLUE, inB);
  }

  if (includeWhite) {
    analogWrite(CONFIG_PIN_WHITE, inW);
  }

  if (CONFIG_DEBUG) {
    Serial.print("Setting LEDs: {");
    if (rgb) {
      Serial.print("r: ");
      Serial.print(inR);
      Serial.print(" , g: ");
      Serial.print(inG);
      Serial.print(" , b: ");
      Serial.print(inB);
    }

    if (includeWhite) {
      if (rgb) {
        Serial.print(", ");
      }
      Serial.print("w: ");
      Serial.print(inW);
    }

    Serial.println("}");
  }
}

void handle_led_functions() {

  if (flash) {
    if (startFlash) {
      startFlash = false;
      flashStartTime = millis();
    }

    if ((millis() - flashStartTime) <= flashLength) {
      if ((millis() - flashStartTime) % 1000 <= 500) {
        setColor(flashRed, flashGreen, flashBlue, flashWhite);
      }
      else {
        setColor(0, 0, 0, 0);
        // If you'd prefer the flashing to happen "on top of"
        // the current color, uncomment the next line.
        // setColor(realRed, realGreen, realBlue, realWhite);
      }
    }
    else {
      flash = false;
      setColor(realRed, realGreen, realBlue, realWhite);
    }
  }
  else if (rgb && colorfade && !inFade) {
    realRed = map(colors[currentColor][0], 0, 255, 0, brightness);
    realGreen = map(colors[currentColor][1], 0, 255, 0, brightness);
    realBlue = map(colors[currentColor][2], 0, 255, 0, brightness);
    realWhite = map(colors[currentColor][3], 0, 255, 0, brightness);
    currentColor = (currentColor + 1) % numColors;
    startFade = true;
  }

  if (startFade) {
    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      setColor(realRed, realGreen, realBlue, realWhite);

      redVal = realRed;
      grnVal = realGreen;
      bluVal = realBlue;
      whtVal = realWhite;

      startFade = false;
    }
    else {
      loopCount = 0;
      stepR = calculateStep(redVal, realRed);
      stepG = calculateStep(grnVal, realGreen);
      stepB = calculateStep(bluVal, realBlue);
      stepW = calculateStep(whtVal, realWhite);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);
        whtVal = calculateVal(stepW, whtVal, loopCount);

        setColor(redVal, grnVal, bluVal, whtVal); // Write current values to LED pins

        Serial.print("Loop count: ");
        Serial.println(loopCount);
        loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }
}

// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS
*
* The program works like this:
* Imagine a crossfade that moves the red LED from 0-10,
*   the green from 0-5, and the blue from 10 to 7, in
*   ten steps.
*   We'd want to count the 10 steps and increase or
*   decrease color values in evenly stepped increments.
*   Imagine a + indicates raising a value by 1, and a -
*   equals lowering it. Our 10 step fade would look like:
*
*   1 2 3 4 5 6 7 8 9 10
* R + + + + + + + + + +
* G   +   +   +   +   +
* B     -     -     -
*
* The red rises from 0 to 10 in ten steps, the green from
* 0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.
*
* In the real program, the color percentages are converted to
* 0-255 values, and there are 1020 steps (255*4).
*
* To figure out how big a step there should be between one up- or
* down-tick of one of the LED values, we call calculateStep(),
* which calculates the absolute gap between the start and end values,
* and then divides that gap by 1020 to determine the size of the step
* between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
    int step = endValue - prevValue; // What's the overall gap?
    if (step) {                      // If its non-zero,
        step = 1020/step;            //   divide by 1020
    }

    return step;
}

/* The next function is calculateVal. When the loop value, i,
*  reaches the step size appropriate for one of the
*  colors, it increases or decreases the value of that color by 1.
*  (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
    if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
        if (step > 0) {              //   increment the value if step is positive...
            val += 1;
        }
        else if (step < 0) {         //   ...or decrement it if step is negative
            val -= 1;
        }
    }

    // Defensive driving: make sure val stays in the range 0-255
    if (val > 255) {
        val = 255;
    }
    else if (val < 0) {
        val = 0;
    }

    return val;
}

/*  Compute RGB from Kelvin or Mireds
 *   
 *   Thanks to Tanner Heland  
 *   http://www.tannerhelland.com/4435/convert-temperature-rgb-algorithm-code/
 *   
 */

int kelvin_to_red(int kelvin){
  int redw = 0;
  int work_kelvin = kelvin / 100;

  if(work_kelvin <= 66){
    redw = 255;
  } else {
    redw = 329.698727446 * pow(static_cast<double>(work_kelvin - 60) , static_cast<double>(-0.1332047592));
    if(redw < 0){ redw = 0;}
    if(redw > 255){ redw = 255;}
  }

  return redw;  
}

int mired_to_red(int mired){
  /* 1000000 / mired = kelvin */
  return kelvin_to_red(1000000/mired);
}

int kelvin_to_green(int kelvin){
  int greenw = 0;
  int work_kelvin = kelvin / 100;

  if(work_kelvin <=66){
    greenw = 99.4708025861 * log(static_cast<double>(work_kelvin)) - 161.1195681661;
  } else {
    greenw = 288.1221695283 * pow(static_cast<double>(work_kelvin - 60), static_cast<double>(-0.0755148492));
  }
  
  if(greenw < 0){ greenw = 0;}
  if(greenw > 255){ greenw = 255;}

  return greenw;
}

int mired_to_green(int mired){
  /* 1000000 / mired = kelvin */
  return kelvin_to_green(1000000/mired);
}

int kelvin_to_blue(int kelvin){
  int bluew = 0;
  int work_kelvin = kelvin / 100;

  if(work_kelvin >= 66){bluew = 255;}
  else if(work_kelvin <=19){bluew = 0;}
  else {
    bluew = 138.5177312231 * log(static_cast<double>(work_kelvin - 10)) - 305.0447927307;
    if(bluew < 0){ bluew = 0;}
    if(bluew > 255){ bluew = 255;}
  }

  return bluew;
}

int mired_to_blue(int mired){
  /* 1000000 / mired = kelvin */
  return kelvin_to_blue(1000000/mired);
}

/* End color temp */

void handlePushButton() {

  if (digitalRead(CONFIG_PUSHBUTTON_PIN) == HIGH) {

    if (buttonActive == false) {  // A press has not been started
      buttonActive = true;
      buttonTimer = millis();  // start the timer
    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {

      longPressActive = true;
      // Do the long press stuff
      Serial.println("long press");
      strip_test();
      next_preset(); 
    }

  } else {

    if (buttonActive == true) {    // a press was started
      if (longPressActive == true) {
        longPressActive = false;
      } else {
        if(millis() - buttonTimer > debounceTime) {
          // Do the short press stuff
          Serial.println("short press");
          next_preset();
        }
      }
      buttonActive = false; 
    }
  }
}

void next_preset(){
  if(currentPreset  >= numPresets){
    currentPreset = 0;
  }

  Serial.print("This preset is ");
  Serial.println(currentPreset);

  red = presets[currentPreset][0];
  green = presets[currentPreset][1];
  blue = presets[currentPreset][2];
  white = presets[currentPreset][3];
  brightness = presets[currentPreset][4];

  if(brightness > 0){
    stateOn = true;
  } else {
    stateOn = false;
  }

  realRed = map(red, 0, 255, 0, brightness);
  realGreen = map(green, 0, 255, 0, brightness);
  realBlue = map(blue, 0, 255, 0, brightness);
  realWhite = map(white, 0, 255, 0, brightness);

  setColor(realRed, realGreen, realBlue, realWhite);

  colorfade = false;

  sendState();

  currentPreset++;
  
}

void handle_dht(){
 
  long now = millis();

  if(now < lastTempReport) lastTempReport = now;

  if ((now - lastTempReport) > 600000 || lastTempReport == 0){  // Report first time then every 10 minutes
    lastTempReport = now;

    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();
  
    if (isnan(h) || isnan(t)) {
      Serial.println("ERROR: Failed to read from DHT sensor!");
      return;
    } else {
      publishData(t, h);
    }
  }

}

// function called to publish the temperature and the humidity
void publishData(float p_temperature, float p_humidity) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)p_temperature;
  root["humidity"] = (String)p_humidity;
  root.prettyPrintTo(Serial);
  Serial.println("");
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(CONFIG_MQTT_SENSOR_TOPIC, data, true);
}

void reconnect() {
   if(last_reconnect > millis()){
    last_reconnect = millis();
   }

  if(millis() - last_reconnect > reconnect_wait){
    last_reconnect = millis();
    //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
    wifiManager.autoConnect(CONFIG_OTA_HOST);
    Serial.print("Attempting MQTT connection...");
    if (client.connect(CONFIG_MQTT_HOST, CONFIG_MQTT_USER, CONFIG_MQTT_PASS)) {
      Serial.println("connected");
      client.subscribe(CONFIG_MQTT_TOPIC_SET);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      //delay(5000);
    }
  }
}
