#include <Homie.h>
// #include <DallasTemperature.h>
// #define ONE_WIRE_BUS D5

// OneWire oneWire(ONE_WIRE_BUS);
// DallasTemperature sensors(&oneWire);
// String ProbeName[5];
// String ProbeAddress;
// int sensor_count;   // used in the Dallas device dicsovery
// float tempC = 0;  // temperature
// unsigned long lastTemperatureSent = 0;
const int STOP_SWITCH = D4;
const int LIMIT_SWITCH = D0;
const int AUTO_MODE = D2;
const int ACTUATE_OPEN = D1;
const int MOTORA = D6;
const int MOTORB = D7;
const int AUTO_STAT_LED = D8;
const int RAIN_SENSE = D5;
const int LIGHT_SENSE = A0;
const int DEFAULT_LIGHT_LIMIT = 80;
bool AUTO_MODE_ENABLE = 0;

bool openState = 0; // 0 = closed 1 = open
bool timerStat = 0;
int lightRead;
Bounce autoDebounce = Bounce(); // Bounce is built into Homie, so you can use it without including it first
Bounce actuateDebounce = Bounce();
Bounce stopDebounce = Bounce();
Bounce limitDebounce = Bounce();
// Bounce rainDebounce = Bounce();
int lastModeValue = -1;
int lastActuatorValue = -1;
int lastLimitValue = -1;
int lastStopValue = -1;
bool setAutoMode = true;
bool setRainMode = false;
const int OFF_INTERVAL = 10;


unsigned long actuatorTimer = millis();
unsigned long sensorTimer = millis();
unsigned long closeOnRainTimer = millis();

HomieNode ActuatorSwitch("actuatorswitch", "switch");
HomieNode AutoMode("automode", "switch");
HomieNode RainSensor("rainsensor", "sensor");
HomieNode LightSensor("lightsensor", "sensor");
// HomieNode temperatureNode("temperature", "temperature");
HomieSetting<long> lightLimitSetting("lightLimit", "Set the light value to open close");
HomieSetting<bool> autoModeEnable("autoModeEnable", "Persistently set if an automated device");
HomieSetting<bool> RainSensorMode("rainModeEnable", "Persistently set rain sensor automation");

// HomieSetting<long> Rainsensor("testsensornode", "set Persistently");
// ############### Function to set up temp sensors #############################
/* String deviceToStr(const uint8_t* Probe01)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(Probe01[i], 16);
  }
  return result;
}
// ############# Homie Setup Loop ########################
void setupHandler() {
  temperatureNode.setProperty("unit").send("c");
} */

// ############### Function to open close and set open/close state #############
float control_actuate(String state, bool motora, bool motorb, bool openStateIn){
  digitalWrite(MOTORA, motora);
  digitalWrite(MOTORB, motorb);

  timerStat = 1;
  ActuatorSwitch.setProperty("state").send(state);
  Homie.getLogger() << "Actuator is " << (state) << endl;
  openState = openStateIn;
  unsigned long resetTimer = millis();
  return resetTimer;
}
// ########### Homie Main Loop Handler ##################
void loopHandler() {
  int modeValue = autoDebounce.read();
  bool modeState = autoDebounce.fell();
  int actuatorValue = actuateDebounce.read();
  bool actuatorState = actuateDebounce.fell();
//  int stopValue = stopDebounce.read();
  bool stopState = stopDebounce.fell();
//  int limitValue = limitDebounce.read();
// use fell when limit switch is NO when in closed position
//  bool limitState = limitDebounce.fell();
// use rise when limit switch is NC when in closed position
  bool limitState = limitDebounce.rose();
//  int rainValue = rainDebounce.read();
//  bool rainState = rainDebounce.fell();

  if ((limitState == 1) && (setAutoMode == true)) {
    actuatorTimer = control_actuate("limit",0,0,openState);
  }
  if (stopState == 1) {
    actuatorTimer = control_actuate("stop",0,0,openState);
  }
  if ((actuatorState == 1) && (setAutoMode == 0)) {
     String value = "na";
     Homie.getLogger() << "Actuator state " << (actuatorValue ? "open" : "close") << endl;
     lastActuatorValue = actuatorValue;
     Serial.println("stuck in tv switch loop");
     if (openState == 0) {
//             bool state = (value == "OPEN");
             value = "OPEN";
             Serial.println("Entering open loop");
             actuatorTimer = control_actuate("OPEN",1,0,1);
     }
     else if (openState == 1) {
//             bool state = (value == "CLOSE");
             value = "CLOSE";
             Serial.println("Entering close loop");
             actuatorTimer = control_actuate("CLOSE",0,1,0);

     }
  }
  // ########## Toggle Automatic mode on/off #################
  if ((modeState == 1) && (AUTO_MODE_ENABLE == 1)) {
    lastModeValue = modeValue;
    setAutoMode = !setAutoMode;
    if (setAutoMode == 0) {
      setAutoMode = false;
      digitalWrite(AUTO_STAT_LED, setAutoMode);
    }
    else if (setAutoMode == 1) {
      setAutoMode = true;
      digitalWrite(AUTO_STAT_LED, setAutoMode);
    }
    Homie.getLogger() << "Auto Mode " << (setAutoMode ? "true" : "false") << endl;
    AutoMode.setProperty("state").send(setAutoMode ? "true" : "false");
  }

// ################# Read Light Sensor every 20 secs ########################
         if (millis() - sensorTimer >= 20000UL)  {
           int readLightLimit;
           readLightLimit = lightLimitSetting.get();
           bool readRainState;
           readRainState = digitalRead(RAIN_SENSE);

           String readLightLimitStr = String(readLightLimit);
           lightRead = analogRead(A0);

           int lightValue;
           lightValue = map(lightRead,0,1023,0,100);
           String lightValueStr = String(lightValue);
           String staticAutoMode = String(AUTO_MODE_ENABLE);
           LightSensor.setProperty("limit").send(readLightLimitStr);
           LightSensor.setProperty("value").send(lightValueStr);
           LightSensor.setProperty("staticautomode").send(staticAutoMode);

        if ((setAutoMode == 1) && (AUTO_MODE_ENABLE == 1)) {
           if ((lightValue <= (readLightLimit - 5)) && (openState == 0) && (readRainState == 1) && (millis() - closeOnRainTimer >= 120000UL)) {
             actuatorTimer = control_actuate("openonlight",1,0,1);
           }
           if ((lightValue > readLightLimit) && (openState == 1)) {
             actuatorTimer = control_actuate("closeonlight",0,1,0);
           }
         }
         else if (setAutoMode == 0) {
           Serial.println("auto mode is off");
         }
         if ((readRainState == 0) && (openState == 1)) {
           Serial.println("It's raining");
           actuatorTimer = control_actuate("closeonrain",0,1,0);
           closeOnRainTimer = millis();
         }
//           actuatorTimer = millis();
           sensorTimer = millis();
         }
// ################ Check timer and turn off if expired ###################
        if ((millis() - actuatorTimer >= OFF_INTERVAL * 2000UL) && (timerStat == 1)) {
            Serial.println("Turning off power");
            actuatorTimer = control_actuate("STOP",0,0,openState);
            timerStat = 0;
                  }

}

// ############## Handler for the Actuator MQTT callback ########
bool ActuatorHandler(const HomieRange& range, const String& value) {
        if (value != "OPEN" && value != "CLOSE" && value != "STOP") return false;
if (value == "OPEN") {
        Serial.println("Actuator open");
        control_actuate(value,1,0,1);
}
if (value == "CLOSE") {
        Serial.println("Actuator Close");
        control_actuate(value,0,1,0);
}
if (value == "STOP") {
        Serial.println("Actuator Stop");
        control_actuate(value,0,0,0);
}
//        openState = !openState;
        return true;
}
// ############## Handler for Auto Mode MQTT callback ########
bool ModeHandler(const HomieRange& range, const String& value) {
        if ((value != "true" && value != "false") && (AUTO_MODE_ENABLE == 0)) return false;
      if (value == "true") {
        setAutoMode = true;
        digitalWrite(AUTO_STAT_LED, HIGH);
      }
      else {
        setAutoMode = false;
        digitalWrite(AUTO_STAT_LED, LOW);

      }
      AutoMode.setProperty("state").send(setAutoMode ? "true" : "false");
}
// ############## Handler for the Rain MQTT callback ########
bool RainHandler(const HomieRange& range, const String& value) {
    if (value != "true" && value != "false") return false;
    if (value == "true") {
      setRainMode = true;
      control_actuate("raining",0,1,0);
    }
    else {
      setRainMode = false;
      control_actuate("rainstop",1,0,1);
    }
    RainSensor.setProperty("state").send(setAutoMode ? "true" : "false");
    openState = 0;

}
void setup() {
  Serial.begin(115200);
  Serial << endl << endl;
  pinMode(STOP_SWITCH, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(AUTO_MODE, INPUT_PULLUP);
  pinMode(ACTUATE_OPEN, INPUT_PULLUP);
  pinMode(MOTORA, OUTPUT);
  pinMode(LIGHT_SENSE, INPUT);
  digitalWrite(MOTORA, LOW);
  pinMode(MOTORB, OUTPUT);
  digitalWrite(MOTORB, LOW);
  pinMode(AUTO_STAT_LED, OUTPUT);
  pinMode(RAIN_SENSE, INPUT_PULLUP);
  autoDebounce.attach(AUTO_MODE);
  autoDebounce.interval(50);
  actuateDebounce.attach(ACTUATE_OPEN);
  actuateDebounce.interval(10);
  stopDebounce.attach(STOP_SWITCH);
  stopDebounce.interval(50);
  limitDebounce.attach(LIMIT_SWITCH);
  limitDebounce.interval(50);
//  rainDebounce.attach(RAIN_SENSE);
//  rainDebounce.interval(50);
  // sensors.begin();
  // sensor_count = sensors.getDeviceCount();


  Homie_setFirmware("actuator_controller", "1.0.1");
  // Homie.setSetupFunction(setupHandler).setLoopFunction(loopHandler);
  Homie.setLoopFunction(loopHandler);
  ActuatorSwitch.advertise("state").settable(ActuatorHandler);
  AutoMode.advertise("state").settable(ModeHandler);
  RainSensor.advertise("state").settable(RainHandler);
  lightLimitSetting.setDefaultValue(DEFAULT_LIGHT_LIMIT).setValidator([] (long candidate) {
  return (candidate >= 0) && (candidate <= 100);
  });
  autoModeEnable.setDefaultValue(AUTO_MODE_ENABLE);
  Homie.setup();
  AUTO_MODE_ENABLE = autoModeEnable.get();
  if (AUTO_MODE_ENABLE == 1) {
    setAutoMode = 1;
    digitalWrite(AUTO_STAT_LED, HIGH);
    }
    else if (AUTO_MODE_ENABLE == 0) {
      setAutoMode = 0;
      digitalWrite(AUTO_STAT_LED, LOW);
    }

  /* int c = 0;
       for(c=0; c<sensor_count; c++) {
          DeviceAddress tempAddress;
          sensors.getAddress(tempAddress, c);
          ProbeName[c] = deviceToStr(tempAddress);


         temperatureNode.advertise(ProbeName[c].c_str());
       }
       temperatureNode.advertise("unit");
       temperatureNode.advertise("degrees"); */
}

void loop() {
  Homie.loop();
  autoDebounce.update();
  actuateDebounce.update();
  stopDebounce.update();
  limitDebounce.update();
//  rainDebounce.update();
}
