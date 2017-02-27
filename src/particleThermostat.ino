/*
 * Project particleThermostat
 * Description: A cloud-connected thermostat
 * Author: Matt Michini
 * Date: Feb 2017
 */

// Photon system mode:
 SYSTEM_MODE(SEMI_AUTOMATIC)

// Pin mappings:
#define PIN_MODE_BTN        A1
#define PIN_TEMP_UP_BTN     D1
#define PIN_TEMP_DOWN_BTN   D2
#define PIN_FAN_BTN         DAC

#define PIN_MAIN_TEMP_SENSOR  A3
#define PIN_AUX_TEMP_SENSOR   A0
#define PIN_LIGHT_SENSOR      A5

#define PIN_MASTER_RELAY    D3
#define PIN_HEAT_RELAY      D4
#define PIN_COOL_RELAY      D5
#define PIN_FAN_RELAY       D6

#define PIN_BLUE_LED   D7

// Heating mode constants:
#define TEMP_BAND_DEG_F       1     // hysteresis band for set point temperature
#define HEAT_REQUEST_DELAY_MS 5000  // delay between when heat is requested and furnace is turned on.
#define MIN_ON_TIME_MS        20000 // minimum time to keep heater on
#define MIN_OFF_TIME_MS       20000 // minimum time heater must remain off between cycles.

// timing:
unsigned long lastHeatRequestTime_ms = 0;
unsigned long lastReadSensorsTime_ms = 0;
unsigned long lastFurnaceOnTime_ms = 0;
unsigned long lastFurnaceOffTime_ms = 0;

// Heating mode variables:
bool heatMode = false;
bool heaterRequest = false;
bool furnaceOn = false;
int setPoint_degF = 70;

// temperature readings:
int mainTemp_mDegC = 0;
int mainTemp_degF = 0;
int auxTemp_mDegC = 0;
int auxTemp_degF = 0;


void setup() {
  // Set pin directions:
  // Button inputs:
  pinMode(PIN_MODE_BTN, INPUT_PULLDOWN);
  pinMode(PIN_TEMP_UP_BTN, INPUT_PULLDOWN);
  pinMode(PIN_TEMP_DOWN_BTN, INPUT_PULLDOWN);
  // Relay outputs:
  pinMode(PIN_MASTER_RELAY, OUTPUT);
  pinMode(PIN_HEAT_RELAY, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  // Temperature sensors:
  pinMode(PIN_MAIN_TEMP_SENSOR, INPUT);
  pinMode(PIN_AUX_TEMP_SENSOR, INPUT);

  // Pin change interrupts for the switches:
  attachInterrupt(PIN_MODE_BTN, ModeButton, RISING);
  attachInterrupt(PIN_TEMP_UP_BTN, TempUpButton, RISING);
  attachInterrupt(PIN_TEMP_DOWN_BTN, TempDownButton, RISING);

  // Serial port setup for LCD display:
  Serial1.begin(9600);
  delay(500);
  // Set backlight brightness (128 - 157)
  Serial1.write(124);
  Serial1.write(157);
  // Clear display:
  Serial1.write(254);
  Serial1.write(1);
  delay(500);

  // set up cloud functions:
  Particle.function("setTemp", SetTemperature);

  // master switch on:
  digitalWrite(PIN_MASTER_RELAY, HIGH);
}


void loop() {
  // connect to the cloud if not already:
  if (!Particle.connected())
    Particle.connect();

  // Read the sensors:
  if (millis() - lastReadSensorsTime_ms > 5000)
  {
    ReadSensors();
    UpdateDisplay();
  }

  // Heater control:
  if (heatMode)
  {
    if (!furnaceOn &&
      mainTemp_degF <= setPoint_degF - TEMP_BAND_DEG_F)
    {
      // should turn heat on
      if (!heaterRequest)
      {
        heaterRequest = true;
        lastHeatRequestTime_ms = millis();
      }
      else if (millis() - lastHeatRequestTime_ms > HEAT_REQUEST_DELAY_MS)
      {
        // We've waited required request delay, now command the furnace
        //  to turn on, but only if we've waited the minimum OFF time
        //  between cycles.
        furnaceOn = (millis() - lastFurnaceOnTime_ms > MIN_OFF_TIME_MS);
      }
    }
    else if (mainTemp_degF >= setPoint_degF + TEMP_BAND_DEG_F)
    {
      if (millis() - lastFurnaceOffTime_ms > MIN_ON_TIME_MS)
        furnaceOn = false;

    }
  }
  else // heatMode == false
  {
    if (millis() - lastFurnaceOffTime_ms > MIN_ON_TIME_MS)
      furnaceOn = false;
  }

  // safety check:
  if (mainTemp_degF > 85 || auxTemp_degF > 90)
  {
    // TO DO: Throw the master switch
    digitalWrite(PIN_MASTER_RELAY, LOW);
    furnaceOn = false;
  }


  // Control furnace relay:
  if (furnaceOn)
  {
    // turn on the furnace relay
    digitalWrite(PIN_HEAT_RELAY, HIGH);
    lastFurnaceOnTime_ms = millis();
  }
  else
  {
    digitalWrite(PIN_HEAT_RELAY, LOW);
    lastFurnaceOffTime_ms = millis();
  }
}


void ReadSensors(void)
{
  lastReadSensorsTime_ms = millis();
  digitalWrite(PIN_BLUE_LED, HIGH);

  // analogRead returns 0-4095 for 0.0-3.3 volts.
  const int mainTemp_mv = analogRead(PIN_MAIN_TEMP_SENSOR) * 3300 / 4096;
  const int auxTemp_mv = analogRead(PIN_AUX_TEMP_SENSOR) * 3300 / 4096;
  const int lightSensor_mv = analogRead(PIN_LIGHT_SENSOR) * 3300 / 4096;

  // compute temperature in thousandths of degC
  mainTemp_mDegC = 25000 + 100*(mainTemp_mv - 750);
  mainTemp_degF = 9*mainTemp_mDegC/(5*1000) + 32;

  auxTemp_mDegC = 25000 + 100*(auxTemp_mv - 750);
  auxTemp_degF = 9*auxTemp_mDegC/(5*1000) + 32;

  // light sensor resistance range = 1k~10kohm, and is in series with a 2.2 kohm resistor.
  //lightSensor = lightSensorVolts;
  //lightSensor = (lightSensorVolts - 0.595F) / (2.269F - 0.595F);
}


void UpdateDisplay(void)
{
  char line1[17] = {0};
  char line2[17] = {0};

  // Compose the first line:
  sprintf(line1, "TEMP: %d SET: %d", mainTemp_degF, setPoint_degF);

  // Compose the second line:
  sprintf(line2, "%s", heatMode ? "HEAT " : "OFF  ");

  // Send the new lines to the LCD display:
  Serial1.write(254); // cursor to beginning of first line
  Serial1.write(128);
  Serial1.write(line1);

  Serial1.write(254); // cursor to beginning of second line
  Serial1.write(192);
  Serial1.write(line2);
}


int SetTemperature(String command)
{
  int newSetpoint = command.toInt();
  if (newSetpoint == 0)
    return -1;

  if (newSetpoint > 85 || newSetpoint < 50)
    return -1;

  // fire up the heat:
  heatMode = true;
  setPoint_degF = newSetpoint;

  return 1;
}


void ModeButton(void)
{
  static unsigned long timeOfLastInt = 0;
  if (millis() - timeOfLastInt > 250)
  {
    heatMode = !heatMode;
    UpdateDisplay();
  }
  timeOfLastInt = millis();
}


void TempUpButton(void)
{
  static unsigned long timeOfLastInt = 0;
  if (millis() - timeOfLastInt > 250)
  {
    if (setPoint_degF < 90)
        ++setPoint_degF;

    UpdateDisplay();
  }
  timeOfLastInt = millis();
}


void TempDownButton(void)
{
  static unsigned long timeOfLastInt = 0;
  if (millis() - timeOfLastInt > 250)
  {
    if (setPoint_degF > 50)
        --setPoint_degF;

    UpdateDisplay();
  }
  timeOfLastInt = millis();
}
