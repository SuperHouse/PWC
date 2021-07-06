/**
   Permobil Wheelchair Controller

   Simulates a wheelchair joystick using I2C digital potentiometers,
   so that a Permobil wheelchair can be controlled electronically.

   The term "position" is used through the code to signify the
   virtual position of the joystick, even though it doesn't physically
   exist. The virtual position of the joystick is then mapped to
   the outputs that connect to the original wheelchair controller.

   Position is measured as % deflection from the zero (central)
   position, ie: from -100% to +100%. This is equivalent to full
   effort by the user in each direction.

                     Y axis
                     +100%
                       ^
                       |
    X axis   -100% <-- 0 --> +100%
                       |
                       v
                     -100%

   External dependencies. Install using Arduino library manager:
     "SparkFun Qwiic Scale NAU7802" by SparkFun Electronics
     "PS3 Controller Host" by Jeffrey van Pernis https://github.com/jvpernis/esp32-ps3

   External dependencies. Install by downloading the ZIP file:
     "PS4-esp32" by aed3 https://github.com/aed3/PS4-esp32
     It may need to be this specific version, if the latest version
     above fails to read values from the controller:
     https://github.com/aed3/PS4-esp32/tree/d23311c8489f2538c8e49d6e39dc33b19e0edbcb

   More information:
     www.superhouse.tv/pwc

   To do:
    - PS4 support is untested. I can't compile sixaxispairtool on my Mac at
      the moment.
    - Zero offsets are currently hard-coded. These should be set at startup
      and when tare is run.
    - Do we need setZeroOffset() after calculateZeroOffset() in setup?

   ESP32 pin assignments:
     D0:
     D1:
     D2:
     D3:
     D4:  SDA
     D5:  SCL
     D6:  Tare pin
     D7:  Disable pin
     D8:
     D9:
     D10:

   By:
    Chris Fryer <chris.fryer78@gmail.com>
    Jonathan Oxer <jon@oxer.com.au>

   Copyright 2021 SuperHouse Automation Pty Ltd www.superhouse.tv
*/
#define VERSION "2.1"
/*--------------------------- Configuration ---------------------------------*/
// Configuration should be done in the included file:
#include "config.h"

/*--------------------------- Libraries -------------------------------------*/
#include <Wire.h>
#include <RunningMedian.h>          // By Rob Tillaart
#if ENABLE_ZEROSTICK
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Load cell amplifier
#endif
#if ENABLE_PS3_CONTROLLER
#include <Ps3Controller.h>          // By Jeffrey van Pernis
#endif
#if ENABLE_PS4_CONTROLLER
#include <PS4Controller.h>          // By aed3
#endif

#if ENABLE_DIGIPOT_OUTPUT
#include <Adafruit_DS3502.h>        // Digital potentiometer
#endif

/*--------------------------- Global Variables ------------------------------*/
int16_t  g_zero_tare_offset_x    = -770;   // X axis tare correction
int16_t  g_zero_tare_offset_y    = -320;   // Y axis tare correction

int32_t  g_input_x_position      = 0;   // Most recent force reading from X axis (+/- %)
int32_t  g_input_y_position      = 0;   // Most recent force reading from Y axis (+/- %)

uint8_t  g_ps_controller_enabled = false;

uint32_t g_last_digipot_time     = 0;   // When we last updated the digipot outputs

int32_t  g_x_zero_offset         = 0;
int32_t  g_y_zero_offset         = 0;
uint8_t  g_next_channel_to_read  = 0;   // 0 = X axis, 1 = Y axis
uint8_t  g_channel_read_count    = 0;   // Accumulate how many times channel has been read
int32_t  g_sensor_raw_value_sum  = 0;   // Accumulate channel readings


/*--------------------------- Function Signatures ---------------------------*/


/*--------------------------- Instantiate Global Objects --------------------*/
// Load cells
#if ENABLE_ZEROSTICK
NAU7802 loadcells;
#endif

RunningMedian x_samples = RunningMedian(5);
RunningMedian y_samples = RunningMedian(5);

#if ENABLE_DIGIPOT_OUTPUT
Adafruit_DS3502 digipot_x = Adafruit_DS3502();
Adafruit_DS3502 digipot_y = Adafruit_DS3502();
#endif

/*--------------------------- Program ---------------------------------------*/
/**
  Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  Serial.println();
  Serial.print("Permobil Wheelchair Controller starting up, v");
  Serial.println(VERSION);

#ifdef ARDUINO_ARCH_ESP32
  Serial.println("Architecture detected: ESP32");
#endif

  Wire.begin();

  pinMode(DISABLE_PIN,      INPUT_PULLUP);
  pinMode(TARE_BUTTON_PIN,  INPUT_PULLUP);

#if ENABLE_DIGIPOT_OUTPUT
  Serial.println("Searching for output digipots:");
  if (!digipot_x.begin(DIGIPOT_X_I2C_ADDR))
  {
    Serial.println("Couldn't find X-axis DS3502 chip");
  }
  Serial.println("Found X-axis DS3502 chip");

  if (!digipot_y.begin(DIGIPOT_Y_I2C_ADDR))
  {
    Serial.println("Couldn't find Y-axis DS3502 chip");
  }
  Serial.println("Found Y-axis DS3502 chip");

  digipot_x.setWiper(DIGIPOT_CENTER);        // Start with pots in central position
  digipot_y.setWiper(DIGIPOT_CENTER);        // Start with pots in central position
#endif

#if ENABLE_ZEROSTICK
  // Start the load cell interface
#if ENABLE_ZEROSTICK_DEBUGGING
  Serial.print("Starting load cell interface: ");
#endif ENABLE_ZEROSTICK_DEBUGGING
  if (loadcells.begin() == false)
  {
#if ENABLE_ZEROSTICK_DEBUGGING
    Serial.println("Not detected.");
#endif ENABLE_ZEROSTICK_DEBUGGING
    //while (1);
  }
#if ENABLE_ZEROSTICK_DEBUGGING
  Serial.println("ok.");
#endif ENABLE_ZEROSTICK_DEBUGGING
  loadcells.setGain(NAU7802_GAIN_1);        // Gain can be set to 1, 2, 4, 8, 16, 32, 64, or 128.
  loadcells.setSampleRate(NAU7802_SPS_320); // Sample rate can be set to 10, 20, 40, 80, or 320Hz
  delay(100);
  tareCellReadings();
#endif ENABLE_ZEROSTICK

#if ENABLE_PS3_CONTROLLER
  //Ps3.attach(notify);
  Ps3.attachOnConnect(ps3OnConnect);
  Ps3.begin(BLE_MAC);
#endif ENABLE_PS3_CONTROLLER

#if ENABLE_PS4_CONTROLLER
  PS4.begin(BLE_MAC);
#endif ENABLE_PS4_CONTROLLER
}


/**
  Loop
*/
void loop()
{
  if (false == g_ps_controller_enabled)
  {
#if ENABLE_ZEROSTICK
    checkTareButton();
    readZerostickInputPosition();
#endif ENABLE_ZEROSTICK
  }

#if ENABLE_PS3_CONTROLLER
  readPs3InputPosition();
  readPs3Battery();
#endif ENABLE_PS3_CONTROLLER

#if ENABLE_PS4_CONTROLLER
  readPs4InputPosition();
  readPs4Battery();
#endif ENABLE_PS4_CONTROLLER

  updateDigipotOutputs();
}


/**
  Check if the tare button is pressed.
*/
#if ENABLE_ZEROSTICK
void checkTareButton()
{
  if (LOW == digitalRead(TARE_BUTTON_PIN))
  {
    // Button is pressed, do the tare!
    delay(100);
    tareCellReadings();
    delay(100);
  }
}
#endif ENABLE_ZEROSTICK

/**
  Send joystick movements to a pair of digital potentiometers using
  I2C. These can be connected to the joystick input of an electric
  wheelchair to simulate the normal joystick.

  Input (zerostick) position is centered at 0, with deflection to
  +/-100 full scale. This range of -100 to +100 must to mapped to the
  required range for the digital potentiometers, which is 0 to 127
  with the center point at 63.
*/
void updateDigipotOutputs()
{
#if ENABLE_DIGIPOT_OUTPUT
  if (digitalRead(DISABLE_PIN) == HIGH) // Pull this pin low to disable
  {
    if (millis() > g_last_digipot_time + DIGIPOT_INTERVAL)
    {
      /* Scale the -100 to +100 input to suit the required 0 to 127 output range */
      // X and Y are inverted for driving the pots
      int16_t pot_position_x = DIGIPOT_CENTER - (int)(g_input_x_position * DIGIPOT_SPEED * 63 / 100);
      int16_t pot_position_y = DIGIPOT_CENTER - (int)(g_input_y_position * DIGIPOT_SPEED * 63 / 100);

      /* Apply scaling so we don't lose most of the joystick position scale */
      pot_position_x = map(pot_position_x, 0, 127, 50, 80);
      pot_position_y = map(pot_position_y, 0, 127, 50, 80);

      /* Apply constraints to prevent going out of range on Permobil wheelchair joystick input */
      pot_position_x = constrain(pot_position_x, 50, 80);
      pot_position_y = constrain(pot_position_y, 50, 80);

      /* Update the positions of the digital potentiometers */
      digipot_x.setWiper(pot_position_x);
      digipot_y.setWiper(pot_position_y);

      g_last_digipot_time = millis();

#if ENABLE_DIGIPOT_DEBUGGING
      Serial.print(g_input_x_position);
      Serial.print("   ");
      Serial.print(g_input_y_position);
      Serial.print("   ");
      Serial.print("X: ");
      Serial.print(pot_position_x);
      Serial.print("   Y: ");
      Serial.println(pot_position_y);
#endif
    }
  }
#endif
}

/**
  Read the sensor pressure levels
*/
#if ENABLE_ZEROSTICK
void readZerostickInputPosition()
{
  //Serial.println("Reading");

  if (loadcells.available() == true)
  {
    if (1 == g_next_channel_to_read) // X axis
    {
      int32_t latest_reading = loadcells.getReading();
      x_samples.add(latest_reading + g_zero_tare_offset_x);
      g_channel_read_count++;

      g_input_x_position = x_samples.getMedian();
      // LOAD_CELL_RATING_GRAMS is the spec on the load cell.
      // INPUT_FULL_SCALE_GRAMS is effort required to reach full scale.
      g_input_x_position = g_input_x_position * LOAD_CELL_RATING_GRAMS / INPUT_FULL_SCALE_GRAMS;
      g_input_x_position = map(g_input_x_position, -LOAD_CELL_FULL_SCALE, LOAD_CELL_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_x_position = constrain(g_input_x_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_x_position)
      {
        g_input_x_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_x_position) {
        g_input_x_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_x_position = 0;
      }

      g_next_channel_to_read = 0;
      loadcells.setChannel(0);
      loadcells.calibrateAFE();

    } else {                         // Y axis
      int32_t latest_reading = loadcells.getReading();
      y_samples.add(latest_reading + g_zero_tare_offset_y);
      g_channel_read_count++;

      g_input_y_position = y_samples.getMedian();
      g_input_y_position = g_input_y_position * LOAD_CELL_RATING_GRAMS / INPUT_FULL_SCALE_GRAMS;
      g_input_y_position = map(g_input_y_position, -LOAD_CELL_FULL_SCALE, LOAD_CELL_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_y_position = constrain(g_input_y_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_y_position)
      {
        g_input_y_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_y_position) {
        g_input_y_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_y_position = 0;
      }

      g_channel_read_count   = 0;
      g_next_channel_to_read = 1;
      loadcells.setChannel(1);
      loadcells.calibrateAFE();
    }

#if ENABLE_ZEROSTICK_DEBUGGING
    Serial.print(g_input_x_position);
    Serial.print("  ");
    Serial.println(g_input_y_position);
#endif ENABLE_ZEROSTICK_DEBUGGING
  }
}
#endif ENABLE_ZEROSTICK

/**
  Reset the zero position of the load cells. Select each channel in turn and tare them.
*/
#if ENABLE_ZEROSTICK
void tareCellReadings()
{
  loadcells.setChannel(0);
  loadcells.calibrateAFE();                 // Internal calibration. Recommended after power up, gain changes, sample rate changes, or channel changes.
  loadcells.calculateZeroOffset(64);        // Tare operation, averaged across 64 readings
  //loadcells.setCalibrationFactor(g_x_calibration_factor);
  g_x_zero_offset = loadcells.getZeroOffset();
#if ENABLE_ZEROSTICK_DEBUGGING
  Serial.print("X zero offset: ");
  Serial.println(loadcells.getZeroOffset());
  Serial.print("X calibration factor: ");
  Serial.println(loadcells.getCalibrationFactor());
#endif ENABLE_ZEROSTICK_DEBUGGING

  loadcells.setChannel(1);
  loadcells.calibrateAFE();                 // Internal calibration. Recommended after power up, gain changes, sample rate changes, or channel changes.
  loadcells.calculateZeroOffset(64);        // Tare operation, averaged across 64 readings
  //loadcells.setCalibrationFactor(g_y_calibration_factor);
  g_y_zero_offset = loadcells.getZeroOffset();
#if ENABLE_ZEROSTICK_DEBUGGING
  Serial.print("Y zero offset: ");
  Serial.println(loadcells.getZeroOffset());
  Serial.print("Y calibration factor: ");
  Serial.println(loadcells.getCalibrationFactor());
#endif ENABLE_ZEROSTICK_DEBUGGING

  loadcells.setChannel(g_next_channel_to_read); // To leave the selected channel in the correct state

#if ENABLE_ZEROSTICK_DEBUGGING
  Serial.println("Tare complete");
#endif ENABLE_ZEROSTICK_DEBUGGING
}
#endif


#if ENABLE_PS3_CONTROLLER
/**
   Report that a controller is connected
*/
void ps3OnConnect()
{
  Serial.println("PS3 controller connected.");
}


/**
   Read and report the state of the PS3 controller battery
*/
void readPs3Battery()
{
#if ENABLE_PS3_DEBUGGING
  uint8_t ps3_battery;
  ps3_battery = Ps3.data.status.battery;
  Serial.print("Controller battery is ");
  if (ps3_battery == ps3_status_battery_charging)      Serial.println("charging");
  else if (ps3_battery == ps3_status_battery_full)     Serial.println("FULL");
  else if (ps3_battery == ps3_status_battery_high)     Serial.println("HIGH");
  else if (ps3_battery == ps3_status_battery_low)      Serial.println("LOW");
  else if (ps3_battery == ps3_status_battery_dying)    Serial.println("DYING");
  else if (ps3_battery == ps3_status_battery_shutdown) Serial.println("SHUTDOWN");
  else Serial.println("UNDEFINED");
#endif ENABLE_PS3_DEBUGGING
}

/**
   Read the right joystick from a paired PS4 controller (BLE)
*/
void readPs3InputPosition()
{
  if (Ps3.isConnected())
  {
    g_ps_controller_enabled = true;
    // Process the X axis
    if (NULL != Ps3.data.analog.stick.lx)
    {
      int8_t x_position = Ps3.data.analog.stick.rx; // What type of values do we get? Decimals?
#if ENABLE_PS3_DEBUGGING
      Serial.printf("Right stick X at %d\n", x_position);
#endif ENABLE_PS3_DEBUGGING
      g_input_x_position = map(x_position, -PS3_FULL_SCALE, PS3_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_x_position = constrain(g_input_x_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_x_position)
      {
        g_input_x_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_x_position) {
        g_input_x_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_x_position = 0;
      }
    }

    // Process the Y axis
    if (NULL != Ps3.data.analog.stick.ly)
    {
      int8_t y_position = Ps3.data.analog.stick.ry; // What type of values do we get? Decimals?
#if ENABLE_PS3_DEBUGGING
      Serial.printf("Right stick Y at %d\n", y_position);
#endif ENABLE_PS3_DEBUGGING
      g_input_y_position = map(y_position, PS3_FULL_SCALE, -PS3_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_y_position = constrain(g_input_y_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_y_position)
      {
        g_input_y_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_y_position) {
        g_input_y_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_y_position = 0;
      }
    }
  }
}
#endif ENABLE_PS3_CONTROLLER


#if ENABLE_PS4_CONTROLLER
/**
   Read and report the state of the PS4 controller battery
*/
void readPs4Battery()
{
#if ENABLE_PS4_DEBUGGING
  Serial.printf("Battery Level : %d\n", PS4.Battery());
  Serial.print("Charging: ");
  if(PS4.Charging())
  {
    Serial.println("YES");
  } else {
    Serial.println("no");
  }
#endif ENABLE_PS4_DEBUGGING
}

/**
   Read the right joystick from a paired PS4 controller (BLE)
*/
void readPs4InputPosition()
{
  if (PS4.isConnected())
  {
    // Process the X axis
    if (PS4.RStickX())
    {
      int8_t x_position = PS4.RStickX(); // What type of values do we get? Decimals?
#if ENABLE_PS4_DEBUGGING
      Serial.printf("Right stick X at %d\n", x_position);
#endif ENABLE_PS4_DEBUGGING
      g_input_x_position = map(x_position, -PS4_FULL_SCALE, PS4_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_x_position = constrain(g_input_x_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_x_position)
      {
        g_input_x_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_x_position) {
        g_input_x_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_x_position = 0;
      }
    }

    // Process the Y axis
    if (PS4.RStickY())
    {
      int8_t y_position = PS4.RStickY(); // What type of values do we get? Decimals?
#if ENABLE_PS4_DEBUGGING
      Serial.printf("Right stick Y at %d\n", y_position);
#endif ENABLE_PS4_DEBUGGING
      g_input_y_position = map(y_position, -PS4_FULL_SCALE, PS4_FULL_SCALE, -100, 100); // Adjust to a percentage of full force
      g_input_y_position = constrain(g_input_y_position, -100, 100);          // Prevent going out of bounds

      if (INPUT_DEAD_SPOT_SIZE < g_input_y_position)
      {
        g_input_y_position -= INPUT_DEAD_SPOT_SIZE;
      } else if (-1 * INPUT_DEAD_SPOT_SIZE > g_input_y_position) {
        g_input_y_position += INPUT_DEAD_SPOT_SIZE;
      } else {
        g_input_y_position = 0;
      }
    }
  }
}
#endif ENABLE_PS4_CONTROLLER
