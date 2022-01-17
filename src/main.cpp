#include <Arduino.h>
#include <EEPROM.h>
#include <avr/sleep.h>

#include <PCD8544.h>
#include <Adafruit_LIS3DH.h>

#define LCD_DC A3
#define LCD_RST A2
#define LCD_SCK 13
#define LCD_DIN 11
#define LCD_CS 10

#define SDA A4
#define SDL A5
#define INTERRUPT 2
#define ACC_DELAY 500

#define LEDC 4
#define LEDR 5
#define LEDB 6
#define LEDL 7
#define LEDF 8
#define DELTA 0.5

#define BUTTON A1
#define BASE_DELAY 50
#define CALIBRATE_HOLD_TIME 3000
#define RESET_HOLD_TIME 7000
#define TIME_TO_SLEEP 5000

enum class display_mode
{
  ACC,
  RAD,
  DEG
};
display_mode operator++(display_mode &mode)
{
  mode = static_cast<display_mode>((static_cast<int>(mode) + 1) % 3);
  return mode;
}

display_mode display = display_mode::ACC;

PCD8544 lcd = PCD8544(LCD_SCK, LCD_DIN, LCD_DC, LCD_RST, LCD_CS);

Adafruit_LIS3DH lis3dh = Adafruit_LIS3DH();
#define CLICK_THRESHOLD 1

float offsets[3] = { 0.0 };
const int sample_count = ACC_DELAY / BASE_DELAY;
int acc_counter = 1;
bool custom_offsets = false;

const int calibrate_hold_count = CALIBRATE_HOLD_TIME / BASE_DELAY;
const int reset_hold_count = RESET_HOLD_TIME / BASE_DELAY;
int hold_counter = 0;

//const int sleep_threshold = TIME_TO_SLEEP / BASE_DELAY;
//int sleep_counter = 0;

void read_offsets()
{
  EEPROM.get(0, offsets[0]);
  EEPROM.get(4, offsets[1]);
  EEPROM.get(8, offsets[2]);

  if (offsets[0] == offsets[1] && offsets[1] == offsets[2] && offsets[2] == 0)
  {
    custom_offsets = false;
  }
  else
  {
    custom_offsets = true;
  }
}

void clear_leds()
{
  digitalWrite(LEDC, 1);
  digitalWrite(LEDF, 1);
  digitalWrite(LEDR, 1);
  digitalWrite(LEDB, 1);
  digitalWrite(LEDL, 1);
}

void setup()
{
  pinMode(LEDF, OUTPUT);
  pinMode(LEDC, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDL, OUTPUT);

  pinMode(BUTTON, INPUT);
  pinMode(INTERRUPT, INPUT);

  clear_leds();

  lcd.begin(84, 48);
  lcd.clear();

  if (!lis3dh.begin(0x18))
  {
    lcd.print("LIS3DH could not start.");
    while (true)
    {
      yield();
    }
  }
  lis3dh.setDataRate(LIS3DH_DATARATE_10_HZ);
  lis3dh.setRange(LIS3DH_RANGE_2_G);

  read_offsets();
}

void save_offsets()
{
    EEPROM.put(0, offsets[0]);
    EEPROM.put(4, offsets[1]);
    EEPROM.put(8, offsets[2]);
}

void light_leds(float *values)
{
  clear_leds();
  bool flat = true;

  if (abs(values[0]) > DELTA)
  {
    flat = false;

    if (values[0] > 0)
    {
      digitalWrite(LEDL, 1);
      digitalWrite(LEDR, 0);
    }
    else
    {
      digitalWrite(LEDL, 0);
      digitalWrite(LEDR, 1);
    }
  }

  if (abs(values[1]) > DELTA)
  {
    flat = false;

    if (values[1] > 0)
    {
      digitalWrite(LEDB, 1);
      digitalWrite(LEDF, 0);
    }
    else
    {
      digitalWrite(LEDB, 0);
      digitalWrite(LEDF, 1);
    }
  }

  if (flat)
  {
    digitalWrite(LEDR, 1);
    digitalWrite(LEDF, 1);
    digitalWrite(LEDB, 1);
    digitalWrite(LEDL, 1);

    digitalWrite(LEDC, 0);
  }
}

void calc_acc(float *values)
{
  float x_2 = values[0] * values[0],
        y_2 = values[1] * values[1],
        z_2 = values[2] * values[2];

  values[3] = sqrt(x_2 + y_2 + z_2);

  switch (display)
  {
  case display_mode::ACC:
    return;
  case display_mode::RAD:
  {
    values[0] = atan2(x_2, sqrt(y_2 + z_2));
    values[1] = atan2(y_2, sqrt(x_2 + z_2));
    //values[2] = atan2(sqrt(x_2 + y_2), z_2);
    break;
  }
  case display_mode::DEG:
  {
    values[0] = atan2(x_2, sqrt(y_2 + z_2)) * 57.3;
    values[1] = atan2(y_2, sqrt(x_2 + z_2)) * 57.3;
    //values[2] = atan2(sqrt(x_2 + y_2), z_2) * 57.3;
    break;
  }
  }
}

void print_acc(sensors_event_t &event)
{
  float values[4] = {
      event.acceleration.x + offsets[0],
      event.acceleration.y + offsets[1],
      event.acceleration.z + offsets[2],
      0.0};

  light_leds(values);

  calc_acc(values);

  lcd.clear();
  if (custom_offsets)
  {
    lcd.print("*");
  }

  switch (display)
  {
  case display_mode::ACC:
    lcd.print("Acc, m/s^2:");
    break;
  case display_mode::RAD:
    lcd.print("Rot, rad:");
    break;
  case display_mode::DEG:
    lcd.print("Rot, deg:");
    break;
  }

  lcd.setCursor(0, 1);
  lcd.print("X: ");
  lcd.print(values[0]);

  lcd.setCursor(0, 2);
  lcd.print("Y: ");
  lcd.print(values[1]);

  if (display == display_mode::ACC)
  {
    lcd.setCursor(0, 3);
    lcd.print("Z: ");
    lcd.print(values[2]);
  }

  // lcd.setCursor(0, 4);
  // lcd.print("a: ");
  // lcd.print(values[3]);
  
  // lcd.setCursor(0, 5);
  // lcd.print("T: ");
  // lcd.print(event.temperature);
}

void calibrate()
{
  float new_offsets[3] = {0.0};
  const int times = 3, // TODO
    rate = 100,
    samples = 1000 / rate,
    total = samples * times;

  for (int i = 0; i < times; i++)
  {
    lcd.clear();
    lcd.print("Calibrating, please wait.");
    lcd.setCursor(0, 2);

    char buffer[64] = {0};
    float prog = 100.0 * i / times;
    sprintf(buffer, "Progress: %.1f%%", prog);
    lcd.print(buffer);
    
    for (int j = 0; j < samples; j++)
    {
      sensors_event_t event;
      lis3dh.getEvent(&event);

      new_offsets[0] += event.acceleration.x;
      new_offsets[1] += event.acceleration.y;
      new_offsets[2] += event.acceleration.z;

      delay(rate);
    }
  }

  offsets[0] = -new_offsets[0] / total;
  offsets[1] = -new_offsets[1] / total;
  offsets[2] = -new_offsets[2] / total;

  save_offsets();

  custom_offsets = true;

  lcd.clear();
  lcd.print("Calibration success.");
  delay(3000);
  lcd.clear();
}

void reset()
{
  lcd.clear();
  lcd.print("Offsets have been reset.");

  offsets[0] = 0.0;
  offsets[1] = 0.0;
  offsets[2] = 0.0;

  save_offsets();

  custom_offsets = false;

  delay(3000);
  lcd.clear();
}

void wake_up()
{
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT));
}

void go_to_sleep()
{
  lis3dh.setClick(1, CLICK_THRESHOLD);
  delay(1000);
  lcd.clear();
  clear_leds();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT), wake_up, CHANGE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void loop()
{
  // if (sleep_counter == sleep_threshold)
  // {
  //   go_to_sleep();
  //   sleep_counter = 0;
  // }
  
  if (digitalRead(BUTTON) == LOW)
  {
    ++hold_counter;
    //sleep_counter = 0;

    if (hold_counter >= calibrate_hold_count)
    {
      lcd.clear();
      lcd.print("Release the button to calibrate, hold further to reset.");
      
      while (digitalRead(BUTTON) == LOW)
      {
        if (++hold_counter == reset_hold_count)
        {
          break;
        }
        delay(BASE_DELAY);
      }
      
      if (hold_counter < reset_hold_count)
      {
        calibrate();
        hold_counter = 0;
      }
    }

    if (hold_counter == reset_hold_count)
    {
      lcd.clear();
      lcd.print("Release the button to reset.");
      while (digitalRead(BUTTON) == LOW)
      {
        delay(BASE_DELAY);
      }
      reset();
      hold_counter = 0;
    }
  }
  else
  {
    if (hold_counter > 0)
    {
      ++display;
    }
    hold_counter = 0;
    //++sleep_counter;
  }

  if (acc_counter == sample_count)
  {
    sensors_event_t event;
    lis3dh.getEvent(&event);

    print_acc(event);
    acc_counter = 0;
  }
  else
  {
    acc_counter++;
  }

  delay(BASE_DELAY);
}
