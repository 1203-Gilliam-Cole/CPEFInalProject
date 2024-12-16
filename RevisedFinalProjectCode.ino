// LIBRARIES
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Servo.h>  // Include Servo library

// MACROS & PIN MAPPING
#define BUTTON_PIN PB0        // Digital pin 53 (START/STOP BUTTON)
#define RESET_BUTTON_PIN PB2  // Digital pin 51 (RESET BUTTON)
#define YELLOW_PIN PA5        // Digital pin 27 
#define GREEN_PIN PA3         // Digital pin 25 
#define RED_PIN PA1           // Digital pin 23 
#define BLUE_PIN PA0          // Digital pin 22 

#define FAN_PIN PB3           // New fan motor pin (Digital pin 49)

#define YELLOW_ON PORTA |= (1 << PORTA5);
#define YELLOW_OFF PORTA &= ~(1 << PORTA5);
#define GREEN_ON PORTA |= (1 << PORTA3);
#define GREEN_OFF PORTA &= ~(1 << PORTA3);
#define RED_ON PORTA |= (1 << PORTA1);
#define RED_OFF PORTA &= ~(1 << PORTA1);
#define BLUE_ON PORTA |= (1 << PORTA0);
#define BLUE_OFF PORTA &= ~(1 << PORTA0);
#define FAN_ON PORTB |= (1 << PORTB3);  // Turn on fan
#define FAN_OFF PORTB &= ~(1 << PORTB3);  // Turn off fan

// GLOBAL VARIABLES
volatile bool START_STOP_button_flag = false; // Start/Stop button flag
volatile bool reset_button_flag = false;      // Reset button flag
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5; // LCD PINS
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); // LCD object

// Create a Servo object
Servo fanServo;  

// ENUMS
typedef enum {
  DISABLED,  // System is off
  IDLE,      // System is idle, monitoring sensors
  ERROR,     // Error state
  RUNNING    // Fan motor running
} cooler_state_t;

cooler_state_t current_state = DISABLED;  // Initial state

// RTC Time Variables
int seconds, minutes, hours, day, month, year;

// FUNCTION DECLARATIONS
void button_init();
int waterLevelSensor();
void uart_init();
void uart_print(const char *str);
void uart_putchar(char c);
void adc_init();
uint16_t adc_read(uint8_t channel);
void turn_off_all_leds();
void update_lcd(const char *message);
bool rtc_init();
void read_time();
void print_time();
int bcdToDec(byte val);
byte decToBcd(int val);

// MAIN FUNCTION
int main(void) {
  // Initialize peripherals
  uart_init();
  adc_init();
  button_init();
  Wire.begin();  // Initialize I2C for RTC
  
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("System Init...");
  _delay_ms(2000);
  lcd.clear();

  // Initialize RTC
  if (!rtc_init()) {
    lcd.setCursor(0, 0);
    lcd.print("RTC Init Fail");
    uart_print("RTC Init Fail\n");
    while (1);  // Stop if RTC is not initialized
  }

  // Initialize Servo Motor on pin 9
  fanServo.attach(9); // Attach servo to pin 9

  // Initialize Fan Motor Pin
  DDRB |= (1 << FAN_PIN);  // Set FAN_PIN as output

// Main loop
while (1) {
  switch (current_state) {
    case DISABLED:
      turn_off_all_leds();
      update_lcd("System Disabled");
      uart_print("Currently Disabled\n");
      YELLOW_ON
      FAN_OFF  // Turn off fan in DISABLED state
      while (current_state == DISABLED) {
        if (START_STOP_button_flag) {
          YELLOW_OFF
          START_STOP_button_flag = false;
          current_state = IDLE; // Transition to IDLE state
        }
        _delay_ms(10);  // Small delay for button press detection
      }
      break;

    case IDLE:
      turn_off_all_leds();
      uart_print("Currently Idle\n");
      update_lcd("System Idle");
      GREEN_ON
      FAN_OFF  // Turn off fan in IDLE state

      while (current_state == IDLE) {
        if (START_STOP_button_flag) {
          START_STOP_button_flag = false;
          GREEN_OFF
          current_state = RUNNING;  // Transition to RUNNING state
        }
        if (reset_button_flag) {
          reset_button_flag = false;
          current_state = DISABLED;  // Transition to DISABLED state
        }
        _delay_ms(100);  // Debounce delay for button press detection
      }
      break;

    case ERROR:
      turn_off_all_leds();
      update_lcd("ERROR: LOW WATER");
      uart_print("Error: Low Water Level\n");
      RED_ON
      FAN_OFF  // Turn off fan in ERROR state
      while (current_state == ERROR) {
        if (START_STOP_button_flag) {
          RED_OFF
          START_STOP_button_flag = false;
          current_state = DISABLED;  // Transition to DISABLED state
        }
        _delay_ms(100);
        if (reset_button_flag) {
          reset_button_flag = false;
          RED_OFF
          current_state = IDLE;  // Transition to IDLE state
        }
      }
      break;

    case RUNNING:
      turn_off_all_leds();
      update_lcd("System Running");
      BLUE_ON
      uart_print("System Running\n");
      FAN_ON  // Turn on fan in RUNNING state

      while (current_state == RUNNING) {
        // Control the servo using the potentiometer
        int potValue = analogRead(A1); // Read potentiometer value (0-1023)
        int angle = map(potValue, 0, 1023, 0, 180); // Map it to servo angle (0-180)
        fanServo.write(angle); // Set servo position

        // Wait for button press to change state from RUNNING
        if (START_STOP_button_flag) {
          BLUE_OFF
          START_STOP_button_flag = false;
          current_state = DISABLED;  // Transition to DISABLED state if START pressed in RUNNING
        }

        // Check water level and transition to ERROR if necessary
        if (waterLevelSensor() < 50) {
          BLUE_OFF
          current_state = ERROR;  // Transition to ERROR if water level is low
        }

        // Handle reset button for RUNNING state (transition to IDLE)
        if (reset_button_flag) {
          reset_button_flag = false;
          BLUE_OFF
          current_state = IDLE;  // Transition to IDLE state if RESET pressed
        }

        _delay_ms(1000);
      }
      break;

    default:
      break;
    }
  }
}
// FUNCTION IMPLEMENTATIONS

void turn_off_all_leds() {
  YELLOW_OFF
  GREEN_OFF
  RED_OFF
  BLUE_OFF
}

void update_lcd(const char *message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);  // Display the system state message
  print_time();        // Display the time on the LCD
}

void print_time() {
  // Fetch the current time
  read_time();
  // Format the time string
  char time_str[9];
  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);
  
  // Display the formatted time on the second row of the LCD
  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(time_str);
  
  // Send the time over UART for serial monitoring
  uart_print("Time: ");
  uart_print(time_str);
  uart_print("\n");
}

void button_init() {
  // Initialize START/STOP button
  DDRB &= ~(1 << BUTTON_PIN);       // Set BUTTON_PIN as input
  PORTB |= (1 << BUTTON_PIN);       // Enable pull-up resistor
  PCICR |= (1 << PCIE0);            // Enable pin change interrupt for PCINT0-PCINT7
  PCMSK0 |= (1 << PCINT0);          // Enable interrupt for BUTTON_PIN

  // Initialize RESET button
  DDRB &= ~(1 << RESET_BUTTON_PIN); // Set RESET_BUTTON_PIN as input
  PORTB |= (1 << RESET_BUTTON_PIN); // Enable pull-up resistor
  PCMSK0 |= (1 << PCINT2);          // Enable interrupt for RESET_BUTTON_PIN

  sei(); // Enable global interrupts

  // Initialize LEDs
  DDRA |= (1 << YELLOW_PIN);
  DDRA |= (1 << GREEN_PIN);
  DDRA |= (1 << RED_PIN);
  DDRA |= (1 << BLUE_PIN);
}

int waterLevelSensor() {
  uint16_t water_level = adc_read(0);  // Read ADC channel 0 (A0)
  char buffer[10];
  itoa(water_level, buffer, 10);  // Convert integer to string
  uart_print(buffer);             // Print water level value
  uart_print("\n");
  return water_level;
}

void uart_init() {
  unsigned int ubrr = 103;  // 9600 baud rate for 16MHz clock
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);                   // Enable transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit, no parity
}

void uart_putchar(char c) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uart_print(const char *str) {
  while (*str) {
    uart_putchar(*str++);
  }
}

void adc_init() {
  ADMUX = (1 << REFS0);                                // Use AVcc as reference
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, prescaler 64
}

uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Mask channel bits
  ADCSRA |= (1 << ADSC);                      // Start conversion
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

bool rtc_init() {
  // Try to communicate with the DS1307 RTC
  Wire.beginTransmission(0x68); // DS1307 address
  if (Wire.endTransmission() != 0) {
    // RTC not responding
    return false;
  }

  // If RTC is in reset mode, set the time
  read_time();
  return true;
}

void read_time() {
  Wire.beginTransmission(0x68);
  Wire.write(0x00); // Start reading at the seconds register
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 7); // Read 7 bytes (Seconds, Minutes, Hours, Day, Date, Month, Year)
  
  seconds = bcdToDec(Wire.read() & 0x7F);  // Mask out the 7th bit
  minutes = bcdToDec(Wire.read());
  hours = bcdToDec(Wire.read() & 0x3F);  // Mask out the 6th bit (24-hour mode)
  day = bcdToDec(Wire.read());
  month = bcdToDec(Wire.read());
  year = bcdToDec(Wire.read());
}

int bcdToDec(byte val) {
  return ((val / 16 * 10) + (val % 16));
}

byte decToBcd(int val) {
  return ((val / 10 * 16) + (val % 10));
}

// ISR for START/STOP Button
ISR(PCINT0_vect) {
  if (!(PINB & (1 << BUTTON_PIN))) {    // Check if button is pressed
    _delay_ms(50);                      // Debounce
    if (!(PINB & (1 << BUTTON_PIN))) {
      START_STOP_button_flag = true;    // Set the button flag
    }
  }
}

// ISR for RESET Button
ISR(PCINT2_vect) {
  if (!(PINB & (1 << RESET_BUTTON_PIN))) {    // Check if RESET button is pressed
    _delay_ms(50);                          // Debounce
    if (!(PINB & (1 << RESET_BUTTON_PIN))) {
      reset_button_flag = true;        // Set the reset flag
    }
  }
}