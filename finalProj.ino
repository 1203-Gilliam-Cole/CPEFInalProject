// AUTHOR: COLE GILLIAM, JORDAN CADELINA
// CPE FINAL PROJECT

// LIBRARIES
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <LiquidCrystal.h>

// MACROS & PIN MAPPING
#define BUTTON_PIN PB0        // Digital pin 53 (START/STOP BUTTON)
#define RESET_BUTTON_PIN PB2  // Digital pin 51 (RESET BUTTON)
#define YELLOW_PIN PA5        // Digital pin 27 
#define GREEN_PIN PA3         // Digital pin 25 
#define RED_PIN PA1           // Digital pin 23 
#define BLUE_PIN PA0          // Digital pin 22 

#define YELLOW_ON PORTA |= (1 << PORTA5);
#define YELLOW_OFF PORTA &= ~(1 << PORTA5);
#define GREEN_ON PORTA |= (1 << PORTA3);
#define GREEN_OFF PORTA &= ~(1 << PORTA3);
#define RED_ON PORTA |= (1 << PORTA1);
#define RED_OFF PORTA &= ~(1 << PORTA1);
#define BLUE_ON PORTA |= (1 << PORTA0);
#define BLUE_OFF PORTA &= ~(1 << PORTA0);

// GLOBAL VARIABLES
volatile bool START_STOP_button_flag = false; // Start/Stop button flag
volatile bool reset_button_flag = false;      // Reset button flag
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5; //LCD PINS
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// ENUMS
typedef enum {
  DISABLED,  // System is off
  IDLE,      // System is idle, monitoring sensors
  ERROR,     // Error state
  RUNNING    // Fan motor running
} cooler_state_t;

cooler_state_t current_state = DISABLED;  // Initial state

// FUNCTION DECLARATIONS
void button_init();
int waterLevelSensor();
void uart_init();
void uart_print(const char *str);
void uart_putchar(char c);
void adc_init();
uint16_t adc_read(uint8_t channel);

// MAIN FUNCTION
int main(void) {
  // Initialize peripherals
  uart_init();
  adc_init();
  button_init();

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0); 

  // Main loop
  while (1) {
    switch (current_state) {
      case DISABLED:
        lcd.clear();
        uart_print("Currently Disabled\n");
        YELLOW_ON
        while (current_state == DISABLED) {
          if (START_STOP_button_flag) {
            YELLOW_OFF
            START_STOP_button_flag = false;
            current_state = IDLE;
          }
        }
        break;

      case IDLE:
        uart_print("Currently Idle\n");
        lcd.clear();
        lcd.print("N/A");
        GREEN_ON
        while (current_state == IDLE) {
          uart_print("Water Level: ");
          if (waterLevelSensor() < 50) {
            GREEN_OFF
            current_state = ERROR;
          } else if (true) {  // Replace "false" with temperature sensor logic
            GREEN_OFF
            current_state = RUNNING;
          }
          _delay_ms(1000);
          if (START_STOP_button_flag) {
            GREEN_OFF
            START_STOP_button_flag = false;
            current_state = DISABLED;
          }
        }
        break;

      case ERROR:
        lcd.clear();
        lcd.print("ERROR: LOW WATER LEVEL");
        uart_print("Error: Low Water Level\n");
        RED_ON
        while (current_state == ERROR) {
          if (START_STOP_button_flag) {
            RED_OFF
            START_STOP_button_flag = false;
            current_state = DISABLED;
          }
          _delay_ms(100);
          if (reset_button_flag) {
            reset_button_flag = false;
            RED_OFF
            current_state = IDLE;
          }
        }
        break;

      case RUNNING:
          lcd.clear();
          lcd.print("N/A");
          BLUE_ON
          uart_print("System Running\n");
        while(current_state == RUNNING){
          if (START_STOP_button_flag) {
            BLUE_OFF
            START_STOP_button_flag = false;
            current_state = DISABLED;
          }   
          if (waterLevelSensor() < 50) {
            BLUE_OFF
            current_state = ERROR;
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
  DDRA |= (1 << RED_PIN);
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

// ISR for START/STOP Button
ISR(PCINT0_vect) {
  if (!(PINB & (1 << BUTTON_PIN))) {    // Check if button is pressed
    _delay_ms(50);                      // Debounce
    if (!(PINB & (1 << BUTTON_PIN))) {
      START_STOP_button_flag = true;   // Set flag
    }
  }
}

// ISR for RESET Button
ISR(PCINT2_vect) {
  if (!(PINB & (1 << RESET_BUTTON_PIN))) {    // Check if reset button is pressed
    _delay_ms(50);                            // Debounce
    if (!(PINB & (1 << RESET_BUTTON_PIN))) {
      reset_button_flag = true;              // Set flag
    }
  }
}