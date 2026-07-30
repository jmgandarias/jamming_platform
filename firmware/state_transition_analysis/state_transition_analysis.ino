/*
Author: Juan M. Gandarias and Diego Caruana
Date: 06/05/2026
Description: state_transition_analysis

This code performs state transition analysis by controlling two relays to
depressurize and pressurize a chamber while sampling pressure data using an ADC.
The sampled data is printed to the Serial Monitor.

Hardware Setup:
- Pressure sensor connected to ADC pin 35.
- Relay 1 connected to pin 14 (for depressurization).
- Relay 2 connected to pin 12 (for pressurization).

*/

#include "driver/adc.h"
#include "driver/gpio.h"

const gpio_num_t RELAY_1_PIN = GPIO_NUM_14;
const gpio_num_t RELAY_2_PIN = GPIO_NUM_12;

const uint32_t N_SAMPLES = 1000;        // Number of samples to collect per transition
const uint32_t timer_frequency = 10000; // Timer frequency in Hz (10 kHz)

uint8_t number_experiments = 10; // Number of experiments to average

volatile uint32_t sample_index = 0;  // Current sample index
volatile bool sampling_done = false; // Flag to indicate sampling completion

uint16_t pressure[N_SAMPLES]; // Array to store pressure samples

hw_timer_t *sample_timer = NULL; // Timer for sampling

// Waits until the user sends the char "c" via serial port.
void wait_key(char c)
{
  bool key_received = false;
  int inByte;
  while (!key_received)
  {
    if (Serial.available() > 0)
    {
      inByte = Serial.read();
      if (inByte == c)
      {
        key_received = true;
      }
    }
  }
}

// Function to read raw ADC value from specified channel
void IRAM_ATTR sampleTimerInterrupt()
{
  if (sample_index < N_SAMPLES)
  {
    pressure[sample_index] = adc1_get_raw(ADC1_CHANNEL_7);
    sample_index++;
  }
  else
  {
    sampling_done = true;
    timerStop(sample_timer); // Stops the timer when done
  }
}

void setup()
{
  Serial.begin(115200);

  gpio_set_direction(RELAY_1_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(RELAY_2_PIN, GPIO_MODE_OUTPUT);

  gpio_set_level(RELAY_1_PIN, 0); // Activate first relay
  gpio_set_level(RELAY_2_PIN, 0); // Activate second relay

  adc1_config_width(ADC_WIDTH_BIT_12);                        // Configure ADC width
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12); // Configure ADC channel

  sample_timer = timerBegin(timer_frequency);                // Initialize timer
  timerAttachInterrupt(sample_timer, &sampleTimerInterrupt); // Attach interrupt
  timerAlarm(sample_timer, 1, true, 0);                      // Set alarm to trigger at specified frequency
  timerStop(sample_timer);                                   // Ensure timer is stopped initially

  wait_key('\n'); // WAIT BEFORE STARTING THE EXPERIMENTS
}

void loop()
{
  // Run the specified number of experiments
  if (number_experiments == 0)
  {
    // Serial.println("Experiments completed.");
    while (1)
    {
      delay(1000); // Do nothing
    }
  }

  // Check if sampling is not done
  if (!sampling_done)
  {
    /*
    Serial.println("Experiment number: " + String(number_experiments));
    Serial.println("=====================================");
    Serial.println("Starting soft-rigid transition sampling...");
    */

    delay(1000); // Wait 1s before starting sampling

    // First, depressurize the chamber (soft-rigid state transition)
    sample_index = 0;
    sampling_done = false;
    gpio_set_level(RELAY_1_PIN, 1); // Activate first relay
    // Start sampling soft-rigid transition
    timerStart(sample_timer);
    while (!sampling_done)
    {
      // Wait for sampling to complete
      delay(10);
    }
    // gpio_set_level(RELAY_1_PIN, 0); // Deactivate first relay

    // Sampling done, process data
    for (uint32_t i = 0; i < N_SAMPLES; i++)
    {
      Serial.println(pressure[i]);
    }

    // Serial.println("Starting rigid-soft transition sampling...");

    delay(100); // Wait before starting sampling again

    // Second, pressurize the chamber (rigid-soft state transition)
    // Start sampling rigid-soft transition
    sample_index = 0;
    sampling_done = false;
    gpio_set_level(RELAY_1_PIN, 0); // Deactivate first relay
    gpio_set_level(RELAY_2_PIN, 1); // Activate second relay
    // Start sampling soft-rigid transition
    timerStart(sample_timer);
    while (!sampling_done)
    {
      // Wait for sampling to complete
      delay(10);
    }
    gpio_set_level(RELAY_2_PIN, 0); // Deactivate first relay

    // Sampling done, process data
    for (uint32_t i = 0; i < N_SAMPLES; i++)
    {
      Serial.println(pressure[i]);
    }

    // delay(1000); // Wait 1s before next experiment ()

    number_experiments--; // Decrease the number of experiments left
    sample_index = 0;
    sampling_done = false;
    // gpio_set_level(RELAY_1_PIN, 0); // Deativate first relay
    // gpio_set_level(RELAY_2_PIN, 1); // Activate second relay
  }

  delay(10);
}