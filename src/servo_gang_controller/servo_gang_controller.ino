#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

/* Shift Register Defines */
#define SER_PIN 2
#define SERCLK_PIN 3
#define RCLK_PIN 4
#define SR_NUM_BITS 8 // also number of servos
#define SERVO_PERIOD_US 20000 // 50 Hz
#define SERVO_SHORT_US 500
#define SERVO_LONG_US 2500

/* Switchable Power Pin */
#define MOSFET_PIN 10

/* PWM Input Defines */
#define PWMIN1_PIN 11
#define PWMIN2_PIN 12
#define PWMIN1_HI_THRESHOLD_US 1000
#define PWMIN2_HI_THRESHOLD_US 1000



uint8_t sr_buffer = 0;
uint8_t servo_positions = 0; // 0 = closed, 1 = open

unsigned long start_us = 0;
unsigned long last_us = 0;

unsigned long last_servo_change_ms = 0;

uint32_t pwmin1_high_us = 0;
uint32_t pwmin1_last_rise_us = 0;
uint32_t pwmin2_high_us = 0;
uint32_t pwmin2_last_rise_us = 0;


void setup() {
	Serial.begin(115200);

	// set pinmodes
	pinMode(SER_PIN, OUTPUT);
	pinMode(SERCLK_PIN, OUTPUT);
	pinMode(RCLK_PIN, OUTPUT);
	pinMode(MOSFET_PIN, OUTPUT);
	pinMode(PWMIN1_PIN, INPUT);
	pinMode(PWMIN2_PIN, INPUT);

	// inititalize digital pins LOW
	digitalWrite(SER_PIN, LOW);
	digitalWrite(SERCLK_PIN, LOW);
	digitalWrite(RCLK_PIN, LOW);
	digitalWrite(MOSFET_PIN, LOW);

	// set up interrupts
	PCintPort::attachInterrupt(PWMIN1_PIN, pwmin1_isr, CHANGE);
	PCintPort::attachInterrupt(PWMIN2_PIN, pwmin2_isr, CHANGE);

	start_us = micros();
	servo_positions = 0b10101010;
}


void loop() {
	if (millis() - last_servo_change_ms > 500) {
		servo_positions = ~servo_positions;
		last_servo_change_ms = millis();
	}
	update_sr_buffer();
}

/**
Switch power to MOSFET based on pwmin2.
**/
void switch_power() {
	if (pwmin2_high_us > PWMIN2_HI_THRESHOLD_US) {
		// turn on MOSFET
		digitalWrite(MOSFET_PIN, HIGH);
	} else {
		// turn off MOSFET
		digitalWrite(MOSFET_PIN, LOW);
	}
}

/**
Updates the shift register buffer to set positions for servos that are open and closed.
**/
void update_sr_buffer() {
	unsigned long curr_us = micros() - start_us;
	if (curr_us >= SERVO_PERIOD_US) {
		// end of counting period
		start_us = micros(); // reset time counter
		sr_buffer = 0xff; // servo signals all start HIGH
	} else if (curr_us > SERVO_SHORT_US && last_us < SERVO_SHORT_US) {
		// set position signal for servos that are open
		for (int i = 0; i < SR_NUM_BITS; i++) {
			if (servo_positions & (1 << i)) {
				// servo is open
				sr_buffer &= ~(1 << i);
			}
		}	
	} else if (curr_us > SERVO_LONG_US && last_us < SERVO_LONG_US) {
		// set position signal for servos that are closed
		for (int i = 0; i < SR_NUM_BITS; i++) {
			if (!(servo_positions & (1 << i))) {
				// servo is closed
				sr_buffer &= ~(1 << i);
			}
		}
	}

	last_us = curr_us;
	sr_write(); // update servo pins
}

/**
Writes the shift register buffer to the shift register.
**/
void sr_write() {
	for (int i = SR_NUM_BITS - 1; i >= 0; i--) {
		uint8_t curr_bitmask = 1 << i;
		digitalWrite(SER_PIN, (sr_buffer & curr_bitmask) == 0 ? LOW : HIGH);

		// pulse serial clock
		digitalWrite(SERCLK_PIN, HIGH);
		digitalWrite(SERCLK_PIN, LOW);
	}
	// pulse register clock
	digitalWrite(RCLK_PIN, HIGH);
	digitalWrite(RCLK_PIN, LOW);
}

void pwmin1_isr() {
	if (digitalRead(PWMIN1_PIN)) {
		// rising
		pwmin1_last_rise_us = micros();
	} else {
		// falling
		pwmin1_high_us = micros() - pwmin1_last_rise_us;
	}
}

void pwmin2_isr() {
	if (digitalRead(PWMIN2_PIN)) {
		// rising
		pwmin2_last_rise_us = micros();
	} else {
		// falling
		pwmin2_high_us = micros() - pwmin2_last_rise_us;
	}
}
