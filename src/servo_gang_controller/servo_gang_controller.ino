#include <PinChangeInterrupt.h>

/* Shift Register Defines */
#define SER_PIN 2
#define SERCLK_PIN 3
#define RCLK_PIN 4
#define SR_NUM_BITS 8 // also number of servos
#define SERVO_PERIOD_US 20000 // 50 Hz
#define SERVO_SHORT_US 1000
#define SERVO_LONG_US 2000

/* Switchable Power Pin */
#define MOSFET_PIN 10

/* PWM Input Defines */
#define PWMIN1_PIN 11
#define PWMIN2_PIN 12
#define PWMIN1_HI_THRESHOLD_US 1500
#define PWMIN2_HI_THRESHOLD_US 1500

#define LED_PIN 13

uint8_t sr_buffer = 0; // shift register buffer
uint8_t servo_positions = 0; // 0b00000000 = all closed, 0b11111111 = all open

// used for shift register servo output
unsigned long start_us = 0;
unsigned long last_us = 0;

// unsigned long last_servo_change_ms = 0;

uint32_t pwmin1_high_us = 0;
uint32_t pwmin1_last_rise_us = 0;
uint32_t pwmin2_high_us = 0;
uint32_t pwmin2_last_rise_us = 0;

// for storing history
uint32_t pwmin1_last_high_us = 0;
uint32_t pwmin2_last_high_us = 0;
bool pwmin1_ignore = false;
bool pwmin2_ignore = false;

bool led_on = false;

uint16_t print_every = 0;
uint32_t last_servo_change_ms = 0;

void setup() {
	// Serial.begin(115200);

	// set pinmodes
	pinMode(SER_PIN, OUTPUT);
	pinMode(SERCLK_PIN, OUTPUT);
	pinMode(RCLK_PIN, OUTPUT);
	pinMode(MOSFET_PIN, OUTPUT);
	pinMode(PWMIN1_PIN, INPUT);
	pinMode(PWMIN2_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);

	// inititalize digital pins LOW
	digitalWrite(SER_PIN, LOW);
	digitalWrite(SERCLK_PIN, LOW);
	digitalWrite(RCLK_PIN, LOW);
	digitalWrite(MOSFET_PIN, LOW);
	digitalWrite(LED_PIN, HIGH);

	// set up interrupts
	attachPCINT(digitalPinToPCINT(PWMIN1_PIN), pwmin1_isr, CHANGE);
	// attachPCINT(digitalPinToPCINT(PWMIN2_PIN), pwmin2_isr, CHANGE);

	start_us = micros();
	servo_positions = 0b11111111;
}


void loop() {
	// print_every++;
	// if(!print_every) {
	// 	Serial.println(servo_positions, HEX);
	// }
	if (pwmin1_high_us > PWMIN1_HI_THRESHOLD_US && pwmin1_last_high_us < PWMIN1_HI_THRESHOLD_US) {
		if (servo_positions == 0) {
			// reset servos
			servo_positions = 0b11111111;
		} else {
			// activate one more servo
			// Serial.println("HA");
			pwmin1_last_high_us = pwmin1_high_us; // avoid duplicate triggers
			toggle_led();
			servo_positions = servo_positions >> 1;
		}
	}

	// if (millis() - last_servo_change_ms > 5000) {
	// 	servo_positions = ~servo_positions;
	// 	last_servo_change_ms = millis();
	// 	toggle_led();
	// }
	update_sr_buffer();
	// enable interrupts and avoid triggering a false super long interval
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
		// last_us = curr_us;
		// sr_write(); // update servo pins
	} else if (curr_us > SERVO_SHORT_US && last_us < SERVO_SHORT_US) {
		// set position signal for servos that are open
		for (int i = 0; i < SR_NUM_BITS; i++) {
			if (servo_positions & (1 << i)) {
				// servo is open
				sr_buffer &= ~(1 << i);
			}
		}
		// last_us = curr_us;
		// sr_write(); // update servo pins	
	} else if (curr_us > SERVO_LONG_US && last_us < SERVO_LONG_US) {
		// set position signal for servos that are closed
		for (int i = 0; i < SR_NUM_BITS; i++) {
			if (!(servo_positions & (1 << i))) {
				// servo is closed
				sr_buffer &= ~(1 << i);
			}
		}
		// last_us = curr_us;
		// sr_write(); // update servo pins
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
		// rising edge
		pwmin1_last_rise_us = micros();
	} else {
		// falling edge
		pwmin1_last_high_us = pwmin1_high_us;
		pwmin1_high_us = micros() - pwmin1_last_rise_us;
	}
}

void pwmin2_isr() {
	if (digitalRead(PWMIN2_PIN)) {
		// rising edge
		pwmin2_last_rise_us = micros();
	} else {
		// falling edge
		pwmin2_last_high_us = pwmin2_high_us;
		pwmin2_high_us = micros() - pwmin2_last_rise_us;
	}
}

void toggle_led() {
	digitalWrite(LED_PIN, led_on);
	led_on = !led_on;
}
