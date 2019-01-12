#define SER_PIN 2
#define SERCLK_PIN 3
#define RCLK_PIN 4

#define SR_NUM_BITS 8 // also number of servos

#define SERVO_PERIOD_US 20000 // 50 Hz
#define SERVO_SHORT_US 500
#define SERVO_LONG_US 2500

uint8_t sr_buffer = 0;
uint8_t servo_positions = 0; // 0 = closed, 1 = open

unsigned long start_us = 0;
unsigned long last_us = 0;

unsigned long last_servo_change_ms = 0;

void setup() {
	Serial.begin(115200);

	// set pinmodes
	pinMode(SER_PIN, OUTPUT);
	pinMode(SERCLK_PIN, OUTPUT);
	pinMode(RCLK_PIN, OUTPUT);

	// inititalize digital pins LOW
	digitalWrite(SER_PIN, LOW);
	digitalWrite(SERCLK_PIN, LOW);
	digitalWrite(RCLK_PIN, LOW);

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

