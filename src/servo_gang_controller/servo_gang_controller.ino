#define SER_PIN 2
#define SERCLK_PIN 3
#define RCLK_PIN 4

#define SR_NUM_BITS 8

uint8_t sr_buffer = 0;

void setup() {
	Serial.begin(115200);
	pinMode(SER_PIN, OUTPUT);
	pinMode(SERCLK_PIN, OUTPUT);
	pinMode(RCLK_PIN, OUTPUT);

	digitalWrite(SER_PIN, LOW);
	digitalWrite(SERCLK_PIN, LOW);
	digitalWrite(RCLK_PIN, LOW);
}

void loop() {
	sr_buffer = 0b10101010;
	sr_write();
	delay(5000);
	sr_buffer = 0b01010101;
	sr_write();
	delay(5000);
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

