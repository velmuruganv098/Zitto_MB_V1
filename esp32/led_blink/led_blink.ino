/*
 * ESP32-S3-mini LED blink
 * LEDs on IO21 and IO25
 */

#define LED_PIN_1 21
#define LED_PIN_2 25

void setup()
{
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
}

void loop()
{
    digitalWrite(LED_PIN_1, HIGH);
    digitalWrite(LED_PIN_2, HIGH);
    delay(500);

    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
    delay(500);
}
