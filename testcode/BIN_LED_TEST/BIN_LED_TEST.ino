// Define the LED pins
const int ledPin1 = 2;
const int ledPin2 = 3;
const int ledPin3 = 4;

void setup() {
  // Set LED pins as output
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
}

void loop() {
  // Example: Display binary of decimal number 5
  displayBinaryOnLEDs(6);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(4);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(6);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(5);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(1);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(0);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(2);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(5);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(9);
  delay(1000); // Wait for 1 second
  displayBinaryOnLEDs(6);
  delay(1000); // Wait for 1 second
  
}

void displayBinaryOnLEDs(int decimalNumber) {
  // Extract individual bits and display on LEDs
  digitalWrite(ledPin1, (decimalNumber & 0b001) ? HIGH : LOW);
  digitalWrite(ledPin2, (decimalNumber & 0b010) ? HIGH : LOW);
  digitalWrite(ledPin3, (decimalNumber & 0b100) ? HIGH : LOW);
}
