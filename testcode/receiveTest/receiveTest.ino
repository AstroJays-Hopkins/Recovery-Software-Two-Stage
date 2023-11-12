void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  String a = Serial.readString();
  Serial.println(a);
}