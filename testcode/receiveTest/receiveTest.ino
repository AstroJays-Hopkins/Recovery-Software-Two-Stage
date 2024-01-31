void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}


void read_data(String result[2]) {
  result[0] = Serial.readString();
  result[1] = Serial.readString();
}

void loop() {
  String data[2];
  read_data(data);
  String full = data[0] + "/" + data[1];
  Serial.println(full);
}