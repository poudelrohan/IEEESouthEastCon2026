void setup() {
  Serial.begin(9600);    // Serial0 (USB) at 9600 baud
  Serial1.begin(9600);   // Serial1 (pins 19/18) at 9600 baud

  Serial.println("hola mundo");
  Serial1.println("bye");
}

void loop() {
}
