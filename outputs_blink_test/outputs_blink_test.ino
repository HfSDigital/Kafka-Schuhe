int i = 0;

#define LED_WIFI_OK           2   // D4
#define LED_MESSAGE_RECEIVED  16  // D0

#define LED_BATTERY_OK        9   // Der ist der schwierige
#define LED_BATTERY_LOW       10

void setup() {
  Serial.begin(115200);
 
  Serial.println("booting...");
  pinMode(LED_WIFI_OK, OUTPUT);      
  pinMode(LED_MESSAGE_RECEIVED, OUTPUT);

  pinMode(LED_BATTERY_OK, OUTPUT);   
  pinMode(LED_BATTERY_LOW, OUTPUT);  

}


void loop() {
  Serial.println(i); 
  i++;
  delay(100);
  digitalWrite(LED_WIFI_OK, HIGH);
  delay(100);
  digitalWrite(LED_WIFI_OK, LOW);

  delay(100);
  digitalWrite(LED_MESSAGE_RECEIVED, HIGH);
  delay(100);
  digitalWrite(LED_MESSAGE_RECEIVED, LOW);

  delay(100);
  digitalWrite(LED_BATTERY_OK, HIGH);
  delay(100);
  digitalWrite(LED_BATTERY_OK , LOW);

  delay(100);
  digitalWrite(LED_BATTERY_LOW, HIGH);
  delay(100);
  digitalWrite(LED_BATTERY_LOW, LOW);
}
