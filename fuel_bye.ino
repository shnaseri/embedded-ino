// wifi
#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
//acc
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>




//remote
const int aPin = 14;
const int bPin = 12;
const int cPin = 25;
const int dPin = 13;

// relay
#define fuelControlWire 15


static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// Set GPIOs for LED and ACC 
//if accident is detected led is high
const int ledAccident = 2;
//if door is locked led is low
const int ledDoor = 4;

bool safetyMode = true;
bool carDoorOpen = false;
bool carCrash= false;
bool carFuel = false;

String wifiName = "honor 8";
String wifiPass = "Mjavad80";


//acc
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

 
//acc 
float accX, accY, accZ;



void getEventNew(void)
{
/* Get a new sensor event */
sensors_event_t event;
accel.getEvent(&event);
 
/* Display the results (acceleration is measured in m/s^2) */
Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" ");
Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" ");
Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print(" ");Serial.println("m/s^2 ");
if((accX || accY || accZ) && !carCrash)
  {
    detectAccident(event.acceleration.x, accX, event.acceleration.y, accY, event.acceleration.z, accZ);   
  }
accX= event.acceleration.x; 
accY= event.acceleration.y; 
accZ= event.acceleration.z;  
}


//remote
void IRAM_ATTR aButton() {
    Serial.println("A-car door opened");
    carDoorOpen = true;
    digitalWrite(ledDoor, HIGH);
    if(!carCrash){digitalWrite(fuelControlWire, LOW); carFuel = true;}
}
void IRAM_ATTR bButton() {
    Serial.println("B-car door locked");
    carDoorOpen = false;
    digitalWrite(ledDoor, LOW);
    if(safetyMode){digitalWrite(fuelControlWire, HIGH); carFuel = false;}
}
void IRAM_ATTR cButton() {
    Serial.println("C- safety mode off");
    safetyMode = false;
    if(!carDoorOpen && !carCrash){digitalWrite(fuelControlWire, LOW); carFuel = true;}
}
void IRAM_ATTR dButton() {
    Serial.println("D- safety mode on");
    safetyMode = true;
    if(!carDoorOpen){digitalWrite(fuelControlWire, HIGH); carFuel = false;}
}

//acident

void detectAccident(float currentAccX, float previousAccX, float currentAccY, float previousAccY, float currentAccZ, float previousAccZ)                                                        
{
  if(abs(currentAccX - previousAccX)> 10 || abs(currentAccY - previousAccY)> 10 || abs(currentAccZ - previousAccZ)> 10){
    digitalWrite(ledAccident, HIGH);
    digitalWrite(fuelControlWire, HIGH);
    carCrash= true;
    Serial.println("car crash!!!!!!!!!!!!");
    
 
  }
}

// http
HTTPClient http;


int mqttInterval = 2, mqttNow, mqttLastUpdate;

#define MQTT_USER "shnaseri"
#define MQTT_PASS "38wdFg#T7YJDSky"
#define MQTT_ID "system"
#define MQTT_URL "d1dca11bd4f54c57892e2850aea09719.s2.eu.hivemq.cloud"
#define MQTT_PORT 8883

const char *recievingTopicDoor = "app/door";
const char *recievingTopicSafety = "app/safety";
const char *sendingTopic = "system";
const char *stat_true = "true";

char mqttBuffer[4096];
char mqttFormat[] = "{\"door\":%d,\"fuel\":%d,\"crash\":%d,\"safety\":%d}";


void connecttowifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifiName.c_str(), wifiPass.c_str());
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(100);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
  }
//  else {
//    Serial.print("Connected");
//  }

}


WiFiClientSecure espClient;
PubSubClient client(espClient);


void setup_mqtt_client() {
  // Loop until we’re reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    // Attempt to connect
    // Insert your password
    if (client.connect(client_id.c_str(), "shnaseri", "38wdFg#T7YJDSky")) {
      Serial.println("connected");
      // Once connected, publish an announcement…
      // … and resubscribe
      client.subscribe(recievingTopicDoor);
      client.subscribe(recievingTopicSafety);
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void callback(char *topic, uint8_t *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  char msgTmp;
  for (int i = 0; i < length; i++)
  {
    msgTmp = payload[i];
  }
  Serial.println(topic);
  if(strcmp(topic,recievingTopicDoor) == 0){
     if(msgTmp == '1'){
    Serial.println("A-car door opened");
    carDoorOpen = true;
    digitalWrite(ledDoor, HIGH);
    if(!carCrash){digitalWrite(fuelControlWire, LOW); carFuel = true;}
      }
     else{
    Serial.println("B-car door locked");
    carDoorOpen = false;
    digitalWrite(ledDoor, LOW);
    if(safetyMode){digitalWrite(fuelControlWire, HIGH); carFuel = false;}
     }
  
    }
 if(strcmp(topic,recievingTopicSafety) == 0) {
       if(msgTmp == '1'){
    Serial.println("C- safety mode off");
    safetyMode = false;
    if(!carDoorOpen && !carCrash){digitalWrite(fuelControlWire, LOW); carFuel = true;}
      }
     else{
    Serial.println("D- safety mode on");
    safetyMode = true;
    if(!carDoorOpen){digitalWrite(fuelControlWire, HIGH); carFuel = false;}
     }
     }
  //  lastTrigger = millis();
  //  intervalPassed = 0;
  Serial.println();
}

// setp
void setup()
{

  // Serial port for debugging purposes
  Serial.begin(115200);

  // Setup Wifi
  WiFi.begin(wifiName.c_str(), wifiPass.c_str());
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  randomSeed(micros());
  
  espClient.setCACert(root_ca);
  client.setServer(MQTT_URL, MQTT_PORT);
  client.setCallback(callback);
  client.setBufferSize(4096);
  setup_mqtt_client();

  
//  reciver Sensor mode INPUT_PULLUP
  pinMode(aPin, INPUT_PULLUP);
  pinMode(bPin, INPUT_PULLUP);
  pinMode(cPin, INPUT_PULLUP);
  pinMode(dPin, INPUT_PULLUP);
// Set interrupt, assign interrupt function and set HIGH mode
  attachInterrupt(digitalPinToInterrupt(aPin), aButton, HIGH);
  attachInterrupt(digitalPinToInterrupt(bPin), bButton, HIGH);
  attachInterrupt(digitalPinToInterrupt(cPin), cButton, HIGH);
  attachInterrupt(digitalPinToInterrupt(dPin), dButton, HIGH);

  
  //Set LED to LOW
   pinMode(ledDoor, OUTPUT);
   digitalWrite(ledDoor, LOW);
   pinMode(ledAccident, OUTPUT);
   digitalWrite(ledAccident, LOW);
   

  // relay
  pinMode(fuelControlWire, OUTPUT);
  digitalWrite(fuelControlWire, HIGH);


//acc
Serial.println("Accelerometer Test"); Serial.println("");
 
//Initialise the sensor
if(!accel.begin())
{
/* There was a problem detecting the ADXL345 ... check your connections */
Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
while(1);
}
 
/* Set the range to whatever is appropriate for your project */
accel.setRange(ADXL345_RANGE_16_G);
  
}

// loop
void loop()
{
  
  delay(100);
  if(carCrash){
    delay(5000);
    carCrash= false;
    if(carDoorOpen || (!safetyMode)){digitalWrite(fuelControlWire, LOW);}
    digitalWrite(ledAccident, LOW);
     Serial.println("car crash resolved :))))))))");
  }


//wifi-reconnect
     connecttowifi();
     
//get acc
    getEventNew();

  if (client.connected())
    client.loop();
  else {
    connecttowifi();
     setup_mqtt_client();
  }

  // publish every mqttNow secs
  mqttNow = millis();
  if (mqttNow - mqttLastUpdate >= mqttInterval)
  {
    // update temperature on app
    sprintf(mqttBuffer, mqttFormat,carDoorOpen,carFuel,carCrash,safetyMode);

    client.publish(sendingTopic, mqttBuffer);
  }

}
