#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.
char ssid[] = "Singh";
char pass[] = "12345678";
// Ensure that the credentials here allow you to publish and subscribe to the ThingSpeak channel.
#define channelID 2322752
const char mqttUserName[] = "LCALIzw2DwIbKyAJNhgsFC4";
const char clientID[] = "LCALIzw2DwIbKyAJNhgsFC4";
const char mqttPass[] = "mLP8Y0NcckBSFic2WK+JOoym";

// Parsed value variables globally declared
String raw_ldr = "";
String temperature = "";
String humidity = "";
String relay_status = "";

// Empty string for data buffering
String receivedData = "";

void uart_setup() {
  Serial.begin(115200);
  // Serial2.begin(115200);

}

void uart_loop() {
  while (Serial.available()) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n') {
      // Process the received line (e.g., parse values)
      if (receivedData.length() > 0) {
        // Split the received data by commas
        String csvData = receivedData;
        String values[5]; // Assuming you have 6 values in the CSV data

        for (int i = 0; i < 5; i++) {
          int commaPos = csvData.indexOf(',');
          if (commaPos != -1) {
            values[i] = csvData.substring(0, commaPos);
            csvData = csvData.substring(commaPos + 1);
          } else {
            values[i] = csvData; // The last value in the CSV data
          }
        }

        // Update the global variables directly
        raw_ldr = values[0];
        temperature = values[1];
        humidity = values[2];
        relay_status = values[3];
        String channel_id = values[4];

        // Use the parsed values as needed
        // Serial.println("ldr: " + raw_ldr);
        // Serial.println("temp: " + temperature);
        // Serial.println("humidity: " + humidity);
        // Serial.println("relay: " + relay_status);
        // Serial.println("channel: " + channel_id);
      }
      receivedData = ""; // Clear the buffer for the next line
    } else {
      // Append the character to the buffer
      receivedData += receivedChar;
    }
  }
}

#define mqttPort 1883
WiFiClient client;
const char *server = "mqtt3.thingspeak.com";
int status = WL_IDLE_STATUS;
long lastPublishMillis = 0;
int connectionDelay = 1;
int updateInterval = 15;
PubSubClient mqttClient(client);

void setup() {
  uart_setup();

  // Delay to allow serial monitor to come up.
  delay(3000);
  // Connect to Wi-Fi network.
  wifi_connect();
  // Configure the MQTT client
  mqttClient.setServer(server, mqttPort);
  // Set the MQTT message handler function.
  mqttClient.setCallback(mqttSubscriptionCallback);
  // Set the buffer to handle the returned JSON. NOTE: A buffer overflow of the message buffer will result in your callback not being invoked.
  mqttClient.setBufferSize(2048);
}

void loop() {
  uart_loop();
  // Reconnect to WiFi if it gets disconnected.
  if (WiFi.status() != WL_CONNECTED) {
    wifi_connect();
  }

  // Connect if MQTT client is not connected and resubscribe to channel updates.
  if (!mqttClient.connected()) {
    mqttConnect();
    mqttSubscribe(channelID); // SUBSCRIBE TO ALL FIELDS
  }

  // Call the loop to maintain connection to the server.
  mqttClient.loop();

  // Update ThingSpeak channel periodically. The update results in the message to the subscriber.
  if (abs(long(millis()) - lastPublishMillis) > updateInterval * 1000) {

    // if (!raw_ldr.equals("") && !temperature.equals("") && !humidity.equals("") && !relay_status.equals("")) {
      String message = "field1=" + String(raw_ldr) + "&field2=" + String(temperature) + "&field3=" + String(humidity) + "&field4=" + String(relay_status) + "&status=MQTTPUBLISH";
      mqttPublish(channelID, message);  
    lastPublishMillis = millis();
  // }
}
}
