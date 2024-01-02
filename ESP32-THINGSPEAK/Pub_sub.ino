// // Function to handle messages from MQTT subscription.

void mqttSubscriptionCallback( char* topic, byte* payload, unsigned int length ) {
  // Print the details of the message that was received to the serial monitor.
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
// //------------------------------------------------------FOR A PARTICULAR FIELD-------------------------------------------------------//
// Subscribe to ThingSpeak channel for updates
// void mqttSubscribe(long subChannelID, int fieldNumber) {
//   String myTopic = "channels/" + String(subChannelID) + "/subscribe/fields/field" + String(fieldNumber);
//   mqttClient.subscribe(myTopic.c_str());
// }
// //------------------------------------------------------FOR ALL FIELDS---------------------------------------------------------------//
// Subscribe to ThingSpeak channel for updates.
void mqttSubscribe( long subChannelID ){
  String myTopic = "channels/"+String( subChannelID )+"/subscribe";
  mqttClient.subscribe(myTopic.c_str());
}
// Publish messages to a ThingSpeak channel//
void mqttPublish(long pubChannelID, String message) {
  String topicString = "channels/" + String(pubChannelID) + "/publish";
  mqttClient.publish(topicString.c_str(), message.c_str());
}

// --------------------------------------------------------FOR MULTIPLE FIELDS--------------------------------------------------------//
void publishToThingSpeak(int channel, int fieldNumber, String data) {
  String topic = "channels/" + String(channelID) + "/publish/fields/field" + String(fieldNumber);
  mqttClient.publish(topic.c_str(), data.c_str());
}