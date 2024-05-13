// chương trình cho 1 AGV
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>
#include "Value.h"
#include "Config.h"


WiFiClient espClient;
PubSubClient client(espClient);
//************************************
const uint16_t mqttBufferSize = 4096;
char mqttBuffer[mqttBufferSize];


// *****<CẤU HÌNH CHO TIÊU ĐỀ PHỤ>*****
Ordermessage order;
Factsheet factsheet;
Connection connection;
State state;
Visualization visualization;
InstantActions instantActions;
//*************************************

//*******<CHƯƠNG TRÌNH CÀI ĐẶT>********
void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setClient(espClient);
}

//******<lấy các thông số từ AVG>******
void thongSoKetNoi(){
  connection.headerId=1;
  connection.timestamp="1991-03-11T11:40:03.12Z";
  connection.version = "1.3.2";
  connection.manufacturer="Manufacturer";
  connection.serialNumber= "SerialNumber";
  connection.connectionState= ONLINE;
}
//***************

//********<chương trình nhận bản tin từ master>********
void receiveOrderMessage(DynamicJsonDocument docOrder){
  order.headerId = docOrder["headerId"];
  Serial.println(order.headerId);
  order.timestamp = docOrder["timestamp"].as<String>();
  Serial.println(order.timestamp);
  order.version = docOrder["version"].as<String>();
  Serial.println(order.version);
  order.manufacturer = docOrder["manufacturer"].as<String>();
  Serial.println(order.manufacturer);
  order.orderId = docOrder["orderId"].as<String>();
  Serial.println(order.orderId);
  order.orderUpdateId = docOrder["orderUpdateId"];
  Serial.println(order.orderUpdateId);
  order.zoneSetId = docOrder["zoneSetId"].as<String>();
  Serial.println(order.zoneSetId);
  int i=0;
  for(JsonObject objnodes: docOrder["nodes"].as<JsonArray>()){
    order.nodes[i].nodeId = docOrder["nodes"][i]["nodeId"].as<String>();
    Serial.println(order.nodes[i].nodeId);
    order.nodes[i].sequenceId = docOrder["nodes"][i]["sequenceId"];
    Serial.println(order.nodes[i].sequenceId);
    order.nodes[i].nodeDescription = docOrder["nodes"][i]["nodeDescription"].as<String>();
    Serial.println(order.nodes[i].nodeDescription);
    order.nodes[i].released = docOrder["nodes"][i]["released"].as<bool>();
    Serial.println(order.nodes[i].released);
    JsonObject objnodePosition = docOrder["nodes"][i]["nodePosition"];
    order.nodes[i].nodePosition.x = objnodePosition["x"];
    Serial.println(order.nodes[i].nodePosition.x);
    order.nodes[i].nodePosition.y = objnodePosition["y"];
    Serial.println(order.nodes[i].nodePosition.y);
    order.nodes[i].nodePosition.theta = objnodePosition["theta"];
    Serial.println(order.nodes[i].nodePosition.theta);
    order.nodes[i].nodePosition.allowedDeviationXY = objnodePosition["allowedDeviationXY"];
    Serial.println(order.nodes[i].nodePosition.allowedDeviationXY);
    order.nodes[i].nodePosition.allowedDeviationTheta = objnodePosition["allowedDeviationTheta"];
    Serial.println(order.nodes[i].nodePosition.allowedDeviationTheta);
    order.nodes[i].nodePosition.mapId = objnodePosition["mapId"].as<String>();
    Serial.println(order.nodes[i].nodePosition.mapId);
    order.nodes[i].nodePosition.mapDescription = objnodePosition["mapDescription"].as<String>();
    Serial.println(order.nodes[i].nodePosition.mapDescription);
    int j=0;
    for(JsonObject objaction: docOrder["nodes"][i]["action"].as<JsonArray>()){
      order.nodes[i].action[j].actionType = docOrder["nodes"][i]["action"][j]["actionType"].as<String>();
      Serial.println(order.nodes[i].action[j].actionType);
      order.nodes[i].action[j].actionId = docOrder["nodes"][i]["action"][j]["actionId"].as<String>();
      Serial.println(order.nodes[i].action[j].actionId);
      order.nodes[i].action[j].actionDescription = docOrder["nodes"][i]["action"][j]["actionDescription"].as<String>();
      Serial.println(order.nodes[i].action[j].actionDescription);
      order.nodes[i].action[j].blockingType = docOrder["nodes"][i]["action"][j]["blockingType"];
      Serial.println(order.nodes[i].action[j].blockingType);
      j++;
    }
    i++;
  }
  int m=0;
  for(JsonObject objedges: docOrder["edges"].as<JsonArray>()){
    order.edges[m].edgeId = docOrder["edges"][m]["edgeId"].as<String>();
    Serial.println(order.edges[m].edgeId);
    order.edges[m].sequenceId = docOrder["edges"][m]["sequenceId"];
    Serial.println(order.edges[m].sequenceId);
    order.edges[m].edgeDescription = docOrder["edges"][m]["edgeDescription"].as<String>();
    Serial.println(order.edges[m].edgeDescription);
    order.edges[i].released = docOrder["edges"][i]["released"].as<bool>();
    Serial.println(order.edges[i].released);
    order.edges[m].startNodeId = docOrder["edges"][m]["startNodeId"].as<String>();
    Serial.println(order.edges[m].startNodeId);
    order.edges[m].endNodeId = docOrder["edges"][m]["endNodeId"].as<String>();
    Serial.println(order.edges[m].endNodeId);
    order.edges[m].maxSpeed = docOrder["edges"][m]["maxSpeed"];
    Serial.println(order.edges[m].maxSpeed);
    order.edges[m].maxHeight = docOrder["edges"][m]["maxHeight"];
    Serial.println(order.edges[m].maxHeight);
    order.edges[m].minHeight = docOrder["edges"][m]["minHeight"];
    Serial.println(order.edges[m].minHeight);
    m++;
  }
  
}

void receiveInstantActionsMessage(DynamicJsonDocument docInstantAtions){
  //instantActions.headerId = docInstantActions["headerId"];
  //Serial.println(instantActions.headerId);
}
//*****************************************************

//******<Chương trình kiểm tra orderMesseage>**********
void checkOrderMesseage(){
  if(order.orderId != ORDERIDOFAGV){

  }else{
    if(order.orderUpdateId>=order.olderOrderUpdateId){

    }else{
      

    }
  }
}

//*****************************************************
void sendConnection(){
  
    thongSoKetNoi();
    DynamicJsonDocument docConnection(1024);
    docConnection["$schema"] =  "https://json-schema.org/draft/2020-12/schema";
    docConnection["title"] = "connection";
    docConnection["headerId"] = connection.headerId;
    docConnection["timestamp"] = connection.timestamp;
    docConnection["version"] = connection.version;
    docConnection["manufacturer"] = connection.manufacturer;
    docConnection["serialNumber"] = connection.serialNumber;
    docConnection["connectionState"] = connection.connectionState;
    char output[1024];
    serializeJson(docConnection, output);
    Serial.println(output);
    client.publish("connect", output);
}

//*****************

//*******<VÍ DỤ CHO FACTSHEET>********


//*******<đăng kí topic factsheet>*******
void sendFactsheet(){
  DynamicJsonDocument docFactsheet(4098);
  docFactsheet["$schema"] =  "https://json-schema.org/draft/2020-12/schema";
  docFactsheet["title"] = "factsheet";
  docFactsheet["headerId"] = factsheet.headerId;
  docFactsheet["timestamp"] = factsheet.timestamp;
  docFactsheet["version"] = factsheet.version;
  docFactsheet["manufacturer"] = factsheet.manufacturer;
  docFactsheet["serialNumber"] = factsheet.serialNumber;
  docFactsheet["localizationParameters"] =factsheet.localizationParameters;
  JsonObject typeSpecObj = docFactsheet.createNestedObject("typeSpecification");
  typeSpecObj["seriesName"] = factsheet.typeSpecification.seriesName;
  typeSpecObj["seriesDescription"] = factsheet.typeSpecification.seriesDescription;
  typeSpecObj["agvKinematic"] = factsheet.typeSpecification.agvKinematic;
  typeSpecObj["agvClass"] = factsheet.typeSpecification.agvClass;
  typeSpecObj["maxLoadMass"] = factsheet.typeSpecification.maxLoadMass;
  JsonObject physParamObj = docFactsheet.createNestedObject("physicalParameters");
  physParamObj["speedMin"]=factsheet.physicalParameters.speedMin;
  physParamObj["speedMax"]=factsheet.physicalParameters.speedMax;
  physParamObj["accelerationMax"]=factsheet.physicalParameters.accelerationMax;
  physParamObj["decelerationMax"]=factsheet.physicalParameters.decelerationMax;
  physParamObj["heightMin"]=factsheet.physicalParameters.heightMin;
  physParamObj["heightMax"]=factsheet.physicalParameters.heightMax;
  physParamObj["width"]=factsheet.physicalParameters.width;
  physParamObj["length"]=factsheet.physicalParameters.length;
  JsonObject protLimitObj = docFactsheet.createNestedObject("protocolLimits");
  JsonObject maxStrLens = protLimitObj.createNestedObject("maxStringLens");
  maxStrLens["msgLen"]=factsheet.protocolLimits.maxStringLens.msgLen;
  maxStrLens["topicSerialLen"]=factsheet.protocolLimits.maxStringLens.topicSerialLen;
  maxStrLens["topicElemLen"]=factsheet.protocolLimits.maxStringLens.topicElemLen;
  maxStrLens["idLen"]=factsheet.protocolLimits.maxStringLens.idLen;
  maxStrLens["idNumericalOnly"]=factsheet.protocolLimits.maxStringLens.idNumericalOnly;
  maxStrLens["enumLen"]=factsheet.protocolLimits.maxStringLens.enumLen;
  maxStrLens["loadIdLen"]=factsheet.protocolLimits.maxStringLens.loadIdLen;
  JsonObject maxArrLens = protLimitObj.createNestedObject("maxArrayLens"); 
  maxArrLens["order.nodes"]=factsheet.protocolLimits.maxArrayLens.order_nodes;
  maxArrLens["order.edges"]=factsheet.protocolLimits.maxArrayLens.order_edges;
  maxArrLens["node.actions"]=factsheet.protocolLimits.maxArrayLens.node_actions;
  maxArrLens["edge.actions"]=factsheet.protocolLimits.maxArrayLens.edge_actions;
  maxArrLens["actions.actionsParameters"]=factsheet.protocolLimits.maxArrayLens.actions_actionsParameters;
  maxArrLens["instantActions"]=factsheet.protocolLimits.maxArrayLens.instantActions;
  maxArrLens["trajectory.knotVector"]=factsheet.protocolLimits.maxArrayLens.trajectory_knotVector;
  maxArrLens["trajectory.controlPoints"]=factsheet.protocolLimits.maxArrayLens.trajectory_controlPoints;
  maxArrLens["state.nodeStates"]=factsheet.protocolLimits.maxArrayLens.state_nodeStates;
  maxArrLens["state.edgeStates"]=factsheet.protocolLimits.maxArrayLens.state_edgeStates;
  maxArrLens["state.loads"]=factsheet.protocolLimits.maxArrayLens.state_loads;
  maxArrLens["state.actionStates"]=factsheet.protocolLimits.maxArrayLens.state_actionStates;
  maxArrLens["state.errors"]=factsheet.protocolLimits.maxArrayLens.state_errors;
  maxArrLens["state.information"]=factsheet.protocolLimits.maxArrayLens.state_information;
  maxArrLens["error.errorReferences"]=factsheet.protocolLimits.maxArrayLens.error_errorReferences;
  maxArrLens["information.infoReferences"]=factsheet.protocolLimits.maxArrayLens.information_infoReferences;
  JsonObject timing = protLimitObj.createNestedObject("timing");
  timing["minOrderInterval"]=factsheet.protocolLimits.timing.minOrderInterval;
  timing["minStateInterval"]=factsheet.protocolLimits.timing.minStateInterval;
  timing["defaultStateInterval"]=factsheet.protocolLimits.timing.defaultStateInterval;
  timing["visualizationInterval"]=factsheet.protocolLimits.timing.visualizationInterval;
  JsonObject protFeatuObj = docFactsheet.createNestedObject("agvProtocolFeatures");
  JsonArray optPara = protFeatuObj.createNestedArray("optionalParameters");
  for(int i=0;i<10;i++){
    JsonObject optParaObj = optPara.createNestedObject();
    optParaObj["parameter"]=factsheet.protocolFeatures.optionalParameters[i].parameter;
    optParaObj["support"]=factsheet.protocolFeatures.optionalParameters[i].support;
    optParaObj["description"]=factsheet.protocolFeatures.optionalParameters[i].description;
  }
  JsonArray agvAc = protFeatuObj.createNestedArray("agvActions");
  

  //JsonObject typeSpecObj = docFactsheet.createNestedObject("typeSpecification");
  //JsonObject typeSpecObj = docFactsheet.createNestedObject("typeSpecification");
  //JsonObject typeSpecObj = docFactsheet.createNestedObject("typeSpecification");
  char output[4098];
  serializeJson(docFactsheet, output);
  client.publish("factsheet", output,true);
}

void viDuThongSoState(){
  state.headerId++;
  state.timestamp = "1991-03-11T11:40:03.12Z";
  state.version = "1.3.2";
  state.manufacturer = "Manufacturer";
  state.serialNumber= "SerialNumber";
  state.orderId = "123";
  state.orderUpdateId = 0;
  state.zoneSetId = "123";
  state.lastNodeId = "123";
  state.lastNodeSequenceId = 123;
  state.driving = true;
  state.paused = true;
  state.newBaseRequest = true;
}

void sendState(){
  DynamicJsonDocument docState(1024);
  docState["$schema"] =  "https://json-schema.org/draft/2020-12/schema";
  docState["title"] = "state";
  docState["headerId"] = state.headerId++;
  docState["timestamp"] = state.timestamp;
  docState["version"] = state.version;
  docState["manufacturer"] = state.manufacturer;
  docState["serialNumber"] = state.serialNumber;
  docState["orderId"] = state.orderId;
  docState["orderUpdateId"] = state.orderUpdateId;
  docState["zoneSetId"] = state.zoneSetId;
  docState["lastNodeId"] = state.lastNodeId;
  docState["lastNodeSequenceId"] = state.lastNodeSequenceId;
  docState["driving"] = state.driving;
  docState["paused"] = state.paused;
  docState["newBaseRequest"] = state.newBaseRequest;
  char output1[1024];
  serializeJson(docState, output1);
  Serial.println(output1);
  client.publish("state", output1);
}

void upDateMessage(){

}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
  Serial.print((char)payload[i]);
  }
  Serial.println();
  if(strcmp(topic, "order") ==0){
    DynamicJsonDocument docOrder(4096);
    deserializeJson(docOrder, payload ,length);
    receiveOrderMessage(docOrder);
  }
  if(strcmp(topic, "instantAtions") ==0){
    DynamicJsonDocument docInstantActions(2048);
    deserializeJson(docInstantActions, payload ,length);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_client_id)) {
      client.subscribe("order");
      client.subscribe("instantActions");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //receiveOrderMessage(payload,length);
  sendConnection();
  delay(5000);
}
