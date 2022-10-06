// **********************************************************************************
// ESP32 Teleinfo2Modbus 
// **********************************************************************************

// **********************************************************************************
//#define DEBUG_MQTT
#define DEBUG_MODBUS
//#define DEBUG_LINKY

#include <WiFi.h>
#include <LibTeleinfo.h>
#include <jsonlib.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusIP_ESP8266.h>

//pinout pour le D1 Mini ESP32
#define TIC_RX_PIN      23
#define RGB_LED_PIN     18


#ifdef RGB_LED_PIN
#include <NeoPixelBus.h>

#define colorSaturation 128
// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(1, RGB_LED_PIN);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

#endif

const char* ssid = "MaisonGrosIOT";
const char* password = "LaclefwifiIOT!";
const uint ServerPort = 502;
const char* mqtt_server = "192.168.4.3";
const uint16_t mqtt_port = 1883;
const char* mqtt_topic = "Linky";
const char* mqtt_user = "mqtt";
const char* mqtt_password = "mqttpass";
const _Mode_e modeLinky = TINFO_MODE_STANDARD; // 0= TINFO_MODE_HISTORIQUE ; 1= TINFO_MODE_STANDARD
// global Modbus memory registers

ModbusIP mb;
RemoteDebug Debug;
#define WEBSOCKET_DISABLED true
WiFiClient espClient;
PubSubClient client(espClient);

// Voir https://www.enika.eu/data/files/produkty/energy%20m/CP/em24%20ethernet%20cp.pdf pour le détail des adresses et valeurs
const uint16_t constantesCompteur[][2]= { 
  { 0x000B, 1648 }, //Carlo Gavazzi identification code UIN 
  { 0x0302, 0x101E }, // Version and revision code of measurement module
  { 0x0304, 0x101E }, // Version and revision code of communication module
  { 0x1002, 3 }, // Measuring system  (3="1Ph", 4=“3P”)
  { 0x0032, 0 }, // Phase sequence 
  { 0x0033, 500 }, // Frequence
  { 0xA100, 0x1 }, //Front selector status 
  { 0x5000, 0x4c69 }, //Serial Digit  1 & 2
  { 0x5001, 0x6e6b }, //Serial Digit  3 & 4
  { 0x5002, 0x7930 }, //Serial Digit  5 & 6
  { 0x5003, 0x3030 }, //Serial Digit  7 & 8
  { 0x5004, 0x3030 }, //Serial Digit  9 & 10
  { 0x5005, 0x3030 }, //Serial Digit  11 & 12
  { 0x5006, 0x3100 }, //Serial Digit  13
  { 0xA000, 0x7 }, //Application : Doit être égal à 7 pour Victron
  //Infos triphasées non utilisées:
  { 0x0000, 0 }, // V-L2
  { 0x000C, 0 }, // A-L2
  { 0x0012, 0 }, // W-L2
  { 0x0042, 0 }, // kWh-L2
  { 0x0004, 0 }, // V-L2
  { 0x0010, 0 }, // A-L2
  { 0x0016, 0 }, // W-L2
  { 0x0044, 0 }, // kWh-L2
  //Initialisation des valeurs pertinentes pour le monophasé
  { 0x0002, 0 }, // V-L1
  { 0x000E, 0 }, // A-L1
  { 0x0012, 0 }, // W-L1
  { 0x0018, 0 }, // VA-L1
  { 0x0028, 0 }, // W-Total
  { 0x002A, 0 }, // VA-Total
  { 0x0040, 0 }, // kWh-L1
  { 0x0034, 0}, // kWh+
  { 0x004E, 0 }, // kWh-
  { 0x0046, 0 }, // kWh+ T1
  { 0x0048, 0 }, // kWh+ T2
  };


TInfo tinfo; // Teleinfo object

// Pour clignotement LED asynchrone
unsigned long blinkLed  = 0;
uint16_t blinkDelay= 0;

// Uptime timer
boolean tick1sec=0;// one for interrupt, don't mess with 
unsigned long uptime=0; // save value we can use in sketch even if we're interrupted

// Used to indicate if we need to send all date or just modified ones
boolean fulldata = true;

/* ======================================================================
Function: PublishIfAvailable
Purpose : Si le label est dispo dans le json publie la valeur (corrigé du ratio) sur Modbus à l'offset.
Input   : A json formatted string, a label string, a modbus offset and a correction factor
Output  : --
Comments: -
====================================================================== */
void PublishIfAvailable(String json1, String label, uint16_t offset, float ratio)
{
  String result = "";
  result = jsonExtract(json1, label); //Total kWh HC
  if (result != "") {
    uint16_t computedresult = result.toInt()*ratio;
    if (mb.Hreg(offset,computedresult)) {
      debugI("Publish %s on Modbus register %X , value : %u",label, offset,computedresult );
    } 
    else if (mb.addHreg(offset,computedresult))
    {
      debugI("Publish %s on Modbus register %X , value : %u",label, offset,computedresult );
    }
    else {
      debugE("NOT Published %s on Modbus register %X , value : %u",label, offset,computedresult );
    }
  }
}

/* ======================================================================
Function: sendJSON 
Purpose : dump teleinfo values on serial
Input   : linked list pointer on the concerned data
          true to dump all values, false for only modified ones
Output  : A json formatted string
Comments: -
====================================================================== */
String sendJSON(ValueList * me, boolean all)
{
  bool firstdata = true;
  String json = "";

  // Got at least one ?
  if (me) {
    // Json start
    json += F("{");

    if (all) {
      json += F("\"_UPTIME\":");
      json += uptime;
      firstdata = false;
    }

    // Loop thru the node
    while (me->next) {
      // go to next node
      me = me->next;

      // uniquement sur les nouvelles valeurs ou celles modifiées 
      // sauf si explicitement demandé toutes
      if ( all || ( me->flags & (TINFO_FLAGS_UPDATED | TINFO_FLAGS_ADDED) ) )
      {
        // First elemement, no comma
        if (firstdata)
          firstdata = false;
        else {
          json += F(", ");
        }

        json += F("\"");
        json += me->name;
        json += F("\":");

        // we have at least something ?
        if (me->value && strlen(me->value))
        {
          boolean isNumber = true;
          char * p = me->value;

          // check if value is number
          while (*p && isNumber) {
            if ( *p < '0' || *p > '9' )
              isNumber = false;
            p++;
          }
  
          // this will add "" on not number values
          if (!isNumber) {
            json += F("\"");
            json += me->value;
            json += F("\"");
          }
          // this will remove leading zero on numbers
          else {
            json += atol(me->value);
          }
        }
      }
    }
   // Json end
   json +=F("}");
  }
  #ifdef DEBUG_LINKY
  debugD("%s", json.c_str());
  #endif
  return json;  
}
 

/* ======================================================================
Function: PublishOnMQTT
Purpose : Diffuse les data sur MQTT
Input   : A json formatted string
Output  : --
Comments: -
====================================================================== */
void pubMQTTvalue(JsonString key, JsonVariant value)
{
  unsigned long s = value;
  #ifdef DEBUG_MQTT
  debugW("%s : %d",key.c_str(), s);
  #endif
  String topic = String();
  topic = String(mqtt_topic) + "/"; 
  topic = topic + String(key.c_str());
  client.publish(topic.c_str(), String(s).c_str());
}

void pubMQTTstring(JsonString key, JsonVariant value)
{
  const char* s = value;
  #ifdef DEBUG_MQTT
  debugW("%s : %s",key.c_str(), s);
  #endif
  String topic = String();
  topic = String(mqtt_topic) + "/"; 
  topic = topic + String(key.c_str());
  client.publish(topic.c_str(), s);
}

void pubModbusTCP(String label, int offset, uint16_t value)
{
  if (mb.Hreg(offset,value)) {
    #ifdef DEBUG_MODBUS
    debugI("%s published on Modbus register %X , value : %u",label, offset,value );
    #endif
  } else {
    #ifdef DEBUG_MODBUS
    debugE("%s NOT PUBLISHED on Modbus register %X , value : %u",label, offset,value);
    #endif
  }
}

void PublishOnMQTT(String json2)
{
  debugD("%s", json2.c_str());
  StaticJsonDocument<1536> doc;
  DeserializationError error = deserializeJson(doc, json2);

  #ifdef DEBUG_MQTT
  if (error) {
    debugE("deserializeJson() failed: %s", error.c_str());
    debugE("%s", json2);
    return;
  }
  #endif

  JsonObject obj = doc.as<JsonObject>();

  for (JsonPair p : obj) {
    String s = p.key().c_str();
    //{"_UPTIME":319500, "ADSC":2147483647, "VTIC":2, "NGTF":"H PLEINE/CREUSE ", "LTARF":" HEURE  PLEINE  ", "EAST":␛[0m3839865, "EASF01":1266150, "EASF02":2573715, "EASF03":0, "EASF04":0, "EASF05":0, "EASF06":0, "EASF07":0, "EASF08":0, "EASF09":0, "EASF10":0, "EASD01":3␛[0m839865, "EASD02":0, "EASD03":0, "EASD04":0, "IRMS1":3, "URMS1":233, "PREF":9, "PCOUP":9, "SINSTS":780, "SMAXSN":2910, "SMAXSN-1":5480, "CCASN":536, "CC␛[0mASN-1":1022, "UMOY1":230, "STGE":"00DA0401", "MSG1":"     PAS DE          MESSAGE    ", "PRM":2147483647, "RELAIS":0, "NTARF":2, "NJOURF":0, "NJOURF+1"␛[0m:0, "PJOURF+1":"0000C001 061E8002 161EC001 NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE"}

    if (s=="SINSTS")
    {
      pubMQTTvalue(p.key(), p.value());
      uint16_t value = p.value();
      uint16_t computedresult = value * 10;
      pubModbusTCP(s,0x0012,computedresult); //W L1
      pubModbusTCP(s,0x0018,computedresult); //VA L1
      pubModbusTCP(s,0x0028,computedresult); //W Total
      pubModbusTCP(s,0x002A,computedresult); //VA Total
    } 
    else if (s=="EAST")
    {
      unsigned long value = p.value();
      unsigned long computedresult = value / 100 ;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x0040,computedresult); //kWh+ L1
      pubModbusTCP(s,0x0034,computedresult); //kWh+ Tot
    }
    else if (s=="EAIT")
    {
      unsigned long value = p.value();
      unsigned long computedresult = value / 100 ;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x004E,computedresult); //kWh- Tot
    }
    else if (s=="EASF01")
    {
      unsigned long value = p.value();
      unsigned long computedresult = value / 100 ;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x0046,computedresult); //kWh+ T1
    }
    else if (s=="EASF02")
    {
      unsigned long value = p.value();
      unsigned long computedresult = value / 100 ;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x0048,computedresult); //kWh+ T2
    }
    else if (s=="UMOY1")
    {
      uint16_t value = p.value();
      uint16_t computedresult = value * 10;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x0000,computedresult); //V L1
    }
    else if (s=="IRMS1")
    {
      uint16_t value = p.value();
      uint16_t computedresult = value * 1000;
      pubMQTTvalue(p.key(), p.value());
      pubModbusTCP(s,0x000C,computedresult); //I L1
    }
    else 
    {
      debugV("Default");
      if (p.value().is<const char*>()) {
        pubMQTTstring(p.key(), p.value());
      } else {
        pubMQTTvalue(p.key(), p.value());
      }
    }

  }

}

/* ======================================================================
Function: JSON2Modbus
Purpose : Extract relevant data from Json and output to Modbus register
Input   : A json formatted string
Output  : --
Comments: -
====================================================================== */
void JSON2Modbus(String json)
{
  // Voir https://www.enika.eu/data/files/produkty/energy%20m/CP/em24%20ethernet%20cp.pdf pour le détail des adresses et valeurs coté ELM24
  //Voir https://www.enedis.fr/media/2035/download pour les label coté Linky
  struct LINKY2MODBUS{
  const char* label;
  int ModbusOffset;
  float ratio;
  } linky2modbus[] = {
// Variable pour mode historiques
//{"_UPTIME":60, "ADCO":2147483647, "OPTARIF":"HC..", "ISOUSC":45, "HCHC":1221514, "HCHP":2482864, "PTEC":"HP..", "IINST":3, "IMAX":90, "PAPP":920, "HHPHC":"A", "MOTDETAT":0}
    //Pour contrat HC/HP
    { "HCHC", 0x0048, 0.01 }, //Compteur Heures Creuses en Wh
    { "HCHP", 0x0046, 0.01 }, //Compteur Heures Pleines en Wh
    //Pour contrat EJP
    { "EJPHN", 0x0046, 0.01 }, //Compteur EJP Heures Normales en Wh
    { "EJPHPM", 0x0048, 0.01 }, //Compteur EJP Pointes en Wh
    //Pour contrat Tempo
    { "BBRHCJB", 0x0046, 0.01 }, //Compteur Bleu HC en Wh
    { "BBRHPJB", 0x0048, 0.01 }, //Compteur Bleu HP en Wh
    { "BBRHCJW", 0x004A, 0.01 }, //Compteur Blanc HC en Wh
    { "BBRHPJW", 0x004C, 0.01 }, //Compteur Blanc HP en Wh      
//    { "BBRHCJR", 0x0046, 0.01 }, //Compteur Rouge HC en Wh *** Malheuresement il n'y a que 4 index sur ELM24
//    { "BBRHPJR", 0x0048, 0.01 }, //Compteur Rouge HP en Wh *** Malheuresement il n'y a que 4 index sur ELM24   
    { "ADCO", 0x5000, 1}, //Identifiant Linky
    //Monophasé
    { "IINST", 0x000C, 1000}, //Intensité instantané en A 
    { "PAPP", 0x0018, 10}, //Puissance Apparente en VA
    // Triphasé
    { "IINST1", 0x000C, 1000}, //Intensité instantané en A - L1
    { "IINST2", 0x000E, 1000}, //Intensité instantané en A - L2
    { "IINST3", 0x0010, 1000}, //Intensité instantané en A - L3

// Variables pour mode Standard
//{"_UPTIME":319500, "ADSC":2147483647, "VTIC":2, "NGTF":"H PLEINE/CREUSE ", "LTARF":" HEURE  PLEINE  ", "EAST":␛[0m3839865, "EASF01":1266150, "EASF02":2573715, "EASF03":0, "EASF04":0, "EASF05":0, "EASF06":0, "EASF07":0, "EASF08":0, "EASF09":0, "EASF10":0, "EASD01":3␛[0m839865, "EASD02":0, "EASD03":0, "EASD04":0, "IRMS1":3, "URMS1":233, "PREF":9, "PCOUP":9, "SINSTS":780, "SMAXSN":2910, "SMAXSN-1":5480, "CCASN":536, "CC␛[0mASN-1":1022, "UMOY1":230, "STGE":"00DA0401", "MSG1":"     PAS DE          MESSAGE    ", "PRM":2147483647, "RELAIS":0, "NTARF":2, "NJOURF":0, "NJOURF+1"␛[0m:0, "PJOURF+1":"0000C001 061E8002 161EC001 NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE NONUTILE"}
    //{ "PRM", 0x5000, 1}, //Identifiant Linky
    { "EAST", 0x0034, 0.01}, //Energie active soutirée totale en Wh
    { "EASF01", 0x0046, 0.01 }, //Compteur EJP Heures Normales en Wh
    { "EASF02", 0x0048, 0.01 }, //Compteur EJP Pointes en Wh
    { "EASF03", 0x004A, 0.01 }, //Compteur EJP Heures Normales en Wh
    { "EASF04", 0x004C, 0.01 }, //Compteur EJP Pointes en Wh
    { "IRMS1", 0x000C, 1000}, //Intensité Efficace en A - L1
    { "IRMS2", 0x000E, 1000}, //Intensité Efficace en A - L2
    { "IRMS3", 0x0010, 1000}, //Intensité Efficace en A - L3
    { "UMOY1", 0x0000, 10}, //Tension en V - L1
    { "UMOY2", 0x0002, 10}, //Tension en V - L2
    { "UMOY3", 0x0004, 10}, //Tension en V - L3
    { "SINSTS", 0x002A, 10}, //Puissance Apparente instantanéé en VA
    { "SINSTS1", 0x0018, 10}, //Puissance Apparente instantanéé en VA - L1
    { "SINSTS2", 0x001A, 10}, //Puissance Apparente instantanéé en VA - L2
    { "SINSTS3", 0x001C, 10}, //Puissance Apparente instantanéé en VA - L2
    { "SINSTI", 0x002A, -10}, //Puissance Apparente instantanéé en VA
  // On triche un peu pour émuler le EM24:
    { "SINSTS", 0x0028, 10}, //Puissance Apparente instantanéé en W (au lieu de VA)
    { "SINSTS", 0x0012, 10}, //Puissance Apparente instantanéé en W (au lieu de VA)
    { "SINSTS1", 0x0012, 10}, //Puissance Apparente instantanéé en W (au lieu de VA) - L1
    { "SINSTS2", 0x0014, 10}, //Puissance Apparente instantanéé en W (au lieu de VA) - L2
    { "SINSTS3", 0x0016, 10}, //Puissance Apparente instantanéé en W (au lieu de VA) - L2
    { "SINSTI", 0x0028, -10}, //Puissance Apparente instantanéé en W (au lieu de VA)
    { "EAST", 0x0040, 0.01}, //Energie active soutirée totale en Wh (Phase1)
    { "EAIT", 0x004E, 0.01}, //Energie active injectée totale en Wh
  };
//{"_UPTIME":60, "ADCO":2147483647, "OPTARIF":"HC..", "ISOUSC":45, "HCHC":1221514, "HCHP":2482864, "PTEC":"HP..", "IINST":3, "IMAX":90, "PAPP":920, "HHPHC":"A", "MOTDETAT":0}
  
  for (byte i = 0; i< (sizeof(linky2modbus) / sizeof(linky2modbus[0])) ; i = i + 1) {
    PublishIfAvailable(json, linky2modbus[i].label, linky2modbus[i].ModbusOffset, linky2modbus[i].ratio);
  }
}

/* ======================================================================
Function: ADPSCallback 
Purpose : called by library when we detected a ADPS on any phased
Input   : phase number 
            0 for ADPS (monophase)
            1 for ADIR1 triphase
            2 for ADIR2 triphase
            3 for ADIR3 triphase
Output  : - 
Comments: should have been initialised in the main sketch with a
          tinfo.attachADPSCallback(ADPSCallback())
====================================================================== */
void ADPSCallback(uint8_t phase)
{
  // Envoyer JSON { "ADPS"; n}
  // n = numero de la phase 1 à 3
  if (phase == 0)
    phase = 1;
  #ifdef DEBUG_LINKY
  Debug.println(F("{\"ADPS\":"));
  Debug.println('0' + phase);
  Debug.println(F("}"));
  #endif
}

/* ======================================================================
Function: NewFrame 
Purpose : callback when we received a complete teleinfo frame
Input   : linked list pointer on the concerned data
Output  : - 
Comments: -
====================================================================== */
void NewFrame(ValueList * me)
{
  #ifdef RGB_LED_PIN
  strip.SetPixelColor(0, red);
  strip.Show();
  #endif
  blinkLed = millis();
  blinkDelay = 50; // 50ms

  // Envoyer les valeurs uniquement si demandé
  if (fulldata) {
    String jsonresult = sendJSON(me, true);
    #ifdef DEBUG_LINKY
    debugD("%s", jsonresult);
    #endif
//    JSON2Modbus(jsonresult);
    PublishOnMQTT(jsonresult);
  }
  fulldata = false;
}

/* ======================================================================
Function: UpdatedFrame 
Purpose : callback when we received a complete teleinfo frame
Input   : linked list pointer on the concerned data
Output  : - 
Comments: it's called only if one data in the frame is different than
          the previous frame
====================================================================== */
void UpdatedFrame(ValueList * me)
{
  #ifdef RGB_LED_PIN
  strip.SetPixelColor(0, blue);
  strip.Show();
  #endif

  blinkLed = millis();
  blinkDelay = 50; // 50ms

  // Envoyer les valeurs 
  String jsonresult2 = sendJSON(me, fulldata);
  #ifdef DEBUG_LINKY
  debugD("%s", jsonresult2);
  #endif
//  JSON2Modbus(jsonresult2);
  PublishOnMQTT(jsonresult2);
  fulldata = false;
}



/* ======================================================================
Function: connected_to_ap
Purpose : Call back functionCollect info about the connected wifi and output them on serial link
Input   : -
Output  : - 
Comments: -
====================================================================== */
void connected_to_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info){
    Serial.println("\nConnected to the WiFi network");
    #ifdef RGB_LED_PIN
    strip.SetPixelColor(0, green);
    strip.Show();
    blinkLed = millis();
    blinkDelay = 500; // 500ms
    #endif
}

/* ======================================================================
Function: start_modbus
Purpose : Démarre le Modbus
Input   : -
Output  : - 
Comments: -
====================================================================== */
void start_modbus(){
  mb.server(ServerPort);
  for (byte i = 0; i< (sizeof(constantesCompteur) / sizeof(constantesCompteur[0])) ; i = i + 1) {
    mb.addHreg(constantesCompteur[i][0],constantesCompteur[i][1]);
  }
}

/* ======================================================================
Function: initOTA
Purpose : 
Input   : -
Output  : - 
Comments: -
====================================================================== */

void initOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("ESP32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}

/* ======================================================================
Function: callback
Purpose : Call back function for MQTT
Input   : -
Output  : - 
Comments: -
====================================================================== */
void callback(char* topic, byte* message, unsigned int length) {
  Debug.print("Message arrived on topic: ");
  Debug.print(topic);
  Debug.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Debug.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Debug.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
  String String1 = mqtt_topic;
  String String2 = "/LED";
  String control_topic = String1 + String2 ;

  if (String(topic) == control_topic) {
    Debug.print("Changing output to ");
    if(messageTemp == "on"){
      Debug.println("on");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if(messageTemp == "off"){
      Debug.println("off");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
/* ======================================================================
Function: got_ip_from_ap
Purpose : Call back functionCollect info about the connected wifi and output them on serial link
Input   : -
Output  : - 
Comments: -
====================================================================== */
void got_ip_from_ap(WiFiEvent_t wifi_event, WiFiEventInfo_t wifi_info){
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
    start_modbus();
    initOTA();
    Debug.begin("ESP32"); 
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}



void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Debug.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Linky", mqtt_user, mqtt_password)) {
      Debug.println("connected");
      // Subscribe
      client.subscribe(mqtt_topic);
    } else {
      Debug.print("failed, rc=");
      Debug.print(client.state());
      Debug.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/* ======================================================================
Function: setup
Purpose : Setup I/O and other one time startup stuff
Input   : -
Output  : - 
Comments: -
====================================================================== */
void setup()
{
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true); // Colors
  // Serial, pour le debug
  Serial.begin(115200);
  Serial.println(F("\r\n\r\n=============="));
  Serial.println(F("Teleinfo"));

  // this resets all the neopixels to an off state
  #ifdef RGB_LED_PIN
  Serial.printf_P(PSTR("Enable WS2812 RGB LED on GPIO=%d\r\n"), RGB_LED_PIN);
  strip.Begin();
  strip.SetPixelColor(0, red);
  strip.Show();
  //On allume la led en rouge tant que le wifi n'est pas ok.

  #endif

  //Connexion au Wifi
  WiFi.mode(WIFI_STA); //Optional
  //On active les fonctions de callback qui s'active une fois le wifi OK
  WiFi.onEvent(connected_to_ap, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(got_ip_from_ap, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  // Configure Teleinfo Soft serial 
  // La téléinfo est connectee sur D3
  // ceci permet d'eviter les conflits avec la 
  // vraie serial lors des uploads
  Serial1.begin(9600, SERIAL_7E1, TIC_RX_PIN);
  pinMode(TIC_RX_PIN, INPUT_PULLUP);

  // Init teleinfo
  tinfo.init(modeLinky);

  // Attacher les callback dont nous avons besoin
  // pour cette demo, ADPS et TRAME modifiée
  tinfo.attachADPS(ADPSCallback);
  tinfo.attachUpdatedFrame(UpdatedFrame);
  tinfo.attachNewFrame(NewFrame); 
  
}

/* ======================================================================
Function: loop
Purpose : infinite loop main code
Input   : -
Output  : - 
Comments: -
====================================================================== */
void loop()
{
  ArduinoOTA.handle();
  static char c;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  // Avons nous recu un ticker de seconde?
  if (tick1sec)
  {
    tick1sec = false;
    uptime++;

    // Forcer un envoi de trame complète toutes les minutes
    // fulldata sera remis à 0 après l'envoi
    if (uptime % 60 == 0)
      fulldata = true;
  }
  
  //// On a reçu un caractère ?
  if ( Serial1.available() ) {
    // Le lire
    c = Serial1.read();
  
    // Gérer
    tinfo.process(c);
  
    // L'affcher dans la console
  //  if (c!=TINFO_STX && c!=TINFO_ETX) {
  //    Serial.print(c);
  //  }
  }

  // Verifier si le clignotement LED doit s'arreter 
  if (blinkLed && ((millis()-blinkLed) >= blinkDelay))
  {
    #ifdef RGB_LED_PIN
    strip.SetPixelColor(0, black);
    strip.Show();
    #endif

    blinkLed = 0;
  }

  if (currentMillis - previousMillis > 1000 ) {
    // save the last time you blinked the LED 
    previousMillis = currentMillis;   
    tick1sec = true;
  }

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  mb.task();
  Debug.handle();

}