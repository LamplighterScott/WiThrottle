  /*
 * Truncated JMRI WiThrottle server implementation for DCC++ command station: GPIO Outputs instead of encoded accessory signals
 * Software version 0.02 untested
 * 
 * Marklin Z 1700mm x 600 layout, four engines with decoders, 12v track power
 * Wemos Mega+ESP8266, Motor Shield, GPIO's connected to Darlington arrays (ULN2803 x3)
 * DCC++ SW adapted for momentary (20ms) GPIO HIGH signal.  Throw and Close signals use one GPIO each per turnout, starting with GPIO 22.  Sketch automatically selects even GPIO number for close signal and odd GPIO number for throw signal.
 * 
 * Change log:
 * 2018-04-9  - Cloned vhar/withrottle v1.02b
 * 2018-04-15 - Adapted for GPIO Outputs-only, used with a Marklin Z set, no accessory encoding
 *              loadAccessories() ==> loadOutputs()
 *              accessoryToggle() ==> outputToggle()
 *              throttleStart()
 * 
 * Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 * JMRI WiThrottle DCC++ ESP8266 https://github.com/vhar/withrottle v1.02b
 * DCC++ https://github.com/DccPlusPlus
 * ESP8266 Core https://github.com/esp8266/Arduino
 * JMRI WiFi Throttle Communications Protocol http://jmri.sourceforge.net/help/en/package/jmri/jmrit/withrottle/Protocol.shtml
 * WiThrottle official site http://www.withrottle.com/WiThrottle/Home.html
 * Download WiThrottle on the AppStore https://itunes.apple.com/us/app/withrottle-lite/id344190130
 * Engine Driver official site https://enginedriver.mstevetodd.com/
 * Download Engine Driver on the GooglePlay https://play.google.com/store/apps/details?id=jmri.enginedriver
 * 
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Create a WiFi access point. */
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include "Config.h"
#define maxCommandLength 30

/* Define Outputs object structures */
typedef struct {  
  int id;  // system id number w/o system and type chars
  int pin;  // Arduino GPIO number
  const char *userName;  // any string for name of output
  const char *type;  // T = turnout, C = decoupler, F = semaphore
  int iFlag;  // See DCC++ Outlets for bits explanation
  int status;  // 0 = unknown, 2 = closed, 4 = thrown 
  int present;  // Exists in DCC++ data file or not: 0 = no, 1 = yes; automatically checked and populated at start-up; leave as 0
} tData;
tData tt[]= {
  {1, 22, "To outer", "T", 0, 0, 0},
  {2, 24, "To inner", "T", 0, 0, 0},
  {3, 26, "Yard 1", "T", 0, 0, 0},
  {4, 28, "Yard 1-1", "T", 0, 0, 0},
  {5, 30, "Outer to Yard 2", "T", 0, 0, 0},
  {6, 32, "Inner Cross to Yard 2", "T", 0, 0, 0},
  {7, 34, "Yard 2-2", "T", 0, 0, 0},
  {8, 36, "Sidetrack", "T", 0, 0, 0},
  {17, 38, "Decoupler 1A", "C", 0, 0, 0},
  {18, 40, "Decoupler 1B", "C", 0, 0, 0},
  {19, 42, "Decoupler 2A", "C", 0, 0, 0},
  {20, 44, "Decoupler 2B", "C", 0, 0, 0},
  {25, 46, "Semaphore", "F", 0, 0, 0},
  {0, 0, "", "", 0, 0, 0}
  };
const int totalOutputs = 13;

/* The interval of check connections between ESP & WiThrottle app */
const int heartbeatTimeout = 10;
boolean heartbeatEnable[maxClient];
unsigned long heartbeat[maxClient*2];

String LocoThrottle[]={"","","","","",""};
int LocoState[maxClient*2][31];
int fKey;
int Throttle =0;
int start;
int finish;
int last;
char commandString[maxCommandLength+1];
boolean alreadyConnected[maxClient];
String powerStatus;
char msg[16];
int status = WL_IDLE_STATUS;

/* Define WiThrottle Server */
WiFiServer server(WTServer_Port);
WiFiClient client[maxClient];

void setup() {
  delay(1500);
  Serial.begin(115200);
  Serial.flush();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(WTServer_Ip, WTServer_Ip, WTServer_NMask);
  WiFi.softAP(ssid, password);

  MDNS.begin(hostString);
  server.begin();
  MDNS.addService("withrottle","tcp", WTServer_Port);
  if (PowerOnStart == 1)
    turnPowerOn();
  else
    turnPowerOff();
    
  
}

void loop() {
  for(int i=0; i<maxClient; i++){
    if (!client[i]) {
      client[i] = server.available();
    }
    else {
      if (client[i].status() == CLOSED) {
        throttleStop(i);
      }
      else if(!alreadyConnected[i]) {
        loadOutputs();
        throttleStart(i);
      }
    }
    if (client[i].available()) {
      String Data = client[i].readString();
      start = -1;
      last = Data.lastIndexOf('\n');
      String changeSpeed[]={"","","","","",""};
      while(start < last){
        finish = Data.indexOf('\n', start+1);
        String clientData = Data.substring(start+1, finish);
        start = finish;
        if (clientData.startsWith("*+")) {
          heartbeatEnable[i] = true;
        }
        else if (clientData.startsWith("PPA")){
          powerStatus = clientData.substring(3);
          Serial.println("<"+powerStatus+">");
          client[i].println("PPA"+powerStatus);
          for(int p=0; p<maxClient; p++){
            if (alreadyConnected[p]){
              client[p].println("PPA"+powerStatus);
            }
          }
        }
        else if (clientData.startsWith("PTA")){
          // "PTA"+status(single digit int)+system(single char)+type(single char)+id(int)
          // ie "PTA1DT12"
          String aStatus = clientData.substring(3,4);
          int aID = clientData.substring(6).toInt();
          outputToggle(aID, aStatus);
        }
        else if (clientData.startsWith("N") || clientData.startsWith("*")){
          client[i].println("*" + String(heartbeatTimeout));
        }
        else if (clientData.startsWith("MT") || clientData.startsWith("MS")) {
          String th = clientData.substring(1,2);
          if (th == "T")
            Throttle = 0+i*2;
          else
            Throttle = 1+i*2;
          String action = clientData.substring(2,3);
          String actionData = clientData.substring(3);
          int delimiter = actionData.indexOf(";");
          String actionKey = actionData.substring(0, delimiter-1);
          String actionVal = actionData.substring(delimiter+2);
          if (action == "+") {
            locoAdd(th, actionKey, i);
          }
          else if (action == "-") {
            locoRelease(th, actionKey, i);
          }
          else if (action == "A") {
            if(actionVal.startsWith("V")){
              changeSpeed[Throttle]=th;
              fKey=actionVal.substring(1).toInt();
              LocoState[Throttle][29]=fKey;
            }
            else {
              locoAction(th, actionKey, actionVal, i);
            }
          }
          heartbeat[Throttle]=millis();
        }
      }
      for(Throttle=0+i*2;Throttle<2+i*2;Throttle++){
        if(changeSpeed[Throttle]!="" && LocoThrottle[Throttle]!=""){
          String locoAddress=LocoThrottle[Throttle].substring(1);
          fKey=LocoState[Throttle][29];
          Serial.print("<t "+String(Throttle+1)+" "+locoAddress+"  "+String(fKey)+" "+String(LocoState[Throttle][30])+">");
          String response = loadResponse();
        }
      }
      if (heartbeatEnable[i]) {
        checkHeartbeat(i);
      }
    }
  }
}

int invert(int value){
  if(value == 0)
    return 1;
  else
    return 0;
}

void turnPowerOn() {
  powerStatus = "1";
  while (Serial.available() <= 0) {
    Serial.println("<1>");
    delay(300);
  }
  char c;
  while(Serial.available()>0)
  {
    c=Serial.read();
  }
}

void turnPowerOff() {
  powerStatus = "0";
  while (Serial.available() <= 0) {
    Serial.println("<0>");
    delay(300);
  }
  char c;
  while(Serial.available()>0)
  {
    c=Serial.read();
  }
}

String loadResponse() {
  while (Serial.available() <= 0) {
    delay(300);
  }
  char c;
  while(Serial.available()>0) {
    c=Serial.read();
    if(c=='<')
      sprintf(commandString,"");
    else if(c=='>')
      return (char*)commandString;
    else if(strlen(commandString)<maxCommandLength)
      sprintf(commandString,"%s%c",commandString,c);
  }
}

void loadOutputs() { // Accessories/Outputs
  Serial.write("<Z>");  // Get the list of Outputs in DCC++ 
  char c;
  while (Serial.available() <= 0)
  {
    delay(300);
  }
  // int t=0;
  while(Serial.available()>0)
  {
    c=Serial.read();
    if(c=='<')
      sprintf(commandString,"");
    else if(c=='>')
    {
      String s;
      char *str=(char*)commandString;
      char * pch;

      // ID
      pch = strtok(str, " ");
      s = (char*)pch;
      int id = s.substring(1).toInt();

      // Pin
      pch = strtok(str, " ");

      // IFLAG
      pch = strtok (NULL, " ");
      s = (char*)pch;
      int i = s.toInt();

      // Status
      pch = strtok (NULL, " ");
      s = (char*)pch;
      int z=2;  // Closed
      if(s=="1") z=4;  //  Thrown

      // Find output from DCC++ output table created by loadOutputs()
      int t = 0;
      for (t = 0 ; t<totalOutputs && tt[t].id!=id; t++);
      if (tt[t].id!=0) {
        tt[t].present = 1;
        // tt[t].iFlag = iFlag;
        tt[t].status = z;
      }

      // tt[t]={id, i, z};  // id, iFlag, status
      // t++;
    }
    else if(strlen(commandString)<maxCommandLength)
      sprintf(commandString,"%s%c",commandString,c);
  }

  // Upload missing Outputs to DCC++
  int t=0;
  for (t=0; t<totalOutputs; t++) {
    if (tt[t].present<1 && tt[t].id!=0) {
      Serial.print("<Z "+String(tt[t].id)+" "+String(tt[t].pin)+" "+String(tt[t].iFlag)+">");
      String response = loadResponse();
      
    }
  }
  // Have DCC++ save settings
  Serial.print("<E>");
  String response = loadResponse();

}

void throttleStart(int i) {
  client[i].flush();
  client[i].setTimeout(500);
  Serial.println("\nNew client");
  client[i].println("VN2.0");
  client[i].println("RL0");
  client[i].println("PPA"+powerStatus);
  client[i].println("PTT]\\[Turnouts}|{Turnout]\\[Closed}|{2]\\[Thrown}|{4");
  client[i].print("PTL");
    for (int t = 0 ; t<totalOutputs; t++) {
      //  System Name, User Name, Status
      // System Name = "D" + type(single char: T, C or F) + id number
      if (tt[t].id!=0) {
        client[i].print("]\\[D"+String(tt[t].type)+String(tt[t].id)+"}|{"+String(tt[t].userName)+"}|{"+tt[t].status);
      }
    }  
  client[i].println("");
  client[i].println("*"+String(heartbeatTimeout));
  alreadyConnected[i] = true;
}

void throttleStop(int i) {
  client[i].stop();
  Serial.println("\n");
  alreadyConnected[i] = false;
  heartbeatEnable[i] = false;
  LocoState[0+i*2][29] = 0;
  heartbeat[0+i*2] = 0;
  LocoState[1+i*2][29] = 0;
  heartbeat[1+i*2] = 0;
}

void locoAdd(String th, String actionKey, int i) {
  LocoThrottle[Throttle] = actionKey;
  client[i].println("M"+th+"+"+actionKey+"<;>");
  for(fKey=0; fKey<29; fKey++){
    LocoState[Throttle][fKey] =0;
    client[i].println("M"+th+"A"+actionKey+"<;>F0"+String(fKey));
  }
  LocoState[Throttle][29] =0;
  LocoState[Throttle][30] =1;
  client[i].println("M"+th+"+"+actionKey+"<;>V0");
  client[i].println("M"+th+"+"+actionKey+"<;>R1");
  client[i].println("M"+th+"+"+actionKey+"<;>s1");
}

void locoRelease(String th, String actionKey, int i) {
  String locoAddress = LocoThrottle[Throttle].substring(1);
  heartbeat[Throttle] =0;
  LocoThrottle[Throttle] = "";
  Serial.print("<t "+String(Throttle+1)+" "+locoAddress+" 0 "+String(LocoState[Throttle][30])+">");
  String response = loadResponse();
  client[i].println("M"+th+"-"+actionKey+"<;>");

}

void locoAction(String th, String actionKey, String actionVal, int i){
  String locoAddress = LocoThrottle[Throttle].substring(1);
  String response;
  if(actionKey == "*"){
    actionKey = LocoThrottle[Throttle];
  }
  if(actionVal.startsWith("F1")){
    fKey = actionVal.substring(2).toInt();
    LocoState[Throttle][fKey] = invert(LocoState[Throttle][fKey]);
    client[i].println("M"+th+"A"+LocoThrottle[Throttle]+"<;>" + "F"+String(LocoState[Throttle][fKey])+String(fKey));
    byte func;
    switch(fKey){
      case 0:
      case 1:
      case 2:
      case 3:
      case 4:
        func = 128+LocoState[Throttle][1]*1+LocoState[Throttle][2]*2+LocoState[Throttle][3]*4+LocoState[Throttle][4]*8+LocoState[Throttle][0]*16;
        Serial.println("<f "+locoAddress+" "+String(func)+">");
      break;
      case 5:
      case 6:
      case 7:
      case 8:
        func = 176+LocoState[Throttle][5]*1+LocoState[Throttle][6]*2+LocoState[Throttle][7]*4+LocoState[Throttle][8]*8;
        Serial.println("<f "+locoAddress+" "+String(func)+">");
      break;
      case 9:
      case 10:
      case 11:
      case 12:
        func = 160+LocoState[Throttle][9]*1+LocoState[Throttle][10]*2+LocoState[Throttle][11]*4+LocoState[Throttle][12]*8;
        Serial.println("<f "+locoAddress+" "+String(func)+">");
      break;
      case 13:
      case 14:
      case 15:
      case 16:
      case 17:
      case 18:
      case 19:
      case 20:
        func = LocoState[Throttle][13]*1+LocoState[Throttle][14]*2+LocoState[Throttle][15]*4+LocoState[Throttle][16]*8+LocoState[Throttle][17]*16+LocoState[Throttle][18]*32+LocoState[Throttle][19]*64+LocoState[Throttle][20]*128;
        Serial.println("<f "+locoAddress+" "+String(222)+" "+String(func)+">");
      break;
      case 21:
      case 22:
      case 23:
      case 24:
      case 25:
      case 26:
      case 27:
      case 28:
        func = LocoState[Throttle][21]*1+LocoState[Throttle][22]*2+LocoState[Throttle][23]*4+LocoState[Throttle][24]*8+LocoState[Throttle][25]*16+LocoState[Throttle][26]*32+LocoState[Throttle][27]*64+LocoState[Throttle][28]*128;
        Serial.println("<f "+locoAddress+" "+String(223)+" "+String(func)+">");
      break;
    }
  }
  else if(actionVal.startsWith("qV")){
    client[i].println("M"+th+"A"+LocoThrottle[Throttle]+"<;>" + "V"+String(LocoState[Throttle][29]));              
  }
  else if(actionVal.startsWith("V")){
    fKey = actionVal.substring(1).toInt();
    LocoState[Throttle][29] = fKey;
    Serial.print("<t "+String(Throttle+1)+" "+locoAddress+"  "+String(fKey)+" "+String(LocoState[Throttle][30])+">");
    response = loadResponse();
  }
  else if(actionVal.startsWith("qR")){
    client[i].println("M"+th+"A"+LocoThrottle[Throttle]+"<;>" + "R"+String(LocoState[Throttle][30]));              
  }
  else if(actionVal.startsWith("R")){
    fKey = actionVal.substring(1).toInt();
    LocoState[Throttle][30] = fKey;
    Serial.print("<t "+String(Throttle+1)+" "+locoAddress+" "+String(LocoState[Throttle][29])+"  "+String(fKey)+">");
    response = loadResponse();
  }
  else if(actionVal.startsWith("X")){
    LocoState[Throttle][29] = 0;
    Serial.print("<t "+String(Throttle+1)+" "+locoAddress+" -1 "+String(LocoState[Throttle][30])+">");
    response = loadResponse();
  }
  else if(actionVal.startsWith("I")){
    LocoState[Throttle][29] = 0;
    Serial.print("<t "+String(Throttle+1)+" "+locoAddress+" 0 "+String(LocoState[Throttle][30])+">");
    response = loadResponse();
  }
  else if(actionVal.startsWith("Q")){
    LocoState[Throttle][29] = 0;
    Serial.print("<t "+String(Throttle+1)+" "+locoAddress+" 0 "+String(LocoState[Throttle][30])+">");
    response = loadResponse();
  }
}

void checkHeartbeat(int i) {
  if(heartbeat[0+i*2] > 0 && heartbeat[0+i*2] + heartbeatTimeout * 1000 < millis()) {
    LocoState[0+i*2][29] = 0;
    heartbeat[0+i*2] = 0;
    client[i].println("MTA"+LocoThrottle[0+i*2]+"<;>" + "V0");
  }
  if(heartbeat[1+i*2] > 0 && heartbeat[1+i*2] + heartbeatTimeout * 1000 < millis()) {
    LocoState[1+i*2][29] = 0;
    heartbeat[1+i*2] = 0;
    client[i].println("MSA"+LocoThrottle[1+i*2]+"<;>" + "V0");
  }
}

void outputToggle(int aID, String aStatus){

  // Convert status to DCC++ format
  int newStat;
  if(aStatus=="T") 
    newStat=1;
  else if(aStatus=="C")
    newStat=0;
  else
    newStat=-1;
  

  // Find output from output table
  int t=0;
  for (t = 0 ; t<totalOutputs && tt[t].id!=aID ; t++);

  if (tt[t].id!=0) {

    // If newStat not Thrown or Closed, flip Thrown/Closed state
    if(newStat<0){
      switch(tt[t].status){
        case 2:
          tt[t].status=4;
          newStat=0;
        break;
        case 4:
          tt[t].status=2;
          newStat=1;
        break;
      }

    } else {

      // Throw or close
      switch(newStat){
        case 0:
          tt[t].status=2;
        break;
        case 1:
          tt[t].status=4;
        break;
      }
    }

    String ptaText = "PTA";
    String accessoryText = "D"+String(tt[t].type);

    for(int i=0; i<maxClient; i++){
      client[i].println(ptaText+String(tt[t].status)+accessoryText+String(tt[t].id));
    }
    Serial.print("<Z "+String(tt[t].id)+" "+String(newStat)+">");
    String response = loadResponse();

  }
}

