#include <SPI.h>
#include <Ethernet.h>
#include <EthernetServer.h>
#include <EEPROM.h>
#define NO_COMMAND -1
#define COMMAND_ON 1
#define COMMAND_OFF 0

#define UNKNOWN_STATE -1
#define STATE_ON 1
#define STATE_OFF 0

#define UNKNOWN_TIMER -1
#define UNKNOWN_TIME 0


int EpromDefaultFactorySettings=1023;
int factoryDefault = EEPROM.read(EpromDefaultFactorySettings);
int EpromInitStateOffset=1016; // size 4 bytes (can fit in one byte)
int EpromIPAddressOffset=1000; // size 4ip+4nm+4gw=12
int EpromMACAddressOffset=1012; // size 2 - last two bytes
int EpromPortOffset=1020 ; //2 bytes
int fixPort=9080;
int eePort=(EEPROM.read(EpromPortOffset)*256+EEPROM.read(EpromPortOffset+1));
int fixIP1=192;
int fixIP2=168;
int fixIP3=1;
int fixIP4=177;
int eeIP1=EEPROM.read(EpromIPAddressOffset);
int eeIP2=EEPROM.read(EpromIPAddressOffset+1);
int eeIP3=EEPROM.read(EpromIPAddressOffset+2);
int eeIP4=EEPROM.read(EpromIPAddressOffset+3);
int fixIPnm1=255;
int fixIPnm2=255;
int fixIPnm3=255;
int fixIPnm4=0;
int eeIPnm1=EEPROM.read(EpromIPAddressOffset+4);
int eeIPnm2=EEPROM.read(EpromIPAddressOffset+5);
int eeIPnm3=EEPROM.read(EpromIPAddressOffset+6);
int eeIPnm4=EEPROM.read(EpromIPAddressOffset+7);
int fixIPgw1=192;
int fixIPgw2=168;
int fixIPgw3=1;
int fixIPgw4=3;
int eeIPgw1=EEPROM.read(EpromIPAddressOffset+8);
int eeIPgw2=EEPROM.read(EpromIPAddressOffset+9);
int eeIPgw3=EEPROM.read(EpromIPAddressOffset+10);
int eeIPgw4=EEPROM.read(EpromIPAddressOffset+11);
int fixMAC1=0xde;
int fixMAC2=0xad;
int fixMAC3=0xbe;
int fixMAC4=0xef;
int fixMAC5=0x1C;
int fixMAC6=0x7C;
int eeMAC5=EEPROM.read(EpromMACAddressOffset);
int eeMAC6=EEPROM.read(EpromMACAddressOffset+1);
int fixR1=0;
int fixR2=0;
int fixR3=0;
int fixR4=0;
int eeR1=EEPROM.read(EpromInitStateOffset);
int eeR2=EEPROM.read(EpromInitStateOffset+1);
int eeR3=EEPROM.read(EpromInitStateOffset+2);
int eeR4=EEPROM.read(EpromInitStateOffset+3);

EthernetServer server(factoryDefault?fixPort:eePort);
byte mac[] = { fixMAC1, 
               fixMAC2, 
               fixMAC3, 
               fixMAC4, 
               (factoryDefault?fixMAC5:eeMAC5), 
               (factoryDefault?fixMAC6:eeMAC6)
             };

IPAddress webServerIP(factoryDefault?fixIP1:eeIP1,
                      factoryDefault?fixIP2:eeIP2,
                      factoryDefault?fixIP3:eeIP3,
                      factoryDefault?fixIP4:eeIP4);

IPAddress netmask(factoryDefault?fixIPnm1:eeIPnm1,
                      factoryDefault?fixIPnm2:eeIPnm2,
                      factoryDefault?fixIPnm3:eeIPnm3,
                      factoryDefault?fixIPnm4:eeIPnm4);

IPAddress gateway(factoryDefault?fixIPgw1:eeIPgw1,
                      factoryDefault?fixIPgw2:eeIPgw2,
                      factoryDefault?fixIPgw3:eeIPgw3,
                      factoryDefault?fixIPgw4:eeIPgw4);

// pins used
#define RELAY_PIN_0 0
#define RELAY_PIN_1 1
#define RELAY_PIN_2 2
#define RELAY_PIN_3 3

String httpRequestLineOne;
String lastHttpCommand;
String prevHttpCommand;
String lastRelayHttpCommand[4];

#define COMMAND_PIN 0
#define COMMAND_CURRENT_STATE_REQUEST 1
#define COMMAND_CURRENT_STATE 2
#define COMMAND_PREVIOUS_STATE 3
#define COMMAND_NEXT_STATE 4
#define COMMAND_NEXT_STATE_TIME 5
int command[4][6]; // relay 4 x (1 pin,cur state,changerq,prevstate,next_state,timer,cmd time)
unsigned long lastChange[4];
unsigned long nextChange[4];
unsigned long lastCommandTime[4];
int firstLineFetched = 0;
int slash0  = 0;
int slash1 = 0;
int slash2 = 0;
int slash3 = 0;
int charPosition = 0;
int targetRelay = 0;
int targetState = 0;
int targetDelay = 0;
unsigned long lastTimeCheck=0;
unsigned long timeCounterResetCount=0;
unsigned long prevTimeCheck=0;

// 255 undefined do not set, 0 off, 1 on
int initState[4]={
  factoryDefault?fixR1:eeR1,
  factoryDefault?fixR2:eeR2,
  factoryDefault?fixR3:eeR3,
  factoryDefault?fixR4:eeR4
};

void(* resetFunc) (void) = 0;

void setup()
{
  int i;
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, webServerIP, gateway, netmask);
  server.begin();

  pinMode(RELAY_PIN_0,OUTPUT);
  pinMode(RELAY_PIN_1,OUTPUT);
  pinMode(RELAY_PIN_2,OUTPUT);
  pinMode(RELAY_PIN_3,OUTPUT);

  command[0][COMMAND_PIN] = RELAY_PIN_0;
  command[1][COMMAND_PIN] = RELAY_PIN_1;
  command[2][COMMAND_PIN] = RELAY_PIN_2;
  command[3][COMMAND_PIN] = RELAY_PIN_3;
  for(i=0;i<4;i++)
    {
      if(initState[i] == 0 || initState[i] == 1) // filter other values than 0 or 1
        digitalWrite(command[i][COMMAND_PIN],initState[i]);
      command[i][COMMAND_CURRENT_STATE_REQUEST] = NO_COMMAND; // command (-1 no command, 0 off, 1 on
      command[i][COMMAND_CURRENT_STATE] = initState[i]; // current state
      command[i][COMMAND_PREVIOUS_STATE] = UNKNOWN_STATE; // prev state
      command[i][COMMAND_NEXT_STATE] = UNKNOWN_STATE; // next_state
      command[i][COMMAND_NEXT_STATE_TIME] = UNKNOWN_TIME; // timer
      lastChange[i]= UNKNOWN_TIME;
      nextChange[i]= UNKNOWN_TIME;
      lastCommandTime[i]= UNKNOWN_TIME;
    }
}


void recordSlashPosition(char c)
{
  charPosition++;
  if( firstLineFetched == 0)
  {
    if(c == '/')
    {
      if(slash0 == 0)
        slash0 = charPosition;
      else
        if(slash0 != 0 && slash1 == 0)
          slash1 = charPosition;
        else
        if(slash0 != 0 && slash1 != 0 && slash2 == 0)
          slash2 = charPosition;
        else              
          if(slash0 != 0 && slash1 != 0 && slash2 != 0 && slash3 == 0)
            slash3 = charPosition;
    }
  }
}

void sendHttpResponose200text(EthernetClient client)
{
  client.println("HTTP/1.0 200 OK");
  client.println("Content-Type: text/html");
  client.println();
}

void printHtmlHeader(EthernetClient client)
{
  client.println("<html>");
  client.println("<meta/>");
  client.println("<body>");
}

void printHtmlFooter(EthernetClient client)
{
    client.println("");
    client.println("</body>");
    client.println("</html>");
}


void printGenericRequestInformation
 (EthernetClient client, String lastHttpCommand, String httpRequestLineOne,
  int slash0, int slash1, int slash2, int slash3)
{
  client.println("<pre>");
  client.println("request was \n<a href=\"" + lastHttpCommand + "\">" + lastHttpCommand + "</a>");
  client.print("request size ");
  client.print(lastHttpCommand.length());
  client.print(" ");
  client.println(httpRequestLineOne.length());
  client.print("slash0 was on ");
  client.println(slash0);
  client.print("slash1 was on ");
  client.println(slash1);
  client.print("slash2 was on ");
  client.println(slash2);
  client.print("slash3 was on ");
  client.println(slash3);
  client.println();
  if(slash1 != 0)
  {
    client.print("1: ");
    client.println(httpRequestLineOne.substring(slash0,slash1-1));
  }
  if(slash2 != 0)
  {
    client.print("2: ");
    client.println(httpRequestLineOne.substring(slash1,slash2-1));
  }
  if(slash3 != 0)
  {
    client.print("3: ");
    client.println(httpRequestLineOne.substring(slash2,slash3-1));
  }
  client.println();
  client.println("</pre>");
}


void printAllRawAnalogChannelValues(EthernetClient client)
{
// output the value of each analog input pin
  for (int analogChannel = 0; analogChannel < 6; analogChannel++)
  {
    client.print("Ain ");
    client.print(analogChannel);
    client.print(": ");
    client.println(analogRead(analogChannel));
  }
}

void printOneRawAnalogChannelValue(EthernetClient client,int analogChannel)
{
  client.print("Ain ");
  client.print(analogChannel);
  client.print(": ");
  client.println(analogRead(analogChannel));
}

void processZeroSlashedArgument(EthernetClient client,String httpCommand)
{
  if( httpCommand == "/favicon.ico")
  {
    client.println("HTTP/1.0 404 Not Found");
    client.println("");
  } else 
  {
    sendHttpResponose200text(client);
  }
  printHtmlHeader(client);
  client.println("Kerber 0.1.2 120705");
  printHtmlFooter(client);
}

void processOneSlashedArgument(EthernetClient client,String httpRequestLineOne,int slash0,int slash1)
{
  String command;

  sendHttpResponose200text(client);
  printHtmlHeader(client);
  command = httpRequestLineOne.substring(slash0,slash1-1);
  if( command == "AnalogInputs" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client); 
    printAllRawAnalogChannelValues(client);
    client.print("</pre>");
  }
  if( command == "A0" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,0);
    client.print("</pre>");
  }
  if( command == "A1" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,1);
    client.print("</pre>");
  }
  if( command == "A2" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,2);
    client.print("</pre>");
  }
  if( command == "A3" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,3);
    client.print("</pre>");
  }
  if( command == "A4" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,4);
    client.print("</pre>");
  }
  if( command == "A5" )
  {
    client.println("<pre>");
    printCurrentMilliSeconds(client);
    printOneRawAnalogChannelValue(client,5);
    client.print("</pre>");
  }
  if( command == "CmdStats" )
  {
    client.println("<pre>");
    printCommandStats(client);
    client.print("</pre>");
  }
  if( command == "R3" )
  {
    int i=3;
    int value;
    client.println("<pre>");
    printDOStats(client,2);
    client.print("</pre>");
  }
  if( command == "r3")
  {
    printRelayCommandStats(client,2);
  }
  if( command == "R4" )
  {
    client.println("<pre>");
    printDOStats(client,3);
    client.print("</pre>");
  }
  if( command == "r4")
  {
    printRelayCommandStats(client,3);
  }
    if( command == "Reset")
  {
    client.println("Going Down");
    client.println("</body></html>");
    client.stop();
    printHtmlFooter(client);
    delay(1000);
    resetFunc();
  }
  if( command == "AddrHelp")
  {
    client.println("<pre>");
    client.println("Addresses: ");
    client.print("DefaultFactory: ");
    client.println(EpromDefaultFactorySettings);
    client.print("InitState: ");
    client.println(EpromInitStateOffset);
    client.print("IPaddress: ");
    client.println(EpromIPAddressOffset);
    client.print("IPnetmask: ");
    client.println(EpromIPAddressOffset+4);
    client.print("IPgateway: ");
    client.println(EpromIPAddressOffset+8);
    client.print("MACAddress: ");
    client.println(EpromMACAddressOffset);
    client.print("Port: ");
    client.println(EpromPortOffset);
    client.println("</pre>");

  }
/*  if( command == "Factory")
  {
    int er = EEPROM.read(EpromDefaultFactorySettings);
    client.println("<pre>");
    client.println("Factory: (");
    client.print(") Booted: ");
    client.print(factoryDefault?1:0);
    client.print(" - ");
    client.println(factoryDefault);
    client.print("NextBoot: ");
    client.print(er?1:0);
    client.print(" - ");
    client.println(er);
    client.println("</pre>");
  }
  if( command == "Booted")
  {
    int er = EEPROM.read(EpromDefaultFactorySettings);

    client.println("<pre>");
    client.println("Factory: (");
    client.print(") Booted: ");
    client.print(factoryDefault?1:0);
    client.print(" - ");
    client.println(factoryDefault);
    client.print("NextBoot: ");
    client.print(er?1:0);
    client.print(" - ");
    client.println(er);
    client.print("InitState booted R1234 (0,1,x)");
    client.print(eeR1); client.print(" ");client.print(eeR2);client.print(" ");
    client.print(eeR3); client.print(" ");client.print(eeR4);client.println(" ");
    client.print("IP booted ");
    client.print(eeIP1); client.print(".");client.print(eeIP2);client.print(".");
    client.print(eeIP3); client.print(".");client.print(eeIP4);client.println(" ");
    client.print("Netmask booted ");
    client.print(eeIPnm1); client.print(".");client.print(eeIPnm2);client.print(".");
    client.print(eeIPnm3); client.print(".");client.print(eeIPnm4);client.println(" ");
    client.print("Gateway booted ");
    client.print(eeIPgw1); client.print(".");client.print(eeIPgw2);client.print(".");
    client.print(eeIPgw3); client.print(".");client.print(eeIPgw4);client.println(" ");
    client.print("MAC booted ");
    client.print(fixMAC1); client.print(":");
    client.print(fixMAC2); client.print(":");
    client.print(fixMAC3); client.print(":");
    client.print(fixMAC4); client.print(":");
    client.print(eeMAC5); client.print(":");
    client.print(eeMAC6); client.println(":");
    client.println("</pre>");
  }
    if( command == "NextBoot")
  {
    int er = EEPROM.read(EpromDefaultFactorySettings);
    int aeePort=(EEPROM.read(EpromPortOffset)*256+EEPROM.read(EpromPortOffset+1));
    int aeeIP1=EEPROM.read(EpromIPAddressOffset);
    int aeeIP2=EEPROM.read(EpromIPAddressOffset+1);
    int aeeIP3=EEPROM.read(EpromIPAddressOffset+2);
    int aeeIP4=EEPROM.read(EpromIPAddressOffset+3);
    int aeeIPnm1=EEPROM.read(EpromIPAddressOffset+4);
    int aeeIPnm2=EEPROM.read(EpromIPAddressOffset+5);
    int aeeIPnm3=EEPROM.read(EpromIPAddressOffset+6);
    int aeeIPnm4=EEPROM.read(EpromIPAddressOffset+7);
    int aeeIPgw1=EEPROM.read(EpromIPAddressOffset+8);
    int aeeIPgw2=EEPROM.read(EpromIPAddressOffset+9);
    int aeeIPgw3=EEPROM.read(EpromIPAddressOffset+10);
    int aeeIPgw4=EEPROM.read(EpromIPAddressOffset+11);
    int aeeMAC1=EEPROM.read(EpromMACAddressOffset);
    int aeeMAC2=EEPROM.read(EpromMACAddressOffset+1);
    int aeeR1=EEPROM.read(EpromInitStateOffset);
    int aeeR2=EEPROM.read(EpromInitStateOffset+1);
    int aeeR3=EEPROM.read(EpromInitStateOffset+2);
    int aeeR4=EEPROM.read(EpromInitStateOffset+3);

    client.println("<pre>");
    client.println("Factory: (");
    client.print(") Booted: ");
    client.print(factoryDefault?1:0);
    client.print(" - ");
    client.println(factoryDefault);
    client.print("NextBoot: ");
    client.print(er?1:0);
    client.print(" - ");
    client.println(er);
    client.print("InitState nextBoot R1234 (0,1,x)");
    client.print(aeeR1); client.print(" ");client.print(aeeR2);client.print(" ");
    client.print(aeeR3); client.print(" ");client.print(aeeR4);client.println(" ");
    client.print("IP nextBoot ");
    client.print(aeeIP1); client.print(".");client.print(aeeIP2);client.print(".");
    client.print(aeeIP3); client.print(".");client.print(aeeIP4);client.println(" ");
    client.print("Netmask nextBoot ");
    client.print(aeeIPnm1); client.print(".");client.print(aeeIPnm2);client.print(".");
    client.print(aeeIPnm3); client.print(".");client.print(aeeIPnm4);client.println(" ");
    client.print("Netmask nextBoot ");
    client.print(aeeIPgw1); client.print(".");client.print(aeeIPgw2);client.print(".");
    client.print(aeeIPgw3); client.print(".");client.print(aeeIPgw4);client.println(" ");
    client.print("MAC nextBoot ");
    client.print(fixMAC1); client.print(":");
    client.print(fixMAC2); client.print(":");
    client.print(fixMAC3); client.print(":");
    client.print(fixMAC4); client.print(":");
    client.print(eeMAC5); client.print(":");
    client.print(eeMAC6); client.println(":");
    client.println("</pre>");
  }
    if( command == "FactoryAll")
  {
    int er = EEPROM.read(EpromDefaultFactorySettings);
    int aeePort=(EEPROM.read(EpromPortOffset)*256+EEPROM.read(EpromPortOffset+1));
    int aeeIP1=EEPROM.read(EpromIPAddressOffset);
    int aeeIP2=EEPROM.read(EpromIPAddressOffset+1);
    int aeeIP3=EEPROM.read(EpromIPAddressOffset+2);
    int aeeIP4=EEPROM.read(EpromIPAddressOffset+3);
    int aeeIPnm1=EEPROM.read(EpromIPAddressOffset+4);
    int aeeIPnm2=EEPROM.read(EpromIPAddressOffset+5);
    int aeeIPnm3=EEPROM.read(EpromIPAddressOffset+6);
    int aeeIPnm4=EEPROM.read(EpromIPAddressOffset+7);
    int aeeIPgw1=EEPROM.read(EpromIPAddressOffset+8);
    int aeeIPgw2=EEPROM.read(EpromIPAddressOffset+9);
    int aeeIPgw3=EEPROM.read(EpromIPAddressOffset+10);
    int aeeIPgw4=EEPROM.read(EpromIPAddressOffset+11);
    int aeeMAC1=EEPROM.read(EpromMACAddressOffset);
    int aeeMAC2=EEPROM.read(EpromMACAddressOffset+1);
    int aeeR1=EEPROM.read(EpromInitStateOffset);
    int aeeR2=EEPROM.read(EpromInitStateOffset+1);
    int aeeR3=EEPROM.read(EpromInitStateOffset+2);
    int aeeR4=EEPROM.read(EpromInitStateOffset+3);

    client.println("<pre>");
    client.println("Factory: (");
    client.print(") Booted: ");
    client.print(factoryDefault?1:0);
    client.print(" - ");
    client.println(factoryDefault);
    client.print("NextBoot: ");
    client.print(er?1:0);
    client.print(" - ");
    client.println(er);
    client.print("InitState fix R1234 (0,1,x) ");
    client.print(fixR1); client.print(" ");client.print(fixR2);client.print(" ");
    client.print(fixR3); client.print(" ");client.print(fixR4);client.println(" ");
    client.print("InitState booted R1234 (0,1,x)");
    client.print(eeR1); client.print(" ");client.print(eeR2);client.print(" ");
    client.print(eeR3); client.print(" ");client.print(eeR4);client.println(" ");
    client.print("InitState nextBoot R1234 (0,1,x)");
    client.print(aeeR1); client.print(" ");client.print(aeeR2);client.print(" ");
    client.print(aeeR3); client.print(" ");client.print(aeeR4);client.println(" ");
    client.print("IP fix ");
    client.print(fixIP1); client.print(".");client.print(fixIP2);client.print(".");
    client.print(fixIP3); client.print(".");client.print(fixIP4);client.println(" ");
    client.print("IP booted ");
    client.print(eeIP1); client.print(".");client.print(eeIP2);client.print(".");
    client.print(eeIP3); client.print(".");client.print(eeIP4);client.println(" ");
    client.print("IP nextBoot ");
    client.print(aeeIP1); client.print(".");client.print(aeeIP2);client.print(".");
    client.print(aeeIP3); client.print(".");client.print(aeeIP4);client.println(" ");
    client.print("Netmask fix ");
    client.print(fixIPnm1); client.print(".");client.print(fixIPnm2);client.print(".");
    client.print(fixIPnm3); client.print(".");client.print(fixIPnm4);client.println(" ");
    client.print("Netmask booted ");
    client.print(eeIPnm1); client.print(".");client.print(eeIPnm2);client.print(".");
    client.print(eeIPnm3); client.print(".");client.print(eeIPnm4);client.println(" ");
    client.print("Netmask nextBoot ");
    client.print(aeeIPnm1); client.print(".");client.print(aeeIPnm2);client.print(".");
    client.print(aeeIPnm3); client.print(".");client.print(aeeIPnm4);client.println(" ");
    client.print("Gateway fix ");
    client.print(fixIPgw1); client.print(".");client.print(fixIPgw2);client.print(".");
    client.print(fixIPgw3); client.print(".");client.print(fixIPgw4);client.println(" ");
    client.print("Netmask booted ");
    client.print(eeIPgw1); client.print(".");client.print(eeIPgw2);client.print(".");
    client.print(eeIPgw3); client.print(".");client.print(eeIPgw4);client.println(" ");
    client.print("Netmask nextBoot ");
    client.print(aeeIPgw1); client.print(".");client.print(aeeIPgw2);client.print(".");
    client.print(aeeIPgw3); client.print(".");client.print(aeeIPgw4);client.println(" ");
    client.print("MAC fix ");
    client.print(fixMAC1); client.print(":");
    client.print(fixMAC2); client.print(":");
    client.print(fixMAC3); client.print(":");
    client.print(fixMAC4); client.print(":");
    client.print(fixMAC5); client.print(":");
    client.print(fixMAC6); client.println(":");
    client.print("MAC booted ");
    client.print(fixMAC1); client.print(":");
    client.print(fixMAC2); client.print(":");
    client.print(fixMAC3); client.print(":");
    client.print(fixMAC4); client.print(":");
    client.print(eeMAC5); client.print(":");
    client.print(eeMAC6); client.println(":");
    client.print("MAC nextBoot ");
    client.print(fixMAC1); client.print(":");
    client.print(fixMAC2); client.print(":");
    client.print(fixMAC3); client.print(":");
    client.print(fixMAC4); client.print(":");
    client.print(eeMAC5); client.print(":");
    client.print(eeMAC6); client.println(":");
    client.println("</pre>");

  }
*/
  printHtmlFooter(client);
}

void processTwoSlashedArguments(String httpRequestLineOne,int slash0, int slash1,int slash2)
{

}

void processThreeSlashedArguments(EthernetClient client,String httpRequestLineOne,int slash0,int slash1,int slash2,int slash3)
{
  String chunkCommand;
  String chunkRelay;
  String chunkTime;
  int decode;
  int relay;
           
  sendHttpResponose200text(client);
  printHtmlHeader(client);

  chunkRelay = httpRequestLineOne.substring(slash0,slash1-1);
  relay = chunkRelay.toInt();
  relay--;
  chunkCommand = httpRequestLineOne.substring(slash1,slash2-1);
  if( relay >= 0 && relay <4 )
  {
    if( chunkCommand == "ON" || chunkCommand == "on" )
    {
      lastRelayHttpCommand[relay] = lastHttpCommand;
      command[relay][COMMAND_CURRENT_STATE_REQUEST] = COMMAND_ON;
    }
    if( chunkCommand == "OFF" || chunkCommand == "off" )
    {
      lastRelayHttpCommand[relay] = lastHttpCommand;
      command[relay][COMMAND_CURRENT_STATE_REQUEST] = COMMAND_OFF;
    }

    // do we have third argument
    chunkTime = httpRequestLineOne.substring(slash2,slash3-1);
    decode = chunkTime.toInt();

    if(decode>0 && command[relay][COMMAND_CURRENT_STATE_REQUEST] != NO_COMMAND)
    {
      command[relay][COMMAND_NEXT_STATE] = command[relay][COMMAND_CURRENT_STATE_REQUEST];
      command[relay][COMMAND_CURRENT_STATE_REQUEST] = NO_COMMAND;
      if( chunkCommand == "on" )
      {
        command[relay][COMMAND_CURRENT_STATE_REQUEST] = COMMAND_ON;
        command[relay][COMMAND_NEXT_STATE] = COMMAND_OFF;
      }
      if( chunkCommand == "off" )
      {
        command[relay][COMMAND_CURRENT_STATE_REQUEST] = COMMAND_OFF;
        command[relay][COMMAND_NEXT_STATE] = COMMAND_ON;
      }
      command[relay][COMMAND_NEXT_STATE_TIME] = decode;
      lastCommandTime[relay] = millis()/100;
      nextChange[relay] = lastCommandTime[relay] + decode;
    }
    printRelayCommandStats(client,relay);
    printCommandActionResolution(client);
  }
  if(relay < 0) {
   if(chunkRelay == "EW")
   {// eprom write
     int addr=chunkCommand.toInt();
     String stringValue = httpRequestLineOne.substring(slash2,slash3-1);
     int value = stringValue.toInt();
     if(addr>=0 && value>=0)
     {
       client.print("Write: addr=");
       client.print(addr);
       client.print(" value=");
       client.println(value);
       EEPROM.write(addr,value);
     }
   }
   if(chunkRelay == "ER")
   { // eprom read
     int addr=chunkCommand.toInt();
     String stringValue = httpRequestLineOne.substring(slash2,slash3-1);
     int value=-1;
     if(addr>=0)
     {
       value=EEPROM.read(addr);
       client.print("Read: addr=");
       client.print(addr);
       client.print(" value=");
       client.println(value);
     }
   }
  }
  printHtmlFooter(client);
}

String decodeState(int x)
{
  if(x == 0)
    return "OFF";
  if(x == -1)
    return "UNK";
  return "ON ";
}

void printDOStats(EthernetClient client,int i) 
{
    client.print("DO ");
    client.print(command[i][COMMAND_PIN] + 1);
    client.print(" " + decodeState(command[i][COMMAND_CURRENT_STATE_REQUEST]));
    client.print(" " + decodeState(command[i][COMMAND_CURRENT_STATE]));
    client.print(" " + decodeState(command[i][COMMAND_PREVIOUS_STATE]));
    client.print(" " + decodeState(command[i][COMMAND_NEXT_STATE]));
    client.print(" ");client.print(command[i][COMMAND_NEXT_STATE_TIME]);
    client.print(" ");client.print(lastCommandTime[i]);
    client.print(" ");client.print(nextChange[i]);
    client.print(" ");client.print(lastChange[i]);
    client.print(" ");client.print(millis()/100);
//    client.println(" <a href=\"" + lastRelayHttpCommand[i] + "\">" + lastRelayHttpCommand[i] + "</a>");
    client.println();
}

void printCommandStats(EthernetClient client)
{
  for(int i=0;i<4;i++)
    printDOStats(client,i);
}

void printRelayCommandStats(EthernetClient client,int i)
{
  client.println("<pre>");
  client.print("REL:");
  client.println(command[i][COMMAND_PIN] + 1);
  client.print("CSR:");
  client.println(decodeState(command[i][COMMAND_CURRENT_STATE_REQUEST]));
  client.print("CCS:");
  client.println(decodeState(command[i][COMMAND_CURRENT_STATE]));
  client.print("CPS:");
  client.println(decodeState(command[i][COMMAND_PREVIOUS_STATE]));
  client.print("CNS:");
  client.println(decodeState(command[i][COMMAND_NEXT_STATE]));
  client.print("CNT:");
  client.println(command[i][COMMAND_NEXT_STATE_TIME]);
  client.print("LCT:");
  client.println(lastCommandTime[i]/100);
  client.print("NCT:");
  client.println(nextChange[i]);
  client.print("CDT:");
  client.println(lastCommandTime[i]/100+command[i][COMMAND_NEXT_STATE_TIME]);
  client.print("LRT:");
  client.println(lastChange[i]);
  client.print("MLS:");
  client.println(millis()/100);
//  client.println("<a href=\"" + lastRelayHttpCommand[i] + "\">" + lastRelayHttpCommand[i] + "</a>");
  client.print("</pre>");
}

void printCurrentMilliSeconds(EthernetClient client)
{
  client.print("cur.ms ");
  client.print(millis());
  client.print(" ");
  client.print(timeCounterResetCount);
  client.println();
}

void printCommandActionResolution(EthernetClient client)
{
  int i;
  for(i=0;i<4;i++)
  {
    if(command[i][COMMAND_CURRENT_STATE_REQUEST] != NO_COMMAND)
    {
      if(command[i][COMMAND_CURRENT_STATE_REQUEST] == COMMAND_OFF &&
         command[i][COMMAND_CURRENT_STATE] != command[i][COMMAND_CURRENT_STATE_REQUEST])
      {
        digitalWrite(command[i][COMMAND_PIN],LOW);
        lastChange[i]= millis()/100;
        client.print("LOW to relay ");
        client.println(i);
      }
      if(command[i][COMMAND_CURRENT_STATE_REQUEST] == COMMAND_ON && 
         command[i][COMMAND_CURRENT_STATE] != command[i][COMMAND_CURRENT_STATE_REQUEST])
      {
        digitalWrite(command[i][COMMAND_PIN],HIGH);
        lastChange[i]= millis()/100;
        client.print("HIGH to REL ");
        client.println(i+1);
      }
      command[i][COMMAND_PREVIOUS_STATE] = command[i][COMMAND_CURRENT_STATE];
      command[i][COMMAND_CURRENT_STATE] = command[i][COMMAND_CURRENT_STATE_REQUEST];
      command[i][COMMAND_CURRENT_STATE_REQUEST] = NO_COMMAND;
// TODO write down what is the time left to execute submited request
    }
  }
}

void actUponCommand()
{
  int i;
  for(i=0;i<4;i++)
  {
    if(command[i][COMMAND_NEXT_STATE_TIME] > 0 && command[i][COMMAND_NEXT_STATE] != NO_COMMAND)
    {
      if(lastCommandTime[i] + command[i][COMMAND_NEXT_STATE_TIME] < millis()/100)
      {
        if(command[i][COMMAND_NEXT_STATE] == COMMAND_OFF)
        {
          digitalWrite(command[i][COMMAND_PIN],LOW);
          lastChange[i]= millis()/100;
        }
        if(command[i][COMMAND_NEXT_STATE] == COMMAND_ON)
        {
          digitalWrite(command[i][COMMAND_PIN],HIGH);
          lastChange[i]= millis()/100;
        }
        command[i][COMMAND_PREVIOUS_STATE] = command[i][COMMAND_CURRENT_STATE];
        command[i][COMMAND_CURRENT_STATE] = command[i][COMMAND_NEXT_STATE];
        command[i][COMMAND_NEXT_STATE] = NO_COMMAND;
      }
    }
  }
}

void loop()
{
  //  int i;
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    httpRequestLineOne= "";
    while (client.connected()) 
    {
      if (client.available()) 
      {
        char c = client.read();
        recordSlashPosition(c);
        if(firstLineFetched == 0)
          httpRequestLineOne += c;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          int spaceOne = httpRequestLineOne.indexOf(" ");
          int spaceTwo = httpRequestLineOne.indexOf(" ",spaceOne+1);

          // store last http command
          prevHttpCommand= lastHttpCommand;
          // set current http command as lastHttpCommand
          lastHttpCommand= httpRequestLineOne.substring(spaceOne+1,spaceTwo);
          // send a standard http response header
//          printGenericRequestInformation
//            (client,lastHttpCommand,httpRequestLineOne,slash0,slash1,slash2,slash3);

          if(slash2 == 0)
          {
            processZeroSlashedArgument(client,lastHttpCommand);
          }
          if(slash2 != 0 && slash3 == 0)
          {
            processOneSlashedArgument(client,httpRequestLineOne,slash0,slash1);
          }
          if(slash3 != 0)
          {
            processThreeSlashedArguments(client,httpRequestLineOne,slash0,slash1,slash2,slash3);
          }
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          firstLineFetched = 1;
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }

// clean up for next request
    httpRequestLineOne = String("");    
    firstLineFetched = 0;
    slash0 = 0;
    slash1 = 0;
    slash2 = 0;
    slash3 = 0;
    charPosition = 0;

    // give the web browser time to receive the data
    delay(100);
    // close the connection:
    client.stop();    
  }
  // we act on every loop as timed event can occur
  actUponCommand();
  prevTimeCheck=lastTimeCheck;
  lastTimeCheck=millis();
  if(lastTimeCheck < prevTimeCheck)
     timeCounterResetCount++;
}

