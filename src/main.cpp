#include <Arduino.h>
#include "kalman.hpp"
/* 
 *  BLE Bluetooth Beacon, Adapted from Arduino Examples and various other sources.
 *  Adriaan van Wijk
 *  2021
 *  Additional Libraries by Rinky-Dink (OLED) and the guy who wrote the BLE one.
 *  This program is meant for a ESP32 board which will search for BLE devices. It then matches a BLE device
 *  with a set of known MAC address (or other identifiable data) and if it matches, and is in range, will function as
 *  a trigger that can allow other applications, such as opening a gate, for example.
 *
 *  The Wifi Addition is meant to allow visitors to connect to the local AP and by accessing 192.168.1.25
 *  open and close the gate. Based on https://lastminuteengineers.com/creating-esp32-web-server-arduino-ide/

 *   Kalman filter comes from https://github.com/hmartiro/kalman-cpp
 *   Eigen comes from https://gitlab.com/libeigen/eigen.git
 *  To Do:
 *
 * 
 * 
 */

#include <WiFi.h>
#include <WebServer.h>
#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include <EEPROM.h>
#include <OLED_I2C.h>
#include <iostream>
#include <string>

//Function Declearations

int median(int incomingdata[], int dataCounter);
void wifi_init();
void triggerGate(uint16_t delaytime);
void handle_OnConnect();
void handle_sendrssi();
void handle_toggleGate();
void handle_exitconfig();
void handle_NotFound();
String SendHTML(uint8_t active);
String refreshpageHTML();

/* Put your SSID & Password */
const char *ssid = "Tarentaal";        // Enter SSID here
const char *password = "birdsfordays"; //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 25);
IPAddress gateway(192, 168, 1, 25);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

/* Enter Known BLE Device Mac Addresses */
String knownBLEAddresses[] = {"db:17:35:a3:4c:27"};

/* Constants */
int RSSI_THRESHOLD = -60;     // Is overwritten by CUSTOM_IN and CUSTOM_OUT dynamically
int RSSI_CONFIG = -40;        // For starting the Webserver
uint8_t RSSI_CUSTOM_IN = 60;  //The EEPROM value will overwrite this.
uint8_t RSSI_CUSTOM_OUT = 60; //The EEPROM value will overwrite this.

bool device_found = false;
bool wifibool = false;
bool config_ble = false;
bool gate_delay = true; // Set as true so we don't need to wait for timedelay on first boot. Gate is primed after restart.

/* More Constants */
// uint8_t s_trigger = 26;
// uint8_t s_config = 25;
// uint8_t s_up = 14;
// uint8_t s_down = 27;
// uint8_t s_enter = 13;
uint8_t LED_BLE = 17;
uint8_t LED_WIFI = 18;
uint8_t LED_TRIGGER = 16;

uint8_t linenumber = 0;
uint8_t LED_BUILTIN = 2; // Can Remove

uint32_t scanTime = 1;    //Duration of each scan in seconds 2 1100 1099
uint16_t interval = 100; //the intervals at which scanning is actively taking place in milliseconds
uint16_t window = 100;   //the window of time after each interval which is being scanned in milliseconds
//500 here is the best compromise. lower for better wifi, higher for better ble.
uint16_t configcounter = 0;

double distance = 0;
double temp = -1;

unsigned long previoustime = 0;
const long time_delay = 1000; // Ideally 15 sec, in ms.
unsigned long currenttime;
int pos = 0;
int scannumber = 0;
int i = 0;
int dataCounter = 0;
int incomingData[50] = {0};
int k;
int prev = 0;
int state = 0; // 0 - inside | 1 - outside
int rssi = -111;
int keyrssi = -111;
String ptr;
String str;
String str3;
BLEScan *pBLEScan; //ble pointer
std::string addressReturn;

unsigned long blescantime =0;

OLED myOLED(21, 22);
extern uint8_t SmallFont[];
extern uint8_t MedNum[];

/* Button Interrupt Handling */

struct Button
{
    const uint8_t PIN;
    uint32_t numberKeyPresses;
};

Button b_trigger = {26, 0};
Button b_config = {25, 0};
Button b_up = {14, 0};
Button b_down = {27, 0};
Button b_enter = {13, 0};

void IRAM_ATTR i_trigger()
{
    b_trigger.numberKeyPresses += 1;
    //QQQ THIS IS PROBABLY A HORRIBLE IDEA
    //digitalWrite(LED_TRIGGER, HIGH);
    //Note that by doing this I'm also triggering the REMOTE. Connected on SAME PIN.
}
void IRAM_ATTR i_config()
{
    b_config.numberKeyPresses += 1;
}
void IRAM_ATTR i_up()
{
    b_up.numberKeyPresses += 1;
}
void IRAM_ATTR i_down()
{
    b_down.numberKeyPresses += 1;
}
void IRAM_ATTR i_enter()
{
    b_enter.numberKeyPresses += 1;
}

/* BLE Function Code */

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        // Serial.println("Function For Loop Start...");
        for (int i = 0; i < (sizeof(knownBLEAddresses) / sizeof(knownBLEAddresses[i])); i++)
        {
            //Compare Each incoming signal with my code above and if it matches, set a flag that the key is within range.
            str = advertisedDevice.toString().c_str();
            //pos = str.indexOf("Address: "); // Note that we can also require a different attribute such as name instead of address to double vertify
            //This was when I was still analyzing hte whole string and doing needless work...
            //str3 = str.substring(pos + 9, pos + 26);
            //Serial.println("In Loop...");
            addressReturn = advertisedDevice.getAddress().toString();

            if ((strcmp(addressReturn.c_str(), knownBLEAddresses[i].c_str()) == 0))
            {
                //    Serial.println("Device MATCHES");
                //    Serial.println("THERE IS A MATCHHHHHHHHHHHHHH\n");
                //    Serial.println(str3.c_str());
                //    Serial.println(knownBLEAddresses[i].c_str());
                device_found = true; //flag that is set that indicates I am within BLE range
            }
            else
            {
                //   Serial.println("Device not found");
                //    Serial.println("No Match")
                //    Serial.println(str3.c_str());
                //   Serial.println(knownBLEAddresses[i].c_str());
            }
            scannumber++;
        }
        //Serial.println("*************END FOR LOOP**************");
    }
};

void setup()
{
    EEPROM.begin(2);
    Serial.begin(115200); //Enable UART on ESP32

    pinMode(b_trigger.PIN, INPUT_PULLUP);
    pinMode(b_config.PIN, INPUT_PULLUP);
    pinMode(b_up.PIN, INPUT_PULLUP);
    pinMode(b_down.PIN, INPUT_PULLUP);
    pinMode(b_enter.PIN, INPUT_PULLUP);

    attachInterrupt(b_trigger.PIN, i_trigger, HIGH);
    attachInterrupt(b_config.PIN, i_config, HIGH);
    attachInterrupt(b_up.PIN, i_up, HIGH);
    attachInterrupt(b_down.PIN, i_down, HIGH);
    attachInterrupt(b_enter.PIN, i_enter, HIGH);

    pinMode(LED_BLE, OUTPUT);
    pinMode(LED_WIFI, OUTPUT);
    pinMode(LED_TRIGGER, OUTPUT);

    pinMode(LED_BUILTIN, OUTPUT);
    RSSI_CUSTOM_IN = EEPROM.read(0); // Note that these are UNSIGNED 8 bit ints, 0 - 255 range.
    RSSI_CUSTOM_OUT = EEPROM.read(1);


    if(!myOLED.begin(SSD1306_128X64))
        while(1);   // In case the library failed to allocate enough RAM for the display buffer...
    myOLED.setFont(SmallFont);

    /* BLE Initialization */
    Serial.println("Scanning...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();                                           //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); //Init Callback Function
    pBLEScan->setActiveScan(true);                                             //active scan uses more power, but get results faster
    pBLEScan->setInterval(interval);                                           // set Scan interval // TRY 128
    pBLEScan->setWindow(window);                                               // less or equal setInterval value // TRY 16
    Serial.println("Finish Initialize");

    
    //Start WIFI and BLE simultaneously
    //Enable if you want Simultaneous BLE and Wifi.
    //wifi_init();


}
void loop()
{
    currenttime = millis();
    //Serial.println("Program Loop Start");
    //Time keeping function used to prevent multiple gate retriggers
    if (currenttime - previoustime >= time_delay)
    {
        previoustime = currenttime;
        gate_delay = true;
    }

    /* Proximity Based Config Trigger */
    if (configcounter >= 2) // 2 Is arbitrary, can be more
    {

        Serial.println("Disable BLE\n");
        configcounter = 0;
        config_ble = true;
        wifibool = false;
    }

    /* Exit BLE Mode, Start Wifi */
    if ((b_config.numberKeyPresses > 0) && (!config_ble))
    {
        Serial.println("Config Button Pushed, Disable BLE\n");
        configcounter = 0;
        config_ble = true;
        wifibool = false;
        delay(25);
        b_config.numberKeyPresses = 0;
    }

    /* Exit Wifi Mode, Return to BLE */
    if ((b_config.numberKeyPresses > 0) && (config_ble))
    {
        digitalWrite(LED_BLE, HIGH);
        Serial.println("\n Button Pushed, Disabling Wifi");
        WiFi.softAPdisconnect(true);
        config_ble = false;
        delay(25);
        b_config.numberKeyPresses = 0;
    }

    /* Trigger Gate */
    if (b_trigger.numberKeyPresses > 0)
    {
        Serial.println("Gate Trigger Button Pushed");
        triggerGate(1500);
        delay(25);
        b_trigger.numberKeyPresses = 0;
    }
    
    if (!config_ble) /* Start of BLE Loop */
    {
        digitalWrite(LED_WIFI, LOW);
        digitalWrite(LED_BLE, HIGH);

        /* OLED DISPLAY UPDATE */
        myOLED.clrScr();
        myOLED.print("RSSI:", LEFT, 8);
        myOLED.printNumI(keyrssi, RIGHT, 8);
        myOLED.print("Algebraic Dist:", LEFT, 24);
        myOLED.printNumF(distance, 3, RIGHT, 24); // 0.123
        myOLED.print("Kalmann Filter:", LEFT, 40);
        myOLED.update();

        //BLE Functionality
        //Serial.println("BLE Loop Start");

        BLEScanResults foundDevices = pBLEScan->start(scanTime, false); // This Takes 1 Second.
  
        //Needed if you want simultaneous wifi to work.
        //server.handleClient();
       
        //Serial.println("BLE End Scanresults\n");
        //Serial.println(foundDevices.getCount());

        for (k = 0; k < foundDevices.getCount(); k++)
        {
            //Serial.printf("Loop of FoundDevices: %d\n", k);
            BLEAdvertisedDevice device = foundDevices.getDevice(k);
            rssi = device.getRSSI();
            if (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0) // Restricts to only KNOWN mac addresses. 0 when identical.
            {
                /*BLE Beacon has been found and is within range. */
                Serial.println(millis() - blescantime); // The time between sucessive updates
                blescantime = millis();

                //Serial.printf("Key found, rssi: (%d) rssi of Current Threshhold: %d \n", rssi, RSSI_THRESHOLD);
                //Serial.printf("%d\n", rssi);
                
                keyrssi = rssi;
                //Serial.printf("RSSIA %d\n", rssi);
                //Convert RSSI to Distance
                // Source : https://iotandelectronics.wordpress.com/2016/10/07/how-to-calculate-distance-from-the-rssi-value-of-the-ble-beacon/
                temp = -70 - rssi;
                temp = temp / 40; // The 40 is arbitrary and must be tuned.
                distance = pow(10, temp);
                //Serial.printf("Algebriac Distance of Key is (Approx) %f m \n", distance);

                //Serial.printf("%f,\n", (double)rssi);
                //Serial.printf("%f\n", distance);

                //Serial.write("%d", rssi);
                //Serial.write("%d",(int16_t)distance);
            }

            if ((rssi > RSSI_THRESHOLD) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
            {
                prev = 1; // Light was turned on previously in this loop
                Serial.printf("Prev has been triggered. %d, %d", rssi, RSSI_THRESHOLD);
            }
            if ((rssi > RSSI_CONFIG) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
            {
                configcounter++;
                Serial.printf("Config Counter: %d\n", configcounter);
            }

        } // END of for Loop

        //  PREV indicates that one of the devices is our key, is in range, AND has the correct identity.
        if (prev == 1)
        {
            //TRIGGER ENABLED.
            if (state == 0 && gate_delay == true)
            {
                Serial.println("Triggered, going OUT");
                triggerGate(1500);
                device_found = false; // Reset device found, so it needs to be triggered again.
                prev = 0;
                state = 1; // Assume now outside
                RSSI_THRESHOLD = -RSSI_CUSTOM_OUT;
                gate_delay = false;
            }

            if (state == 1 && gate_delay == true) // Assume state = 1 so I am outside.
            {
                Serial.println("Triggered, coming IN");
                triggerGate(1500);
                device_found = false; // Reset device found, so it needs to be triggered again.
                prev = 0;
                state = 0; // Assume now inside
                RSSI_THRESHOLD = -RSSI_CUSTOM_IN;
                gate_delay = false;
            }
            //prev = 0;
        }
        pBLEScan->clearResults(); // delete results fromBLEScan buffer to release memory
    }
    else /* Start of Wifi Server*/
    {
        if (!wifibool)
            wifi_init();       //initialize wifi, runs once.
        server.handleClient(); // handle the wifi page requests

        /*Update OLED Display */
        myOLED.clrScr();
        myOLED.print("Editing Values:", LEFT, 8);
        myOLED.print("Custom RSSI IN:", LEFT, 24);
        myOLED.printNumI(-RSSI_CUSTOM_IN, RIGHT, 24);
        myOLED.print("Custom RSSI OUT:", LEFT, 40);
        myOLED.printNumI(-RSSI_CUSTOM_OUT, RIGHT, 40);
        myOLED.update();

        if (linenumber == 0) //On the first line of OLED Display
        {
            //Consdier including a flash indicator here.
            if (b_up.numberKeyPresses > 0)
            {
                RSSI_CUSTOM_IN += 2;
                delay(25);
                b_up.numberKeyPresses = 0;
            }

            if (b_down.numberKeyPresses > 0)
            {
                RSSI_CUSTOM_IN -= 2;
                delay(25);
                b_down.numberKeyPresses = 0;
            }

            if (b_enter.numberKeyPresses > 0)
            {
                linenumber = 1;
                delay(25);
                b_enter.numberKeyPresses = 0;
            }
        }
        if (linenumber == 1) //On the second line of OLED Display
        {
            //Consdier including a flash indicator here.
            if (b_up.numberKeyPresses > 0)
            {
                RSSI_CUSTOM_OUT += 2;
                delay(25);
                b_up.numberKeyPresses = 0;
            }

            if (b_down.numberKeyPresses > 0)
            {
                RSSI_CUSTOM_OUT -= 2;
                delay(25);
                b_down.numberKeyPresses = 0;
            }

            if (b_enter.numberKeyPresses > 0)
            {
                linenumber = 0;
//ENABLE FOR PRODUCTION BELOW
                 EEPROM.write(0, RSSI_CUSTOM_IN);
                 EEPROM.write(1, RSSI_CUSTOM_OUT);
                 EEPROM.commit();
                 Serial.println("State saved in flash memory:");
                 Serial.printf("Custom IN: %d\n", RSSI_CUSTOM_IN);
                 Serial.printf("Custom OUT: %d\n", RSSI_CUSTOM_OUT);
                // Update OLED to show value has been saved.
                 myOLED.clrScr();       
                 myOLED.print("Custom Values Saved", CENTER, 24);
                 myOLED.update();
                 delay(2000); // To allow user to read message
                 b_enter.numberKeyPresses = 0;
            }
        }

    }
    

} // end of main loop


/* Functions Library: */
void triggerGate(uint16_t delaytime)
{
    digitalWrite(LED_TRIGGER, HIGH);
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_BLE, LOW);
    delay(delaytime);
    digitalWrite(LED_TRIGGER, LOW);
}
void wifi_init() /* Wifi Intialization - Runs Once. */
{

    digitalWrite(LED_BLE, LOW);
    digitalWrite(LED_WIFI, HIGH);

    myOLED.clrScr();
    //myOLED.print("Wifi Innit.", LEFT, 8);
    myOLED.print("Wifi Initialize", CENTER, 24);
    //myOLED.printNumI(RSSI_CUSTOM_IN, RIGHT, 24);
    //myOLED.print("Custom RSSI OUT:", LEFT, 40);
    //myOLED.printNumI(RSSI_CUSTOM_OUT, RIGHT, 40);
    myOLED.update();

    WiFi.softAP(ssid, password);
    //delay(2000); // potential delay to avoid crashing.
    WiFi.softAPConfig(local_ip, gateway, subnet);
    gate_delay = false;

    /* WebServer Command Structure */
    server.on("/", handle_OnConnect);
    server.on("/toggle", handle_toggleGate);
    server.on("/exitconfig", handle_exitconfig);
    server.on("/sendrssi", HTTP_POST, handle_sendrssi);
    server.onNotFound(handle_NotFound);
    server.begin();
    Serial.println("HTTP server started");
    wifibool = true;
}

/* Web Handling Functions */

void handle_OnConnect()
{
    server.send(200, "text/html", SendHTML(LOW)); //Not Active
    digitalWrite(LED_TRIGGER, LOW);               // lol why is this here
}

void handle_sendrssi()
{

    if (server.arg("custom_in") != "")
        RSSI_CUSTOM_IN = abs(server.arg("custom_in").toInt());
    if (server.arg("custom_out") != "")
        RSSI_CUSTOM_OUT = abs(server.arg("custom_out").toInt());

    EEPROM.write(0, RSSI_CUSTOM_IN);
    EEPROM.write(1, RSSI_CUSTOM_OUT);
    EEPROM.commit();
    delay(20); // Just in case
    Serial.println("State saved in flash memory:");
    Serial.printf("Custom IN: %d\n", RSSI_CUSTOM_IN);
    Serial.printf("Custom OUT: %d\n", RSSI_CUSTOM_OUT);
    server.send(200, "text/html", refreshpageHTML());
}

void handle_toggleGate()
{
    digitalWrite(LED_TRIGGER, HIGH);
    server.send(200, "text/html", SendHTML(HIGH)); //Active
    delay(1500);
    handle_OnConnect();
}

void handle_exitconfig()
{
    //Put a cute disconnect loading page here that delays a bit before disconnecting
    server.send(200, "text/plain", "Note to self: Remember to create the page that tells people the router is saving and quitting... :)");
    Serial.println("\nDisconnecting from AP now");
    WiFi.softAPdisconnect(true);
    config_ble = false;
}

void handle_NotFound()
{
    server.send(404, "text/plain", "Not found. Are you sure you have the right fishtank?");
}

/* Web Page Design */

String SendHTML(uint8_t active)
{

    ptr = "<!DOCTYPE html> <html>\n";
    ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
    ptr += "<title>Gate Control</title>\n";
    ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
    ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
    ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
    ptr += ".buttonsmall {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 18px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";

    ptr += ".button-on {background-color: #d74242;}\n";
    ptr += ".button-on:active {background-color: #d74242;}\n";
    ptr += ".button-off {background-color: #852b2b;}\n";
    ptr += ".button-off:active {background-color: #2c3e50;}\n";
    ptr += ".button-save {background-color: #429bd7;}\n";

    ptr += "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
    ptr += "</style>\n";

    if (active)
    {
        ptr += "<meta http-equiv='refresh' content='2;url=/'>\n";
    }

    ptr += "</head>\n";
    ptr += "<body>\n";
    ptr += "<h1>Tarentaal Gate Control</h1>\n";

    /* Toggle Gate*/
    if (active)
    {
        ptr += "<p>Pushing Button</p><a class=\"button button-off\" href=\"/\">Gate Opening</a>\n";
    }
    else
    {
        ptr += "<p>Button Released</p><a class=\"button button-on\" href=\"/toggle\">Toggle Gate</a>\n";
    }

    /*Save and Quit*/
    ptr += "<a class=\"buttonsmall button-save\" href=\"/exitconfig\">Save Settings and Quit</a>\n";

    /* Form and Button Part*/
    ptr += "<form action=\"/sendrssi\" method=\"POST\"><input type = \"number\" name = \"custom_in\" placeholder = \"Custom RSSI coming in\"><br>";
    ptr += "<input type = \"number\" name = \"custom_out\" style = \"border-color:green;\"placeholder = \"Custom RSSI going out\"><br>";
    ptr += "<br><input type = \"submit\" value = \"Enter\">";
    ptr += "</form>";

    /*Footers*/
    ptr += "<h6>Created by A. van Wijk, 2021</h6>\n";

    ptr += "</body>\n";
    ptr += "</html>\n";
    return ptr;
}

String refreshpageHTML()
{

    ptr = "<!DOCTYPE html> <html>\n";
    ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
    ptr += "<title>Gate Control</title>\n";
    ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; vertical-align: middle;}\n";
    ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
    ptr += ".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
    ptr += ".buttonsmall {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 18px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
    ptr += ".button-on {background-color: #d74242;}\n";
    ptr += ".button-on:active {background-color: #d74242;}\n";
    ptr += ".button-off {background-color: #852b2b;}\n";
    ptr += ".button-off:active {background-color: #2c3e50;}\n";
    ptr += ".button-save {background-color: #429bd7;}\n";
    ptr += "p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
    ptr += "</style>\n";

    ptr += "<meta http-equiv='refresh' content='2;url=/'>\n";

    ptr += "</head>\n";
    ptr += "<body>\n";

    ptr += "<h1>Value Saved!</h1>\n";

    ptr += "</body>\n";
    ptr += "</html>\n";
    return ptr;
}