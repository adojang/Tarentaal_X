#include <Arduino.h>

/*
 *  BLE Bluetooth Beacon, Adapted from Arduino Examples
 *  Adriaan van WIjk
 *  Last Mod:  Jan 2021
 *
 *  This program is meant for a ESP32 board which will search for BLE devices. It then matches a BLE device
 *  with a set of known MAC address (or other identifiable data) and if it matches, and is in range, will function as
 *  a trigger that can allow other applications, such as opening a gate, for example.
 *
 *  The Wifi Addition is meant to allow visitors to connect to the local AP and by accessing 192.168.1.25
 *  open and close the gate. Based on https://lastminuteengineers.com/creating-esp32-web-server-arduino-ide/
 *
 *  To Do:
 *
 *  - Multiple users, profiles, use EEPROM, remember who's in and out etc. etc.
 *  - Build a watchdog that will reset the ESP if something goes terribly wrong.
 * 
 */

#include <WiFi.h>
#include <WebServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <EEPROM.h>

#include <iostream>
#include <string>

//Function Declearations

void wifi_init();
void triggerGate(uint16_t delaytime);
void handle_OnConnect();
void handle_sendrssi();
void handle_toggleGate();
void handle_exitconfig();
void handle_NotFound();
String SendHTML(uint8_t active);
String refreshpageHTML ();


 /* Put your SSID & Password */
const char* ssid = "Tarentaal";  // Enter SSID here
const char* password = "birdsfordays";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192, 168, 1, 25);
IPAddress gateway(192, 168, 1, 25);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

/* Enter Known BLE Device Mac Addresses */
String knownBLEAddresses[] = {"db:17:35:a3:4c:27"};

/* Constants */
int RSSI_THRESHOLD = -88;   // Is overwritten by CUSTOM_IN and CUSTOM_OUT dynamically
int RSSI_CONFIG = -50;      // For starting the Webserver
uint8_t RSSI_CUSTOM_IN = 88;   //The EEPROM value will overwrite this.
uint8_t RSSI_CUSTOM_OUT = 88;  //The EEPROM value will overwrite this.

bool device_found = false;
bool wifibool = false;
bool config_ble = false;
bool gate_delay = true; // Set as true so we don't need to wait for timedelay on first boot. Gate is primed after restart.

uint8_t LED_BUILTIN = 2;
uint32_t scanTime = 2; //Duration of each scan in seconds
uint16_t interval = 1100; //the intervals at which scanning is actively taking place in milliseconds
uint16_t window = 1099; //the window of time after each interval which is being scanned in milliseconds
uint16_t configcounter = 0;
double distance = 0;
double temp;

unsigned long previoustime = 0;
const long time_delay = 15000;
unsigned long currenttime;
int pos = 0;
int scannumber = 0;
int i = 0;
int k;
int prev = 0;
int state = 0; // 0 - inside | 1 - outside
int rssi = -111;
int keyrssi = -111;
String ptr;
String str;
String str3;
BLEScan* pBLEScan; //ble pointer

/* BLE Function Code */

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {

        for (int i = 0; i < (sizeof(knownBLEAddresses) / sizeof(knownBLEAddresses[i])); i++)
        {
            //Compare Each incoming signal with my code above and if it matches, set a flag that the key is within range.
            str = advertisedDevice.toString().c_str();
            pos = str.indexOf("Address: "); // Note that we can also require a different attribute such as name instead of address to double vertify
            str3 = str.substring(pos + 9, pos + 26);

            if ((strcmp(str3.c_str(), knownBLEAddresses[i].c_str()) == 0))
            {
                //Serial.println("Device MATCHES");
                //Serial.println(str3.c_str());
                //Serial.println(knownBLEAddresses[i].c_str());
                device_found = true;                              //flag that is set that indicates I am within BLE range
            }
            else
            {
                //Serial.println("Device DOES NOT MATCH");
                //Serial.println(str3.c_str());
                //Serial.println(knownBLEAddresses[i].c_str());
            }
            scannumber++;
        }
        //Serial.println("*************END FOR LOOP**************");
    }
};

void setup() {
    EEPROM.begin(2);
    Serial.begin(115200); //Enable UART on ESP32
    pinMode(LED_BUILTIN, OUTPUT);
    RSSI_CUSTOM_IN = EEPROM.read(0); // Note that these are UNSIGNED 8 bit ints, 0 - 255 range.
    RSSI_CUSTOM_OUT = EEPROM.read(1);

    /* BLE Initialization */
    Serial.println("Scanning...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan(); //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); //Init Callback Function
    pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
    pBLEScan->setInterval(interval); // set Scan interval // TRY 128
    pBLEScan->setWindow(window);  // less or equal setInterval value // TRY 16

}
void loop() {
    currenttime = millis();

    //Time keeping function
    if (currenttime - previoustime >= time_delay)
    {
        previoustime = currenttime;
        gate_delay = true;
    }

    //Config flag function
    if (configcounter >= 2)
    {
        Serial.println("Disable BLE");
    configcounter = 0;
    config_ble = true;
    wifibool = false;
    }

    //Start of the bluetooth loop
    if (!config_ble)
    {
        //BLE Functionality
        BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
        for (k = 0; k < foundDevices.getCount(); k++)
        {
            BLEAdvertisedDevice device = foundDevices.getDevice(k);
            rssi = device.getRSSI();
            if (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0)
            {
                Serial.printf("Key found, rssi: (%d) rssi of Current Threshhold: %d \n", rssi, RSSI_THRESHOLD);
                keyrssi = rssi;

                //Convert RSSI to Distance

                temp = (-70 - keyrssi) / (10 * 2);
                distance = pow(10, temp);
                Serial.printf("Distance of Key is (Approx) %d m \n", (int) distance);

            }

            if ((rssi > RSSI_THRESHOLD) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
            {
                prev = 1; // Light was turned on previously in this loop
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
            
            if (state == 1 && gate_delay == true)// Assume state = 1 so I am outside.
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
        pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory      
    }
    else
    { 
        /* Begin the Wifi Server*/

        if (!wifibool) wifi_init(); //initialize wifi
        server.handleClient(); // handle the wifi page requests.
    }
}

void wifi_init()
{
    /* Wifi Intialization */
    triggerGate(200);
    delay(200);
    triggerGate(200);
    delay(200);
    triggerGate(200);

    WiFi.softAP(ssid, password);
    delay(2000); // Important so it doesn't crash... Probably
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
/* Functions Library: */
void triggerGate(uint16_t delaytime)
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delaytime);
    digitalWrite(LED_BUILTIN, LOW);
}

/* Web Handling Functions */

void handle_OnConnect() {
    server.send(200, "text/html", SendHTML(LOW)); //Not Active
    digitalWrite(LED_BUILTIN, LOW);
}

void handle_sendrssi() {

    if(server.arg("custom_in") != "") RSSI_CUSTOM_IN = abs(server.arg("custom_in").toInt());
    if (server.arg("custom_out") != "") RSSI_CUSTOM_OUT = abs(server.arg("custom_out").toInt());

    EEPROM.write(0, RSSI_CUSTOM_IN);
    EEPROM.write(1, RSSI_CUSTOM_OUT);
    EEPROM.commit();
    delay(20); // Just in case
    Serial.println("State saved in flash memory, using -90");
    server.send(200, "text/html", refreshpageHTML());
 
}

void handle_toggleGate()
{
    digitalWrite(LED_BUILTIN, HIGH);
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

void handle_NotFound() {
    server.send(404, "text/plain", "Not found. Are you sure you have the right fishtank?");
}


//Web Page Design

String SendHTML(uint8_t active) {

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

    if(active)
    {
       ptr +="<meta http-equiv='refresh' content='2;url=/'>\n";
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
    ptr += "<input type = \"number\" name = \"custom_out\" placeholder = \"Custom RSSI going out\"><br>";
    ptr += "<br><input type = \"submit\" value = \"Enter\">";
    ptr += "</form>";

    /*Footers*/
    ptr += "<h6>Created by A. van Wijk, 2021</h6>\n";

    ptr += "</body>\n";
    ptr += "</html>\n";
    return ptr;
}

String refreshpageHTML () {

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