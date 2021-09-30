#include "Arduino.h"
//#include <ArduinoBLE>
// Unused Libraries

/* #include <kalman/KalmanFilterBase.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/ExtendedKalmanFilter.hpp>
//#include "kalman.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <chrono>
#include <Husarnet.h>
*/

#include <cmath>
#include <algorithm>
#include <iostream>
#include <vector>
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
 *   Web Integration Thanks to HusarNet and https://www.hackster.io/donowak/internet-controlled-led-strip-using-esp32-arduino-2ca8a9
 * 
 * NOTE: Getting lots of +56 values means it cannot find the device :)
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

/*
const char* hostName = "tarentaal";  
const char* husarnetJoinCode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/wKrHCwZGrVdpHiDqS7twpf";
const char* dashboardURL = "default";
*/


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
const char *ssid_client = "Graves Into Gardens";        // Enter SSID here
const char *password_client = "throughchristalone"; //Enter Password here

const char *ssid_host = "Tarentaal";        // Enter SSID here
const char *password_host = "birdsfordays"; //Enter Password here
String loc = "IN";

/* Put IP Address details */
IPAddress local_ip(10, 1, 1, 1);
IPAddress gateway(10, 10, 10, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);

/* Enter Known BLE Device Mac Addresses */
//Owlet1 - db:17:35:a3:4c:27
// Peep - dc:0d:69:a7:f7:9a
String knownBLEAddresses[] = {"dc:0d:69:a7:f7:9a"};

/* Constants */
//int RSSI_THRESHOLD = -90;     // Is overwritten by CUSTOM_IN and CUSTOM_OUT dynamically
//int RSSI_CONFIG = -10;        // For starting the Webserver - IMPOSSIBLE for now.
//uint8_t RSSI_CUSTOM_IN = 88;  //RSSI trigger from IN going OUT The EEPROM value will overwrite this.
//uint8_t RSSI_CUSTOM_OUT = 88; //RSSI trigger from OUT coming IN The EEPROM value will overwrite this.

float DIST_THRESHOLD = 1.5;
float DIST_CUSTOM_IN = 1.2;
float DIST_CUSTOM_OUT = 10;
float DIST_CONFIG = 0.1;
//Note, DIST_CONFIG is disabled at the moment.

bool device_found = false;
bool wifibool = false;
bool config_ble = false;
bool gate_delay = false; // Set as true so we don't need to wait for timedelay on first boot. Gate is primed after restart.

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
uint8_t LED_BUILTIN = 2; // Just for extra visual cues that it IS actually triggering.

// Window shall be LE to interval

uint32_t scanTime = 1;   //Duration of each scan in seconds 2 1100 1099
uint16_t interval = 200; //the intervals at which scanning is actively taking place in milliseconds
uint16_t window = 200;   //the window of time after each interval which is being scanned in milliseconds
//500 here is the best compromise. lower for better wifi, higher for better ble.
//Lower scantiems are better for lower latencies, but higher scantimes are better for consistency. If the beacon is far away, a higher scantime is preferred.
uint16_t configcounter = 0;

//double distance = 0;
float wijkstra_dist =100;

unsigned long previoustime = 0;
uint32_t time_delay = 2000; // Ideally 15 sec, in ms. 20000
unsigned long currenttime;
int pos = 0;
int scannumber = 0;
int dataCounter = 0;
int incomingData[50] = {0};
int prev = 0;
int state = 0; // 0 - inside | 1 - outside
int rssi = -200;
int rssiprevious = -200;
int keyrssi = -200;
int med =-200;
int kal =-200;
int sor =-200;
long last_discovered = 200;
int ctr = 1;
String ptr;
String str;
String str3;
BLEScan *pBLEScan; //ble pointer
std::string addressReturn;

unsigned long blescantime = 0;

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


std::vector<int> rssi_hist{1, 1, 1, 1, 1};
std::vector<int> med_hist{-1, -1, -1, -1, -1};
std::vector<int> sdor_med_hist{-1, -1, -1, -1, -1};
std::vector<int> movingavg{0, 0, 0, 0, 0};
std::vector<float> dist_hist{15,15,15,15,15};



/* ------------------------------------------- End of Constant Initialization --------------------------------------------- */

/* BLE Function Code */

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        // Serial.println("Function For Loop Start...");
        for (int i = 0; i < (sizeof(knownBLEAddresses) / sizeof(knownBLEAddresses[i])); i++)
        {

            //TEST
            

            //Compare Each incoming signal with my code above and if it matches, set a flag that the key is within range.
            str = advertisedDevice.toString().c_str();
            //pos = str.indexOf("Address: "); // Note that we can also require a different attribute such as name instead of address to double vertify
            //This was when I was still analyzing hte whole string and doing needless work...
            //str3 = str.substring(pos + 9, pos + 26);
            //Serial.println("In Loop...");
            addressReturn = advertisedDevice.getAddress().toString();

            if ((strcmp(addressReturn.c_str(), knownBLEAddresses[i].c_str()) == 0))
            {
                //THIS IS THE MOST IMPORTANT LINE IN THIS WHOLE FILE
                advertisedDevice.getScan()->stop(); //Potentially include a timeout so that a MINIMUM of 200ms is required before a next scan (?)

                

                //    Serial.printf("Device MATCHES %d\n", rssi);
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

    

    DIST_CUSTOM_IN = EEPROM.read(0); // Note that these are UNSIGNED 8 bit ints, 0 - 255 range.
    DIST_CUSTOM_OUT = EEPROM.read(1);
    DIST_THRESHOLD = DIST_CUSTOM_IN;


    /*
    RSSI_CUSTOM_IN = EEPROM.read(0); // Note that these are UNSIGNED 8 bit ints, 0 - 255 range.
    RSSI_CUSTOM_OUT = EEPROM.read(1);
    RSSI_THRESHOLD = -RSSI_CUSTOM_IN;

    */
    if (!myOLED.begin(SSD1306_128X64))
        while (1)
            ; // In case the library failed to allocate enough RAM for the display buffer...
    myOLED.setFont(SmallFont);

    /* BLE Initialization */
    Serial.println("Scanning...");
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();                                           //create new scan
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); //Init Callback Function
    pBLEScan->setActiveScan(true);                                             //active scan uses more power, but get results faster
    pBLEScan->setInterval(interval);                                           // set Scan interval // TRY 128
    pBLEScan->setWindow(window);                                                // less or equal setInterval value // TRY 16
    Serial.println("Finish Initialize");

}

int meanx(std::vector<int> data) // Calculates mean of rssi data
{
    int k;
    float sum =0;
    for(k=0;k<5;k++)
    {
        sum += data[k];    
    }
    
    return (sum) /data.size();
}

int meanx(std::vector<float> data) //Calculates mean of distance data
{
    int k;
    float sum =0;
    for(k=0;k<5;k++)
    {
        sum += data[k];    
    }
    
    return (sum) /data.size();
}

int SDOR(std::vector<int> hist_SDOR) // SDOR for rssi data
{
    //Calculate mean
    int mean = meanx(rssi_hist);

    //Calculate std Deviation.
    int k;
    float sum =0;
    for(k=0;k<5;k++)
    {
        //Serial.printf("hist_SDOR [%d]: %d\n", k, hist_SDOR[k]);
        sum += (pow(abs(hist_SDOR[k] - mean),2));
        //Serial.println(sum);   
    }
    int SD = sqrt(sum/5);
    //Serial.printf("SD: %d\n", SD);
    //Remove the elements that fall outside the permitted range of 2 x std deviation
    int limit =5;
     for(k=0;k<limit;k++)
    {
          if ((hist_SDOR[k] < (mean - 2*SD)))
          {
              hist_SDOR.erase(hist_SDOR.begin()+k);
              //Serial.printf("Erase:%d, %d\n", hist_SDOR[k],(mean-2*SD));
              limit = limit -1; //This prevents trying to access bad memory
          }
    }

    //Calculate new average

    sum =0;
    for(k=0;k<limit;k++)
    {
        sum += hist_SDOR[k];    
    }


    return 0.25*(sum/(limit)) + rssi*0.75;
}

int median(std::vector<int> hist_med) // Calculates the median of rssi_hist, returns median, and stores history in med_hist
{
    //Add newest RSSI, discard oldest.
    //for(std::size_t i = 0; i < rssi_hist.size(); ++i) {
    //std::cout << rssi_hist[i] << "\n";
    //}

    std::sort(hist_med.begin(), hist_med.end()); //Sort from most negative to least negative. ie: {-111, -110, -100}
    // for(std::size_t i = 0; i < hist_med.size(); ++i) {
    //std::cout << hist_med[i] << "\n";
    //}

    //Serial.printf("Sorted: %d, %d, %d\n", rssi_hist[0], rssi_hist[1], rssi_hist[2]);
    //Update Med History
    med_hist.pop_back();
    med_hist.insert((med_hist.begin()), hist_med[2]);

    return hist_med[2]; //Return middle value.
}

int kalvin(std::vector<int> hist_kal) //kalman for rssi
{
    int k;
    //Runs through the whole rssi_hist buffer and tries to calculate the 'true' value.
    std::vector<float> p{0, 0, 0, 0, 0,0,0,0};
    std::vector<float> p_update{0, 0, 0, 0, 0};
    std::vector<float> x{0, 0, 0, 0, 0,0,0,0};
    std::vector<float> x_update{0, 0, 0, 0, 0};
    
    x[0]  = meanx(rssi_hist); // Our Initial Estimate of the RSSI, in this case the mean of the past 5 values.
    p[0] = 15*15; //How uncertain we are in our predictions
    float q = 0.005;
    
    /* Setup */
    p[1] = p[0] + q;
    x[1] = x[0];
    
    
    float r_1 = 15*15; // The measurement error squared. In RSSI.
    float K_gain; // The Kalman Gain

    /* Begin Iterations - these do not change each iteration */
    
    
    for (k = 0; k < 5; k++)
    {
        //Serial.printf("Iteration %d, Predict: %f\n",k,x[k+3]);
        
        // Step 2 - Update -
        K_gain = p[k+1] / (p[k+1] + r_1);
        //Serial.printf("Iteration %d, K: %f\n",k,K_gain);
        x_update[k] = x[k + 1] + K_gain * (hist_kal[k] - x[k + 1]);
        //Serial.printf("Iteration %d, x_update %f\n",k,x_update[k]);
        p_update[k] = (1 - K_gain) * p[k + 1];
        //Serial.printf("Iteration %d, p_update %f\n",k,p_update[k]);

        // Step 3 - Predict - 
        x[k + 2] = x_update[k];
        p[k + 2] = p_update[k] + q;
        //Serial.printf("Iteration %d, Predict: %f\n",k,x[k+2]);
        
    }
    
    
    return (int8_t)(x[6]);
    //return 1;
}

float kalvin(std::vector<float> dist_kal) //kalman specifically for distance
{
    int k;
    //Runs through the whole rssi_hist buffer and tries to calculate the 'true' value.
    std::vector<float> p{0, 0, 0, 0, 0,0,0,0};
    std::vector<float> p_update{0, 0, 0, 0, 0};
    std::vector<float> x{0, 0, 0, 0, 0,0,0,0};
    std::vector<float> x_update{0, 0, 0, 0, 0};
    
    x[0]  = meanx(dist_kal); // Our Initial Estimate of the distance, in this case the mean of the past 5 values.

    //Serial.printf("X[0]: %f\n", x[0]);
    p[0] = 0.4*0.4; //How uncertain we are in our predictions. Difference between TRUE value and PREDICTED value.
    float q = 0.005;
    
    /* Setup */
    p[1] = p[0] + q;
    x[1] = x[0];

    //Serial.printf("x[1]: %f\n", x[1]);
    //Serial.printf("p[1]: %f\n", p[1]);
    
    
    float r_1 = 0.75*0.75; // The measurement error squared. In RSSI.
    float K_gain; // The Kalman Gain

    /* Begin Iterations - these do not change each iteration */
    
    
    for (k = 0; k < 5; k++)
    {
    
        // Step 2 - Update -
        K_gain = p[k+1] / (p[k+1] + r_1);
        //Serial.printf("Iteration %d, K: %f\n",k,K_gain);
        x_update[k] = x[k + 1] + K_gain * (dist_kal[k] - x[k + 1]);
        //Serial.printf("Iteration %d, x_update %f\n",k,x_update[k]);
        p_update[k] = (1 - K_gain) * p[k + 1];
        //Serial.printf("Iteration %d, p_update %f\n",k,p_update[k]);

        // Step 3 - Predict - 
        x[k + 2] = x_update[k];
        p[k + 2] = p_update[k] + q;
        //Serial.printf("Iteration %d, Predict: %f\n",k,x[k+2]);
        
    }
    
    return (x[6]);
}

void hist_update(int keyrssi) //Update rssi_history vector
{
    rssi_hist.pop_back();
    rssi_hist.insert((rssi_hist.begin()), keyrssi);
    return;
}

void dist_hist_update(float dist) // Update distance_history vector
{
    dist_hist.pop_back();
    dist_hist.insert((dist_hist.begin()), dist);
    return;
}

float est_dist(int RSSI) //Estimate distance given RSSI using a model.
{
//0db - -52
// 4db -50
// -4db -58
//float dist = pow(10,( (-75 - RSSI)/(10*3.8) )); // The Emperical Model

//Room Model - Double Exp
//float dist = 0.00002365*exp(-0.1329*RSSI)+33.71*exp(0.07747*RSSI);

//Lab Model - Single Exp - Positive RSSI
//float dist = 0.00001935*exp(0.1337*RSSI*-1);

//Lab Model - Double Exp - Neg RSSI
float dist = 0.01718*(exp(-0.03644*RSSI)) + 0.000001143*(exp(-0.1611*RSSI));
return dist;
}

float wijkstra(){

//Preprocessing Section ----------------

//Apply Median
    //Already applied in main loop.

//Apply SDOR to med_hist
//sdor_med_hist.pop_back();
//sdor_med_hist.insert((sdor_med_hist.begin()), SDOR(med_hist)); //Feed med_hist into SDOR, store in vector. Update each iteration.

//Distance Estimation ------------------
    //Take the mean of the last 5 SDOR values, feed into the distance estimation algorithm.
    //float distance = est_dist((meanx(sdor_med_hist)));
    //Serial.printf("SDOR Output: %d\n", SDOR(med_hist));
    float distance = est_dist(SDOR(med_hist));

    //Serial.printf("Raw Distance: %f\n", distance);

    dist_hist_update(distance); //Updates global variable dist_hist which feeds kalvin

//Apply Kalman to distance values.
    float final_est =  kalvin(dist_hist);

//Result
//Serial.printf("Output: %f\n", final_est);
return final_est;



}

void loop()
{
    /* Do not Remove */
    hist_update(keyrssi); //Update history of past keyrssi values
    med = median(rssi_hist); //Calculates the median of rssi_hist, returns median, and stores history in med_hist
    wijkstra_dist = wijkstra();
    /*End */

    sor = SDOR(rssi_hist); // Calculates SDOR, returns only single value estimate.
    kal = kalvin(rssi_hist); //Initial guess is the average of past 5 samples.
    //dist_hist_update(est_dist(kal));
    //kal_dist = kalvin(dist_hist); //Smooth Distance. RAW, KAL, Dist, KAL

    Serial.printf("%f, %f\n", wijkstra_dist, est_dist(rssi));

    //Serial.printf("%f", wijkstra());
    //History of Median Output Measurements.
    
    //Serial.printf("%d,%d,%d,%d\n",(int8_t)(keyrssi), (int8_t)(med), (int8_t)(kal), (int8_t)(sor)); //rssi and median
    //Serial.printf("Distance: %f,%f,%f,%f\n", est_dist(keyrssi), est_dist(med),est_dist(kal),est_dist(sor));
    


    rssiprevious = rssi;
    currenttime = millis();

    
    /* Line for Plotting */

    //Serial.printf("%d,%d,%d,%d\n",(int8_t)(rssi), (int8_t)(med), (int8_t)(kal), (int8_t)(sor)); //rssi and median
    
    
    
    
    
    //Serial.printf("%d,%d\n",(int8_t)(rssi), (int8_t)(med)); //rssi and median

    /*Time keeping function used to prevent multiple gate retriggers */
    if (currenttime - previoustime >= time_delay) // time delay is 15 sec ideally.
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
        digitalWrite(LED_BLE, LOW);
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
        digitalWrite(LED_WIFI, LOW);
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

        /* OLED DISPLAY UPDATE */
        myOLED.clrScr();
        myOLED.print("Update Freq:", LEFT, 8);
        myOLED.printNumI(last_discovered, RIGHT, 8);
        myOLED.print("Filtered RSSI:", LEFT, 24);
        myOLED.printNumF(SDOR(med_hist), 3, RIGHT, 24); // 0.123
        myOLED.print("Wijkstra Dist:", LEFT, 40);
        myOLED.printNumF(wijkstra_dist, 3, RIGHT, 40);
        myOLED.print("Location:", LEFT, 56);
        myOLED.print(loc, RIGHT, 56);
        myOLED.update();


        // ORIGINAL BLEScanResults foundDevices = pBLEScan->start(scanTime, false); // This Takes 1 Second.
        //Serial.println("StartScan");
        pBLEScan->start(scanTime, false); // This Takes 1 Second.
       
        BLEScanResults foundDevices  = pBLEScan->getResults();
       
        
        //Needed if you want simultaneous wifi to work.
        
        //server.handleClient();
        //Serial.println("BLE End Scanresults\n");
        //Serial.println(foundDevices.getCount());

        for (int k = 0; k < foundDevices.getCount(); k++)
        {
            //Serial.printf("Loop of FoundDevices: %d\n", k);
            BLEAdvertisedDevice device = foundDevices.getDevice(k);
            rssi = device.getRSSI();
            if (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0) // Restricts to only KNOWN mac addresses. 0 when identical.
            {
                /*BLE Beacon has been found and is within range. */
                
                //Serial.println();
                //Serial.println(millis() - blescantime); // The time between sucessive updates
                last_discovered = millis() - blescantime;
                
                /*
                movingavg.pop_back();
                movingavg.insert((movingavg.begin()), last_discovered);
                
                
                ctr = 0;
                for (int i=0; i< 5; i++) // Resets moving avg to only use the last 10 values.
                {
                ctr += movingavg[i];
                }
                Serial.printf("Moving Avg: %d\n", (ctr/5));

                */
                blescantime = millis();

            ///Serial.printf("Key found, keyrssi: (%d) rssi of Current Threshhold: %d \n", rssi, RSSI_THRESHOLD);
                //Serial.printf("%d\n", rssi);

                keyrssi = rssi;

                //Serial.printf("RSSIA %d\n", rssi);
                //Convert RSSI to Distance
                // Source : https://iotandelectronics.wordpress.com/2016/10/07/how-to-calculate-distance-from-the-rssi-value-of-the-ble-beacon/
                //temp = -70 - rssi;
                //temp = temp / 40; // The 40 is arbitrary and must be tuned.
                //distance = pow(10, temp);
                //Serial.printf("Algebriac Distance of Key is (Approx) %f m \n", distance);

            }
            
            //Serial.printf("Med: %d, Thresh: %d\n", med, RSSI_THRESHOLD);
            //Serial.printf("Device found: %d", (device_found == true));
            //Serial.println((strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0));


            /* Original RSSI Version
            if ((kal > RSSI_THRESHOLD) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
           // if ((med > RSSI_THRESHOLD))
            {
                prev = 1; // Light was turned on previously in this loop
                //Serial.printf("Prev has been triggered. %d, %d", rssi, RSSI_THRESHOLD);
            }
            if ((keyrssi > RSSI_CONFIG) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
            {
                configcounter++;
                Serial.printf("Config Counter: %d\n", configcounter);
            }

            */

           /* Updated Distance Version */

           if ((wijkstra_dist < DIST_THRESHOLD) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
            {
                 // if ((med > RSSI_THRESHOLD))
                prev = 1; // Light was turned on previously in this loop
                //Serial.printf("Prev has been triggered. %d, %d", rssi, RSSI_THRESHOLD);
            }

            if ((wijkstra_dist < DIST_CONFIG) && (device_found == true) && (strcmp(device.getAddress().toString().c_str(), knownBLEAddresses[0].c_str()) == 0))
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
                loc = "OUT";
                //time_delay = 60000 // 60 second delay after exiting
                device_found = false; // Reset device found, so it needs to be triggered again.
                prev = 0;
                state = 1; // Assume now outside
                //RSSI_THRESHOLD = -RSSI_CUSTOM_OUT;
                DIST_THRESHOLD = DIST_CUSTOM_OUT;
                gate_delay = false;
            }
            
            if (state == 1 && gate_delay == true) // Assume state = 1 so I am outside.
            {
                Serial.println("Triggered, coming IN");
                triggerGate(1500);
                loc = "IN";
                //time_delay = 45000 // 60 second delay after exiting
                device_found = false; // Reset device found, so it needs to be triggered again.
                prev = 0;
                state = 0; // Assume now inside
                //RSSI_THRESHOLD = -RSSI_CUSTOM_IN;
                DIST_THRESHOLD = DIST_CUSTOM_IN;
                gate_delay = false;
            }
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
        myOLED.print("Custom DIST IN:", LEFT, 24);
        myOLED.printNumF(DIST_CUSTOM_IN, 3, RIGHT, 24);
        myOLED.print("Custom DIST OUT:", LEFT, 40);
        myOLED.printNumF(DIST_CUSTOM_OUT, 3, RIGHT, 40);
      
        myOLED.update();

        if (linenumber == 0) //On the first line of OLED Display
        {
            //Consdier including a flash indicator here.
            if (b_up.numberKeyPresses > 0)
            {
                DIST_CUSTOM_IN += 0.25;
                delay(25);
                b_up.numberKeyPresses = 0;
            }

            if (b_down.numberKeyPresses > 0)
            {
                DIST_CUSTOM_IN -= 0.25;
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
                DIST_CUSTOM_OUT += 0.25;
                delay(25);
                b_up.numberKeyPresses = 0;
            }

            if (b_down.numberKeyPresses > 0)
            {
                DIST_CUSTOM_OUT -= 0.25;
                delay(25);
                b_down.numberKeyPresses = 0;
            }

            if (b_enter.numberKeyPresses > 0)
            {
                linenumber = 0;
               
                
                EEPROM.write(0, DIST_CUSTOM_IN);
                EEPROM.write(1, DIST_CUSTOM_OUT);
                EEPROM.commit();
                Serial.println("State saved in flash memory:");
                Serial.printf("Custom IN: %f\n", DIST_CUSTOM_IN);
                Serial.printf("Custom OUT: %f\n", DIST_CUSTOM_OUT);
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
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_WIFI, LOW);
    digitalWrite(LED_BLE, LOW);
    delay(delaytime);
    digitalWrite(LED_TRIGGER, LOW);
    digitalWrite(LED_BUILTIN, LOW);
}

void wifi_station()
{
    WiFi.softAP(ssid_host, password_host);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    Serial.println("Wifi Hosted..!");
}

void wifi_client()
{
    /* Enable Station Mode Where it connects to a local Wifi */

    
    WiFi.begin(ssid_client, password_client);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected..!");
    Serial.print("Got IP: ");  Serial.println(WiFi.localIP());
    
}


void wifi_init() /* Wifi Intialization - Runs Once. */
{

    digitalWrite(LED_BLE, LOW);
    digitalWrite(LED_WIFI, HIGH);

    wifi_station();
    //wifi_client();

    myOLED.clrScr();
    //myOLED.print("Wifi Innit.", LEFT, 8);
    myOLED.print("Wifi Initialized", CENTER, 24);
    //myOLED.printNumI(RSSI_CUSTOM_IN, RIGHT, 24);
    //myOLED.print("Custom RSSI OUT:", LEFT, 40);
    //myOLED.printNumI(RSSI_CUSTOM_OUT, RIGHT, 40);
    myOLED.update();

    gate_delay = false;
    delay(150); // potential delay to avoid crashing.

    /* Husarnet to allow for internet CLI
    Husarnet.selfHostedSetup(dashboardURL);
    Husarnet.join(husarnetJoinCode, hostName);
    Husarnet.start();
    delay(5000);
    */
   
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
        DIST_CUSTOM_IN = abs(server.arg("custom_in").toInt());
    if (server.arg("custom_out") != "")
        DIST_CUSTOM_OUT = abs(server.arg("custom_out").toInt());

    EEPROM.write(0, DIST_CUSTOM_IN);
    EEPROM.write(1, DIST_CUSTOM_OUT);
    EEPROM.commit();
    delay(20); // Just in case
    Serial.println("State saved in flash memory:");
    Serial.printf("Custom IN: %f\n", DIST_CUSTOM_IN);
    Serial.printf("Custom OUT: %f\n", DIST_CUSTOM_OUT);
    server.send(200, "text/html", refreshpageHTML());
}

void handle_toggleGate()
{
    //triggerGate(1500); - Replace Later
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
    server.send(404, "text/plain", "Not found. Are you sure you're surfing the right web?");
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