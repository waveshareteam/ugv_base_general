// wifi ctrl functions.
// you can refer to this website below to upload a config file to ESP32 Flash.
// https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/

// libraries:
// #include <LittleFS.h>
// #include <WIFI.h>
// #include <ArduinoJson.h>

// you need to init Serial.
// bool InfoPrint = true;

// wifi config
// wifi mode on boot.
// 0: OFF (you need to use uart-command or upload a new wifiConfig.json to turn it on again)
// 1: AP (default mode as a brand new product)
// 2: STA
// 3: AP+STA (default mode after first wifi connection succeed)
byte WIFI_MODE_ON_BOOT = 1;
const char* sta_ssid = "none";
const char* sta_password = "none";
const char* ap_ssid = "UGV";
const char* ap_password = "12345678";

// true: change the WIFI_MODE_ON_BOOT to 3 when first STA mode succeed.
bool defaultModeToAPSTA = true;

// wifiConfig.yaml example:
// wifi_mode_on_boot:3
// sta_ssid:"WIFI_NAME"
// sta_ssid:"WIFI_PASSWORD"
// ap_ssid:"WIFI_NAME"
// ap_ssid:"WIFI_PASSWORD"
File wifiConfigYaml;


// other args:
unsigned long connectionStartTime;
unsigned long connectionTimeout = 15000;
byte WIFI_CURRENT_MODE = -1;
IPAddress localIP;
DynamicJsonDocument wifiDoc(256);
bool wifiConfigFound = false;


// update oled accroding to wifi settings.
void updateOledWifiInfo() {
  switch(WIFI_CURRENT_MODE) {
  case 0: 
    screenLine_0 = "AP: OFF";
    screenLine_1 = "ST: OFF";
    break;
  case 1:
    screenLine_0 = String("AP:") + ap_ssid;
    screenLine_1 = "ST: OFF";
    break;
  case 2:
    screenLine_0 = "AP: OFF";
    screenLine_1 = String("ST:") + localIP.toString();
    break;
  case 3:
    screenLine_0 = String("AP:") + ap_ssid;
    screenLine_1 = String("ST:") + localIP.toString();
    break;
  }
  oled_update();
}


// load the wifiConfig.json form Flash.
// the file name is wifiConfig.json in root path.
bool loadWifiConfig() {
	wifiConfigYaml = LittleFS.open("/wifiConfig.json", "r");
	if (wifiConfigYaml) {
		if (InfoPrint == 1) {Serial.println("/wifiConfig.json load succeed.");}
		
		String line = wifiConfigYaml.readStringUntil('\n');

		// parse the YAML file using ArduinoJson.
		deserializeJson(wifiDoc, line);

		// read configuration values.
		WIFI_MODE_ON_BOOT = wifiDoc["wifi_mode_on_boot"];
		sta_ssid = wifiDoc["sta_ssid"];
		sta_password = wifiDoc["sta_password"];
		ap_ssid = wifiDoc["ap_ssid"];
		ap_password = wifiDoc["ap_password"];

		if (InfoPrint == 1) {
			Serial.println(line);
		}

		wifiConfigYaml.close();
		wifiConfigFound = true;
		jsonInfoHttp.clear();
  	jsonInfoHttp["ip"] = "/wifiConfig.json load succeed.";
 		jsonInfoHttp["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
 		jsonInfoHttp["sta_ssid"] = sta_ssid;
 		jsonInfoHttp["sta_password"] = sta_password;
 		jsonInfoHttp["ap_ssid"] = ap_ssid;
 		jsonInfoHttp["ap_password"] = ap_password;
		return true;

	} else {
		if (InfoPrint == 1) {Serial.println("cound not found wifiConfig.json.");}
		wifiConfigFound = false;
		return false;
	}
}


// get the ip address.
IPAddress getIPAddress(byte inputMode) {
	localIP = WiFi.localIP();
	if (InfoPrint == 1) {
		Serial.print("IP: ");
		Serial.println(localIP.toString());
	}

	jsonInfoHttp.clear();
  jsonInfoHttp["ip"] = localIP.toString();
	return localIP;
}


// create a wifiConfig.json file
// from the args already be using.
bool createWifiConfigFileByStatus() {
	if (WIFI_MODE_ON_BOOT != 0 || WIFI_MODE_ON_BOOT != -1){
		wifiDoc.clear();
		wifiDoc["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
		wifiDoc["sta_ssid"] = sta_ssid;
		wifiDoc["sta_password"] = sta_password;
		wifiDoc["ap_ssid"] = ap_ssid;
		wifiDoc["ap_password"] = ap_password;

		File configFile = LittleFS.open("/wifiConfig.json", "w");
		if (configFile) {
			serializeJson(wifiDoc, configFile);
			configFile.close();
			if (InfoPrint == 1) {
				Serial.println("/wifiConfig.json created.");
			}
			jsonInfoHttp.clear();
  		jsonInfoHttp["info"] = "/wifiConfig.json created.";
			jsonInfoHttp["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
			jsonInfoHttp["sta_ssid"] = sta_ssid;
			jsonInfoHttp["sta_password"] = sta_password;
			jsonInfoHttp["ap_ssid"] = ap_ssid;
			jsonInfoHttp["ap_password"] = ap_password;
			return true;
		} else {
			jsonInfoHttp.clear();
  		jsonInfoHttp["info"] = "/wifiConfig.json open failed.";
			return false;
		}
	} else {
		jsonInfoHttp.clear();
  	jsonInfoHttp["info"] = "not for this wifi_mode_on_boot.";
		return false;
	}
}


// set wifi as AP mode.
bool wifiModeAP(const char* input_ssid, const char* input_password) {
	WiFi.disconnect();
	if (InfoPrint == 1) {Serial.println("wifi mode on boot: AP");}
	// WiFi.mode(WIFI_AP);
	WiFi.mode(WIFI_AP_STA);
	WiFi.softAP(input_ssid, input_password);
	if (InfoPrint == 1) {
		Serial.println("AP mode starts...");
		Serial.print("SSID: ");
		Serial.println(input_ssid);
		Serial.print("Password: ");
		Serial.println(input_password);
		Serial.println("AP Address: 192.168.4.1");
	}
	WIFI_CURRENT_MODE = 1;
	localIP = WiFi.localIP();
	ap_ssid = input_ssid;
	ap_password = input_password;

	updateOledWifiInfo();

	jsonInfoHttp.clear();
  jsonInfoHttp["info"] = "AP mode starts";
  jsonInfoHttp["ap_ssid"] = ap_ssid;
  jsonInfoHttp["ap_password"] = ap_password;

	return true;
}


// set wifi as STA mode.
bool wifiModeSTA(const char* input_ssid, const char* input_password) {
	WiFi.disconnect();
	if (InfoPrint == 1) {Serial.println("wifi mode on boot: STA");}
	// WiFi.mode(WIFI_STA);
	WiFi.mode(WIFI_AP_STA);
	WiFi.begin(input_ssid, input_password);
	connectionStartTime = millis();

	if (InfoPrint == 1) {Serial.println("STA mode starts: connecting to ");
					Serial.println(input_ssid);}
	while (WiFi.status() != WL_CONNECTED) {
		unsigned long currentTime = millis();
		if (InfoPrint == 1) {Serial.print(".");}
		delay(500);

		if (currentTime - connectionStartTime >= connectionTimeout) {
			WIFI_CURRENT_MODE = -1;
			if (InfoPrint == 1) {Serial.println(".");Serial.println("STA connection timeout.");}
			wifiModeAP(ap_ssid, ap_password);
			updateOledWifiInfo();

			jsonInfoHttp.clear();
			jsonInfoHttp["info"] = "STA connection timeout.";

			return false;
			break;
		}
	}

	if (InfoPrint == 1) {Serial.println(".");Serial.println("STA connection succeed.");}
	WIFI_CURRENT_MODE = 2;
	getIPAddress(WIFI_CURRENT_MODE);
	sta_ssid = input_ssid;
	sta_password = input_password;

	jsonInfoHttp.clear();
	jsonInfoHttp["info"] = "STA connection succeed.";
	jsonInfoHttp["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
	jsonInfoHttp["sta_ssid"] = sta_ssid;
	jsonInfoHttp["sta_password"] = sta_password;
	jsonInfoHttp["ap_ssid"] = ap_ssid;
	jsonInfoHttp["ap_password"] = ap_password;

	if (defaultModeToAPSTA && !wifiConfigFound) {
		WIFI_MODE_ON_BOOT = 3;
		if (InfoPrint == 1) {Serial.println("[default] wifi mode on boot: AP+STA");}
		jsonInfoHttp["info"] = "[default] wifi mode on boot: AP+STA";
		createWifiConfigFileByStatus();
	}
	updateOledWifiInfo();

	return true;
}


// set wifi as AP+STA mode.
bool wifiModeAPSTA(const char* input_ap_ssid, const char* input_ap_password, const char* input_sta_ssid, const char* input_sta_password) {
	WiFi.disconnect();
	if (InfoPrint == 1) {Serial.println("wifi mode on boot: AP+STA");}
	WiFi.mode(WIFI_AP_STA);
	WiFi.softAP(input_ap_ssid, input_ap_password);
	if (InfoPrint == 1) {
		Serial.println("AP/AP+STA mode starts...");
		Serial.print("AP SSID: ");
		Serial.println(input_ap_ssid);
		Serial.print("AP Password: ");
		Serial.println(input_ap_password);
		Serial.println("AP Address: 192.168.4.1");
	}
	ap_ssid = input_ap_ssid;
	ap_password = input_ap_password;
	
	WiFi.begin(input_sta_ssid, input_sta_password);
	connectionStartTime = millis();

	if (InfoPrint == 1) {Serial.print("STA/AP+STA mode starts: connecting to ");
					Serial.println(input_sta_ssid);}
	while (WiFi.status() != WL_CONNECTED) {
		unsigned long currentTime = millis();
		if (InfoPrint == 1) {Serial.print(".");}
		delay(500);

		if (currentTime - connectionStartTime >= connectionTimeout) {
			WIFI_CURRENT_MODE = -1;
			if (InfoPrint == 1) {Serial.println(".");Serial.println("STA connection timeout.");}
			wifiModeAP(ap_ssid, ap_password);
			updateOledWifiInfo();

			jsonInfoHttp.clear();
			jsonInfoHttp["info"] = "STA connection timeout.";

			return false;
			break;
		}
	}

	if (InfoPrint == 1) {Serial.println("STA connection succeed.");}
	WIFI_CURRENT_MODE = 3;
	getIPAddress(WIFI_CURRENT_MODE);
	sta_ssid = input_sta_ssid;
	sta_password = input_sta_password;
	if (defaultModeToAPSTA && !wifiConfigFound) {
		WIFI_MODE_ON_BOOT = 3;
		if (InfoPrint == 1) {Serial.println("[default] wifi mode on boot: AP+STA");}
		createWifiConfigFileByStatus();
	}
	updateOledWifiInfo();

	jsonInfoHttp.clear();
	jsonInfoHttp["info"] = "STA connection succeed.";
	jsonInfoHttp["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
	jsonInfoHttp["sta_ssid"] = sta_ssid;
	jsonInfoHttp["sta_password"] = sta_password;
	jsonInfoHttp["ap_ssid"] = ap_ssid;
	jsonInfoHttp["ap_password"] = ap_password;
	
	return true;
}


// disconnect wifi.
void wifiStop() {
	WiFi.disconnect();
	WIFI_CURRENT_MODE = 0;
	WiFi.mode(WIFI_AP_STA);
	updateOledWifiInfo();
}


// wifi mode on boot starts.
bool wifiModeOnBoot() {
	bool funcStatus = false;
	switch(WIFI_MODE_ON_BOOT) {
	case 0: 
		if (InfoPrint == 1) {
			Serial.println("wifi mode on boot: OFF");
		}
		funcStatus = true;
		WIFI_CURRENT_MODE = 0;
		WiFi.mode(WIFI_AP_STA);
		break;
	case 1:
		funcStatus = wifiModeAP(ap_ssid, ap_password);
		break;
	case 2:
		funcStatus = wifiModeSTA(sta_ssid, sta_password);
		break;
	case 3:
		funcStatus = wifiModeAPSTA(ap_ssid, ap_password, sta_ssid, sta_password);
		break;
	}
	return funcStatus;
}


// change the WIFI_MODE_ON_BOOT.
void configWifiModeOnBoot(byte inputMode) {
	WIFI_MODE_ON_BOOT = inputMode;
	if (InfoPrint == 1) {
		Serial.print("wifi_mode_on_boot: ");
		Serial.println(WIFI_MODE_ON_BOOT);
	}
	createWifiConfigFileByStatus();
}


// create a wifiConfig.json file
// from the args input.
void createWifiConfigFileByInput(byte inputMode, const char* inputApSsid, const char* inputApPassword, const char* inputStaSsid, const char* inputStaPassword) {
	WIFI_MODE_ON_BOOT = inputMode;
	wifiModeAPSTA(inputApSsid, inputApPassword, inputStaSsid, inputStaPassword);
	if (InfoPrint == 1) {
		Serial.print("wifi_mode_on_boot: ");
		Serial.println(WIFI_MODE_ON_BOOT);
	}
	createWifiConfigFileByStatus();
}


// wifi information feedback.
void wifiStatusFeedback() {
	wifiDoc["ip"] = localIP.toString();
	wifiDoc["rssi"] = WiFi.RSSI();
	serializeJson(wifiDoc, Serial);

	jsonInfoHttp.clear();
	jsonInfoHttp["ip"] = wifiDoc["ip"];
	jsonInfoHttp["rssi"] = wifiDoc["rssi"];
	jsonInfoHttp["wifi_mode_on_boot"] = WIFI_MODE_ON_BOOT;
	jsonInfoHttp["sta_ssid"] = sta_ssid;
	jsonInfoHttp["sta_password"] = sta_password;
	jsonInfoHttp["ap_ssid"] = ap_ssid;
	jsonInfoHttp["ap_password"] = ap_password;
	jsonInfoHttp["mac"] = thisMacStr;
}


// wifi init.
void initWifi() {
	loadWifiConfig();
	wifiModeOnBoot();
}