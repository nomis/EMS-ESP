/*
 * MyESP - my ESP helper class to handle Wifi, MQTT and Telnet
 * 
 * Paul Derbyshire - December 2018
 * Version 1.1 - Feb 22 2019. Added support for ESP32
 *
 * Ideas borrowed from Espurna https://github.com/xoseperez/espurna
 */

#include "MyESP.h"

// constructor
MyESP::MyESP() {
    _app_hostname = strdup("MyESP");
    _app_name     = strdup("MyESP");
    _app_version  = strdup("1.0.0");

    _boottime     = strdup("unknown");
    _load_average = 100; // calculated load average

    _telnetcommand_callback = NULL;
    _telnet_callback        = NULL;

    _fs_callback          = NULL;
    _fs_settings_callback = NULL;

    _helpProjectCmds       = NULL;
    _helpProjectCmds_count = 0;

    _use_serial                = false;
    _mqtt_host                 = NULL;
    _mqtt_password             = NULL;
    _mqtt_username             = NULL;
    _mqtt_retain               = false;
    _mqtt_keepalive            = 300;
    _mqtt_will_topic           = NULL;
    _mqtt_will_online_payload  = NULL;
    _mqtt_will_offline_payload = NULL;
    _mqtt_base                 = NULL;
    _mqtt_topic                = NULL;
    _mqtt_qos                  = 0;
    _mqtt_reconnect_delay      = MQTT_RECONNECT_DELAY_MIN;

    _wifi_password  = NULL;
    _wifi_ssid      = NULL;
    _wifi_callback  = NULL;
    _wifi_connected = false;

    _suspendOutput = false;
}

MyESP::~MyESP() {
    end();
}

// end
void MyESP::end() {
    SerialAndTelnet.end();
    jw.disconnect();
}

// general debug to the telnet or serial channels
void MyESP::myDebug(const char * format, ...) {
    if (_suspendOutput)
        return;

    va_list args;
    va_start(args, format);
    char test[1];

    int len = ets_vsnprintf(test, 1, format, args) + 1;

    char * buffer = new char[len];
    ets_vsnprintf(buffer, len, format, args);
    va_end(args);

    SerialAndTelnet.println(buffer);

    delete[] buffer;
}


// for flashmemory. Must use PSTR()
void MyESP::myDebug_P(PGM_P format_P, ...) {
    if (_suspendOutput)
        return;

    char format[strlen_P(format_P) + 1];
    memcpy_P(format, format_P, sizeof(format));

    va_list args;
    va_start(args, format_P);
    char test[1];
    int  len = ets_vsnprintf(test, 1, format, args) + 1;

    char * buffer = new char[len];
    ets_vsnprintf(buffer, len, format, args);

    va_end(args);

    // capture & print timestamp
    char timestamp[10] = {0};
    snprintf_P(timestamp, sizeof(timestamp), PSTR("[%06lu] "), millis() % 1000000);
    SerialAndTelnet.print(timestamp);

    SerialAndTelnet.println(buffer);

    delete[] buffer;
}

// use Serial?
bool MyESP::getUseSerial() {
    return (_use_serial);
}

// called when WiFi is connected, and used to start OTA, MQTT
void MyESP::_wifiCallback(justwifi_messages_t code, char * parameter) {
    if ((code == MESSAGE_CONNECTED)) {
#if defined(ARDUINO_ARCH_ESP32)
        String hostname = String(WiFi.getHostname());
#else
        String hostname = WiFi.hostname();
#endif

        myDebug_P(PSTR("[WIFI] SSID  %s"), WiFi.SSID().c_str());
        myDebug_P(PSTR("[WIFI] CH    %d"), WiFi.channel());
        myDebug_P(PSTR("[WIFI] RSSI  %d"), WiFi.RSSI());
        myDebug_P(PSTR("[WIFI] IP    %s"), WiFi.localIP().toString().c_str());
        myDebug_P(PSTR("[WIFI] MAC   %s"), WiFi.macAddress().c_str());
        myDebug_P(PSTR("[WIFI] GW    %s"), WiFi.gatewayIP().toString().c_str());
        myDebug_P(PSTR("[WIFI] MASK  %s"), WiFi.subnetMask().toString().c_str());
        myDebug_P(PSTR("[WIFI] DNS   %s"), WiFi.dnsIP().toString().c_str());
        myDebug_P(PSTR("[WIFI] HOST  %s"), hostname.c_str());

        // start OTA
        ArduinoOTA.begin(); // moved to support esp32
        myDebug_P(PSTR("[OTA] listening to %s.local:%u"), ArduinoOTA.getHostname().c_str(), OTA_PORT);

        // MQTT Setup
        _mqtt_setup();

        _wifi_connected = true;

        // finally if we don't want Serial anymore, turn it off
        if (!_use_serial) {
            Serial.println("Disabling serial port");
            Serial.flush();
            Serial.end();
            SerialAndTelnet.setSerial(NULL);
        } else {
            Serial.println("Using serial port output");
        }

        // call any final custom settings
        if (_wifi_callback) {
            _wifi_callback();
        }
    }

    if (code == MESSAGE_ACCESSPOINT_CREATED) {
        myDebug_P(PSTR("[WIFI] MODE AP --------------------------------------"));
        myDebug_P(PSTR("[WIFI] SSID  %s"), jw.getAPSSID().c_str());
        myDebug_P(PSTR("[WIFI] IP    %s"), WiFi.softAPIP().toString().c_str());
        myDebug_P(PSTR("[WIFI] MAC   %s"), WiFi.softAPmacAddress().c_str());

        // call any final custom settings
        if (_wifi_callback) {
            _wifi_callback();
        }
    }

    if (code == MESSAGE_CONNECTING) {
        myDebug_P(PSTR("[WIFI] Connecting to %s"), parameter);
        _wifi_connected = false;
    }

    if (code == MESSAGE_CONNECT_FAILED) {
        myDebug_P(PSTR("[WIFI] Could not connect to %s"), parameter);
        _wifi_connected = false;
    }

    if (code == MESSAGE_DISCONNECTED) {
        myDebug_P(PSTR("[WIFI] Disconnected"));
        _wifi_connected = false;
    }
}

// received MQTT message
// we send this to the call back function. Important to parse are the event strings such as MQTT_MESSAGE_EVENT and MQTT_CONNECT_EVENT
void MyESP::_mqttOnMessage(char * topic, char * payload, size_t len) {
    if (len == 0)
        return;

    char message[len + 1];
    strlcpy(message, (char *)payload, len + 1);

    // myDebug_P(PSTR("[MQTT] Received %s => %s"), topic, message); // enable for debugging

    // topics are in format MQTT_BASE/HOSTNAME/TOPIC
    char * topic_magnitude = strrchr(topic, '/'); // strip out everything until last /
    if (topic_magnitude != nullptr) {
        topic = topic_magnitude + 1;
    }

    // Send message event to custom service
    (_mqtt_callback)(MQTT_MESSAGE_EVENT, topic, message);
}

// MQTT subscribe
// to MQTT_BASE/app_hostname/topic
void MyESP::mqttSubscribe(const char * topic) {
    if (mqttClient.connected() && (strlen(topic) > 0)) {
        unsigned int packetId = mqttClient.subscribe(_mqttTopic(topic), _mqtt_qos);
        myDebug_P(PSTR("[MQTT] Subscribing to %s (PID %d)"), _mqttTopic(topic), packetId);
    }
}

// MQTT unsubscribe
// to MQTT_BASE/app_hostname/topic
void MyESP::mqttUnsubscribe(const char * topic) {
    if (mqttClient.connected() && (strlen(topic) > 0)) {
        unsigned int packetId = mqttClient.unsubscribe(_mqttTopic(topic));
        myDebug_P(PSTR("[MQTT] Unsubscribing to %s (PID %d)"), _mqttTopic(topic), packetId);
    }
}

// MQTT Publish
void MyESP::mqttPublish(const char * topic, const char * payload) {
    // myDebug_P(PSTR("[MQTT] Sending pubish to %s with payload %s"), _mqttTopic(topic), payload);
    mqttClient.publish(_mqttTopic(topic), _mqtt_qos, _mqtt_retain, payload);
}

// MQTT onConnect - when a connect is established
void MyESP::_mqttOnConnect() {
    myDebug_P(PSTR("[MQTT] Connected"));
    _mqtt_reconnect_delay = MQTT_RECONNECT_DELAY_MIN;

    // say we're alive to the Last Will topic
    mqttClient.publish(_mqttTopic(_mqtt_will_topic), 1, true, _mqtt_will_online_payload);

    // call custom function to handle mqtt receives
    (_mqtt_callback)(MQTT_CONNECT_EVENT, NULL, NULL);
}

// MQTT setup
void MyESP::_mqtt_setup() {
    if (!_mqtt_host) {
        myDebug_P(PSTR("[MQTT] disabled"));
    }

    _mqtt_reconnect_delay = MQTT_RECONNECT_DELAY_MIN;

    mqttClient.onConnect([this](bool sessionPresent) { _mqttOnConnect(); });

    mqttClient.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
        if (reason == AsyncMqttClientDisconnectReason::TCP_DISCONNECTED) {
            myDebug_P(PSTR("[MQTT] TCP Disconnected. Check mqtt logs."));
            (_mqtt_callback)(MQTT_DISCONNECT_EVENT, NULL, NULL); // call callback with disconnect
        }
        if (reason == AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED) {
            myDebug_P(PSTR("[MQTT] Identifier Rejected"));
        }
        if (reason == AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE) {
            myDebug_P(PSTR("[MQTT] Server unavailable"));
        }
        if (reason == AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS) {
            myDebug_P(PSTR("[MQTT] Malformed credentials"));
        }
        if (reason == AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED) {
            myDebug_P(PSTR("[MQTT] Not authorized"));
        }
    });

    //mqttClient.onSubscribe([this](uint16_t packetId, uint8_t qos) { myDebug_P(PSTR("[MQTT] Subscribe ACK for PID %d"), packetId); });

    //mqttClient.onPublish([this](uint16_t packetId) { myDebug_P(PSTR("[MQTT] Publish ACK for PID %d"), packetId); });

    mqttClient.onMessage(
        [this](char * topic, char * payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
            _mqttOnMessage(topic, payload, len);
        });
}

// WiFI setup
void MyESP::_wifi_setup() {
    jw.setHostname(_app_hostname); // Set WIFI hostname (otherwise it would be ESP-XXXXXX)
    jw.subscribe([this](justwifi_messages_t code, char * parameter) { _wifiCallback(code, parameter); });
    jw.enableAP(false);
    jw.setConnectTimeout(WIFI_CONNECT_TIMEOUT);
    jw.setReconnectTimeout(WIFI_RECONNECT_INTERVAL);
#ifdef WIFI_SSID
    jw.enableAPFallback(false);
#else
    jw.enableAPFallback(true);                 // AP mode only as fallback
#endif
    jw.enableSTA(true);                        // Enable STA mode (connecting to a router)
    jw.enableScan(false);                      // Configure it to scan available networks and connect in order of dBm
    jw.cleanNetworks();                        // Clean existing network configuration
    jw.addNetwork(_wifi_ssid, _wifi_password); // Add a network
}

// OTA Setup
void MyESP::_ota_setup() {
    if (!_wifi_ssid) {
        return;
    }

    //ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(_app_hostname);

    ArduinoOTA.onStart([this]() { myDebug_P(PSTR("[OTA] Start")); });
    ArduinoOTA.onEnd([this]() { myDebug_P(PSTR("[OTA] Done, restarting...")); });
    ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
        static unsigned int _progOld;
        unsigned int        _prog = (progress / (total / 100));
        if (_prog != _progOld) {
            myDebug_P(PSTR("[OTA] Progress: %u%%\r"), _prog);
            _progOld = _prog;
        }
    });

    ArduinoOTA.onError([this](ota_error_t error) {
        if (error == OTA_AUTH_ERROR)
            myDebug_P(PSTR("[OTA] Auth Failed"));
        else if (error == OTA_BEGIN_ERROR)
            myDebug_P(PSTR("[OTA] Begin Failed"));
        else if (error == OTA_CONNECT_ERROR)
            myDebug_P(PSTR("[OTA] Connect Failed"));
        else if (error == OTA_RECEIVE_ERROR)
            myDebug_P(PSTR("[OTA] Receive Failed"));
        else if (error == OTA_END_ERROR)
            myDebug_P(PSTR("[OTA] End Failed"));
    });
}

// sets boottime
void MyESP::setBoottime(const char * boottime) {
    if (_boottime) {
        free(_boottime);
    }
    _boottime = strdup(boottime);
}

// Set callback of sketch function to process project messages
void MyESP::setTelnet(command_t * cmds, uint8_t count, telnetcommand_callback_f callback_cmd, telnet_callback_f callback) {
    _helpProjectCmds        = cmds;         // command list
    _helpProjectCmds_count  = count;        // number of commands
    _telnetcommand_callback = callback_cmd; // external function to handle commands
    _telnet_callback        = callback;
}

void MyESP::_telnetConnected() {
    myDebug_P(PSTR("[TELNET] Telnet connection established"));
    _consoleShowHelp(); // Show the initial message
    if (_telnet_callback) {
        (_telnet_callback)(TELNET_EVENT_CONNECT); // call callback
    }
}

void MyESP::_telnetDisconnected() {
    myDebug_P(PSTR("[TELNET] Telnet connection closed"));
    if (_telnet_callback) {
        (_telnet_callback)(TELNET_EVENT_DISCONNECT); // call callback
    }
}

// Initialize the telnet server
void MyESP::_telnet_setup() {
    SerialAndTelnet.setWelcomeMsg("");
    SerialAndTelnet.setCallbackOnConnect([this]() { _telnetConnected(); });
    SerialAndTelnet.setCallbackOnDisconnect([this]() { _telnetDisconnected(); });
    SerialAndTelnet.setDebugOutput(false);
    SerialAndTelnet.begin(TELNET_SERIAL_BAUD); // default baud is 115200

    // init command buffer for console commands
    memset(_command, 0, TELNET_MAX_COMMAND_LENGTH);
}

// Show help of commands
void MyESP::_consoleShowHelp() {
    SerialAndTelnet.println();
    SerialAndTelnet.printf("* Connected to: %s version %s", _app_name, _app_version);
    SerialAndTelnet.println();

    if (WiFi.getMode() & WIFI_AP) {
        SerialAndTelnet.printf("* ESP8266 is in AP mode with SSID %s", jw.getAPSSID().c_str());
        SerialAndTelnet.println();
    } else {
#if defined(ARDUINO_ARCH_ESP32)
        String hostname = String(WiFi.getHostname());
#else
        String hostname = WiFi.hostname();
#endif
        SerialAndTelnet.printf("* Hostname: %s   IP: %s   MAC: %s",
                               hostname.c_str(),
                               WiFi.localIP().toString().c_str(),
                               WiFi.macAddress().c_str());
#ifdef ARDUINO_BOARD
        SerialAndTelnet.printf("  Board: %s", ARDUINO_BOARD);
#endif
        SerialAndTelnet.println();
        SerialAndTelnet.printf("* Connected to WiFi SSID: %s", WiFi.SSID().c_str());
        SerialAndTelnet.println();
        SerialAndTelnet.printf("* Boot time: %s", _boottime);
        SerialAndTelnet.println();
    }
    SerialAndTelnet.printf("* Free RAM:%d KB, Load:%d%%", (ESP.getFreeHeap() / 1024), getSystemLoadAverage());
    SerialAndTelnet.println();
    // for battery power is ESP.getVcc()

    SerialAndTelnet.println(FPSTR("*"));
    SerialAndTelnet.println(FPSTR("* Commands:"));
    SerialAndTelnet.println(FPSTR("*  ?=help, CTRL-D=quit"));
    SerialAndTelnet.println(FPSTR("*  reboot"));
    SerialAndTelnet.println(FPSTR("*  set"));
    SerialAndTelnet.println(FPSTR("*  set wifi <ssid> <password>"));
    SerialAndTelnet.println(FPSTR("*  set <mqtt_host | mqtt_username | mqtt_password> [value]"));
    SerialAndTelnet.println(FPSTR("*  set erase"));
    SerialAndTelnet.println(FPSTR("*  set serial"));

    // print custom commands if available. Taken from progmem
    if (_telnetcommand_callback) {
        // find the longest key length so we can right align it
        uint8_t max_len = 0;
        for (uint8_t i = 0; i < _helpProjectCmds_count; i++) {
            if (strlen(_helpProjectCmds[i].key) > max_len)
                max_len = strlen(_helpProjectCmds[i].key);
        }

        for (uint8_t i = 0; i < _helpProjectCmds_count; i++) {
            SerialAndTelnet.print(FPSTR("*  "));
            SerialAndTelnet.print(FPSTR(_helpProjectCmds[i].key));
            for (uint8_t j = 0; j < ((max_len + 5) - strlen(_helpProjectCmds[i].key)); j++) { // account for longest string length
                SerialAndTelnet.print(FPSTR(" "));                                            // padding
            }
            SerialAndTelnet.println(FPSTR(_helpProjectCmds[i].description));
        }
    }

    SerialAndTelnet.println(); // newline
}

// reset / restart
void MyESP::resetESP() {
    myDebug_P(PSTR("* Reboot ESP..."));
    end();
#if defined(ARDUINO_ARCH_ESP32)
    ESP.restart();
#else
    ESP.restart();
#endif
}

// read next word from string buffer
char * MyESP::_telnet_readWord() {
    char * word = strtok(NULL, ", \n");
    return word;
}

// change setting for 2 params (set <command> <value1> <value2>)
void MyESP::_changeSetting2(const char * setting, const char * value1, const char * value2) {
    if (strcmp(setting, "wifi") == 0) {
        if (_wifi_ssid)
            free(_wifi_ssid);
        if (_wifi_password)
            free(_wifi_password);
        _wifi_ssid     = NULL;
        _wifi_password = NULL;

        if (value1) {
            _wifi_ssid = strdup(value1);
        }

        if (value2) {
            _wifi_password = strdup(value2);
        }

        (void)fs_saveConfig();
        SerialAndTelnet.println("Wifi credentials set. Type 'reboot' to restart...");
    }
}

// change settings - always as strings
// messy code but effective since we don't have too many settings
// wc is word count, number of parameters after the 'set' command
void MyESP::_changeSetting(uint8_t wc, const char * setting, const char * value) {
    bool ok = false;

    // check for our internal commands first
    if (strcmp(setting, "erase") == 0) {
        _fs_eraseConfig();
        return;
    } else if ((strcmp(setting, "wifi") == 0) && (wc == 1)) { // erase wifi settings
        if (_wifi_ssid)
            free(_wifi_ssid);
        if (_wifi_password)
            free(_wifi_password);
        _wifi_ssid     = NULL;
        _wifi_password = NULL;
        ok             = true;
    } else if (strcmp(setting, "mqtt_host") == 0) {
        if (_mqtt_host)
            free(_mqtt_host);
        _mqtt_host = NULL; // just to be sure
        if (value) {
            _mqtt_host = strdup(value);
        }
        ok = true;
    } else if (strcmp(setting, "mqtt_username") == 0) {
        if (_mqtt_username)
            free(_mqtt_username);
        _mqtt_username = NULL; // just to be sure
        if (value) {
            _mqtt_username = strdup(value);
        }
        ok = true;
    } else if (strcmp(setting, "mqtt_password") == 0) {
        if (_mqtt_password)
            free(_mqtt_password);
        _mqtt_password = NULL; // just to be sure
        if (value) {
            _mqtt_password = strdup(value);
        }
        ok = true;
    } else if (strcmp(setting, "serial") == 0) {
        ok          = true;
        _use_serial = false;
        if (value) {
            if (strcmp(value, "on") == 0) {
                _use_serial = true;
                ok          = true;
            } else if (strcmp(value, "off") == 0) {
                _use_serial = false;
                ok          = true;
            } else {
                ok = false;
            }
        }
    } else {
        // finally check for any custom commands
        ok = (_fs_settings_callback)(MYESP_FSACTION_SET, wc, setting, value);
    }

    if (!ok) {
        SerialAndTelnet.println("\nInvalid parameter for set command.");
        return;
    }

    // check for 2 params
    if (value == nullptr) {
        SerialAndTelnet.printf("%s setting reset to its default value.", setting);
    } else {
        // must be 3 params
        SerialAndTelnet.printf("%s changed.", setting);
    }
    SerialAndTelnet.println();

    (void)fs_saveConfig();
}

void MyESP::_telnetCommand(char * commandLine) {
    // count the number of arguments
    char *   str   = commandLine;
    bool     state = false;
    unsigned wc    = 0;
    while (*str) {
        if (*str == ' ' || *str == '\n' || *str == '\t') {
            state = false;
        } else if (state == false) {
            state = true;
            ++wc;
        }
        ++str;
    }

    // check first for reserved commands
    char * temp             = strdup(commandLine); // because strotok kills original string buffer
    char * ptrToCommandName = strtok((char *)temp, ", \n");

    // set command
    if (strcmp(ptrToCommandName, "set") == 0) {
        if (wc == 1) {
            SerialAndTelnet.println();
            SerialAndTelnet.println("Stored settings:");
            SerialAndTelnet.printf("  wifi=%s ", (!_wifi_ssid) ? "<not set>" : _wifi_ssid);
            if (!_wifi_password) {
                SerialAndTelnet.print("<not set>");
            } else {
                for (uint8_t i = 0; i < strlen(_wifi_password); i++)
                    SerialAndTelnet.print("*");
            }
            SerialAndTelnet.println();
            SerialAndTelnet.printf("  mqtt_host=%s", (!_mqtt_host) ? "<not set>" : _mqtt_host);
            SerialAndTelnet.println();
            SerialAndTelnet.printf("  mqtt_username=%s", (!_mqtt_username) ? "<not set>" : _mqtt_username);
            SerialAndTelnet.println();
            SerialAndTelnet.printf("  mqtt_password=");
            if (!_mqtt_password) {
                SerialAndTelnet.print("<not set>");
            } else {
                for (uint8_t i = 0; i < strlen(_mqtt_password); i++)
                    SerialAndTelnet.print("*");
            }

            SerialAndTelnet.println();
            SerialAndTelnet.printf("  serial=%s", (_use_serial) ? "on" : "off");

            SerialAndTelnet.println();

            // print custom settings
            (_fs_settings_callback)(MYESP_FSACTION_LIST, 0, NULL, NULL);

            SerialAndTelnet.println();
            SerialAndTelnet.println("Usage: set <setting> [value...]");
        } else if (wc == 2) {
            char * setting = _telnet_readWord();
            _changeSetting(1, setting, NULL);
        } else if (wc == 3) {
            char * setting = _telnet_readWord();
            char * value   = _telnet_readWord();
            _changeSetting(2, setting, value);
        } else if (wc == 4) {
            char * setting = _telnet_readWord();
            char * value1  = _telnet_readWord();
            char * value2  = _telnet_readWord();
            _changeSetting2(setting, value1, value2);
        }
        return;
    }

    // reboot command
    if ((strcmp(ptrToCommandName, "reboot") == 0) && (wc == 1)) {
        resetESP();
    }

    // call callback function
    (_telnetcommand_callback)(wc, commandLine);
}

// handler for Telnet
void MyESP::_telnetHandle() {
    SerialAndTelnet.handle();

    static uint8_t charsRead = 0;
    // read asynchronously until full command input
    while (SerialAndTelnet.available()) {
        char c = SerialAndTelnet.read();

        if (_use_serial) {
            SerialAndTelnet.serialPrint(c); // echo to Serial if connected
        }

        switch (c) {
        case '\r': // likely have full command in buffer now, commands are terminated by CR and/or LF
        case '\n':
            _command[charsRead] = '\0'; // null terminate our command char array
            if (charsRead > 0) {
                charsRead      = 0; // is static, so have to reset
                _suspendOutput = false;
                if (_use_serial) {
                    SerialAndTelnet.println(); // force newline if in Telnet
                }
                _telnetCommand(_command);
            }
            break;
        case '\b': // (^H) handle backspace in input: put a space in last char
        case 0x7F: // (^?)
            if (charsRead > 0) {
                _command[--charsRead] = '\0';
                if (_use_serial) {
                        if (c != '\b') {
                            SerialAndTelnet.serialPrint('\b');
                        }
                        SerialAndTelnet.serialPrint(' ');
                        SerialAndTelnet.serialPrint('\b');
                }
            }
            break;
        case '?':
            if (!_suspendOutput) {
                _consoleShowHelp();
            } else {
                _command[charsRead++] = c; // add it to buffer as its part of the string entered
            }
            break;
        case 0x04: // EOT, CTRL-D
            myDebug_P(PSTR("[TELNET] exiting telnet session"));
            SerialAndTelnet.disconnectClient();
            break;
        default:
            _suspendOutput = true;
            if (charsRead < TELNET_MAX_COMMAND_LENGTH) {
                _command[charsRead++] = c;
            }
            _command[charsRead] = '\0'; // just in case
            break;
        }
    }
}

// ensure we have a connection to MQTT broker
void MyESP::_mqttConnect() {
    if (!_mqtt_host || mqttClient.connected() || (WiFi.status() != WL_CONNECTED)) {
        return;
    }

    // Check reconnect interval
    static unsigned long last = 0;
    if (millis() - last < _mqtt_reconnect_delay)
        return;
    last = millis();

    // Increase the reconnect delay
    _mqtt_reconnect_delay += MQTT_RECONNECT_DELAY_STEP;
    if (_mqtt_reconnect_delay > MQTT_RECONNECT_DELAY_MAX) {
        _mqtt_reconnect_delay = MQTT_RECONNECT_DELAY_MAX;
    }

    mqttClient.setServer(_mqtt_host, MQTT_PORT);
    mqttClient.setClientId(_app_hostname);
    mqttClient.setKeepAlive(_mqtt_keepalive);
    mqttClient.setCleanSession(false);

    // last will
    if (_mqtt_will_topic) {
        myDebug_P(PSTR("[MQTT] Setting last will topic %s"), _mqttTopic(_mqtt_will_topic));
        mqttClient.setWill(_mqttTopic(_mqtt_will_topic), 1, true, _mqtt_will_offline_payload); // retain always true
    }

    if (_mqtt_username && _mqtt_password) {
        myDebug_P(PSTR("[MQTT] Connecting to MQTT using user %s..."), _mqtt_username);
        mqttClient.setCredentials(_mqtt_username, _mqtt_password);
    } else {
        myDebug_P(PSTR("[MQTT] Connecting to MQTT..."));
    }

    // Connect to the MQTT broker
    mqttClient.connect();
}

// Setup everything we need
void MyESP::setWIFI(const char * wifi_ssid, const char * wifi_password, wifi_callback_f callback) {
    // Check SSID too long or missing
    if (!wifi_ssid || *wifi_ssid == 0x00 || strlen(wifi_ssid) > 31) {
        _wifi_ssid = NULL;
    } else {
        _wifi_ssid = strdup(wifi_ssid);
    }

    // Check PASS too long
    if (!wifi_password || *wifi_ssid == 0x00 || strlen(wifi_password) > 31) {
        _wifi_password = NULL;
    } else {
        _wifi_password = strdup(wifi_password);
    }

    // callback
    _wifi_callback = callback;
}

// init MQTT settings
void MyESP::setMQTT(const char *    mqtt_host,
                    const char *    mqtt_username,
                    const char *    mqtt_password,
                    const char *    mqtt_base,
                    unsigned long   mqtt_keepalive,
                    unsigned char   mqtt_qos,
                    bool            mqtt_retain,
                    const char *    mqtt_will_topic,
                    const char *    mqtt_will_online_payload,
                    const char *    mqtt_will_offline_payload,
                    mqtt_callback_f callback) {
    // can be empty
    if (!mqtt_host || *mqtt_host == 0x00) {
        _mqtt_host = NULL;
    } else {
        _mqtt_host = strdup(mqtt_host);
    }

    // mqtt username and password can be empty
    if (!mqtt_username || *mqtt_username == 0x00) {
        _mqtt_username = NULL;
    } else {
        _mqtt_username = strdup(mqtt_username);
    }

    // can be empty
    if (!mqtt_password || *mqtt_password == 0x00) {
        _mqtt_password = NULL;
    } else {
        _mqtt_password = strdup(mqtt_password);
    }

    // base
    if (_mqtt_base) {
        free(_mqtt_base);
    }
    _mqtt_base = strdup(mqtt_base);

    // callback
    _mqtt_callback = callback;

    // various mqtt settings
    _mqtt_keepalive = mqtt_keepalive;
    _mqtt_qos       = mqtt_qos;
    _mqtt_retain    = mqtt_retain;

    // last will
    if (!mqtt_will_topic || *mqtt_will_topic == 0x00) {
        _mqtt_will_topic = NULL;
    } else {
        _mqtt_will_topic = strdup(mqtt_will_topic);
    }

    if (!mqtt_will_online_payload || *mqtt_will_online_payload == 0x00) {
        _mqtt_will_online_payload = NULL;
    } else {
        _mqtt_will_online_payload = strdup(mqtt_will_online_payload);
    }

    if (!mqtt_will_offline_payload || *mqtt_will_offline_payload == 0x00) {
        _mqtt_will_offline_payload = NULL;
    } else {
        _mqtt_will_offline_payload = strdup(mqtt_will_offline_payload);
    }
}

// builds up a topic by prefixing the base and hostname
char * MyESP::_mqttTopic(const char * topic) {
    char buffer[MQTT_MAX_TOPIC_SIZE] = {0};

    strlcpy(buffer, _mqtt_base, sizeof(buffer));
    strlcat(buffer, "/", sizeof(buffer));
    strlcat(buffer, _app_hostname, sizeof(buffer));
    strlcat(buffer, "/", sizeof(buffer));
    strlcat(buffer, topic, sizeof(buffer));

    if (_mqtt_topic) {
        free(_mqtt_topic);
    }
    _mqtt_topic = strdup(buffer);

    return _mqtt_topic;
}


// print contents of file
// assume Serial is open
void MyESP::_fs_printConfig() {
    myDebug_P(PSTR("[FS] Contents:"));

    File configFile = SPIFFS.open(MYEMS_CONFIG_FILE, "r");
    while (configFile.available()) {
        SerialAndTelnet.print((char)configFile.read());
    }
    myDebug_P(PSTR(""));
    configFile.close();
}

// format File System
void MyESP::_fs_eraseConfig() {
    myDebug_P(PSTR("[FS] Erasing settings, please wait a few seconds. ESP will automatically restart when finished."));

    if (SPIFFS.format()) {
        delay(1000); // wait 1 seconds
        resetESP();
    }
}

void MyESP::setSettings(fs_callback_f callback_fs, fs_settings_callback_f callback_settings_fs) {
    _fs_callback          = callback_fs;
    _fs_settings_callback = callback_settings_fs;
}

// load from spiffs
bool MyESP::_fs_loadConfig() {
    File configFile = SPIFFS.open(MYEMS_CONFIG_FILE, "r");

    size_t size = configFile.size();
    if (size > 1024) {
        myDebug_P(PSTR("[FS] Config file size is too large"));
        return false;
    } else if (size == 0) {
        myDebug_P(PSTR("[FS] Failed to open config file"));
        // file does not exist, so assume its the first install. Set serial to on
        _use_serial = true;
        return false;
    }

    // assign buffer
    std::unique_ptr<char[]> buf(new char[size]);

    // use configFile.readString
    configFile.readBytes(buf.get(), size);

    StaticJsonBuffer<SPIFFS_MAXSIZE> jsonBuffer;
    JsonObject &                     json = jsonBuffer.parseObject(buf.get());

    const char * value;

    value      = json["wifi_ssid"];
    _wifi_ssid = (value) ? strdup(value) : NULL;

    value          = json["wifi_password"];
    _wifi_password = (value) ? strdup(value) : NULL;

    value      = json["mqtt_host"];
    _mqtt_host = (value) ? strdup(value) : NULL;

    value          = json["mqtt_username"];
    _mqtt_username = (value) ? strdup(value) : NULL;

    value          = json["mqtt_password"];
    _mqtt_password = (value) ? strdup(value) : NULL;

    // callback for loading custom settings
    // ok is false if there's a problem loading a custom setting (e.g. does not exist)
    bool ok = (_fs_callback)(MYESP_FSACTION_LOAD, json);

    // new configs after release 1.3.x
    if (json.containsKey("use_serial")) {
        _use_serial = (bool)json["use_serial"];
    } else {
        _use_serial = false; // if first time, set serial to off
        ok          = false;
    }

    configFile.close();

    return ok;
}

// save settings to spiffs
bool MyESP::fs_saveConfig() {
    StaticJsonBuffer<SPIFFS_MAXSIZE> jsonBuffer;
    JsonObject &                     json = jsonBuffer.createObject();

    json["wifi_ssid"]     = _wifi_ssid;
    json["wifi_password"] = _wifi_password;
    json["mqtt_host"]     = _mqtt_host;
    json["mqtt_username"] = _mqtt_username;
    json["mqtt_password"] = _mqtt_password;
    json["use_serial"]    = _use_serial;

    // callback for saving custom settings
    (void)(_fs_callback)(MYESP_FSACTION_SAVE, json);

    File configFile = SPIFFS.open(MYEMS_CONFIG_FILE, "w");
    if (!configFile) {
        Serial.println("[FS] Failed to open config file for writing");
        return false;
    }

    json.printTo(configFile);

    configFile.close();

    return true;
}

// init the SPIFF file system and load the config
// if it doesn't exist try and create it
// force Serial for debugging, and turn it off afterwards
void MyESP::_fs_setup() {
    if (!SPIFFS.begin()) {
        Serial.println("[FS] Failed to mount the file system");
        return;
    }

    // load the config file. if it doesn't exist create it
    if (!_fs_loadConfig()) {
        Serial.println("[FS] Re-creating config file");
        fs_saveConfig();
    }

    // _fs_printConfig(); // for debugging
}

uint16_t MyESP::getSystemLoadAverage() {
    return _load_average;
}

// calculate load average
void MyESP::_calculateLoad() {
    static unsigned long last_loadcheck    = 0;
    static unsigned long load_counter_temp = 0;
    load_counter_temp++;

    if (millis() - last_loadcheck > LOADAVG_INTERVAL) {
        static unsigned long load_counter     = 0;
        static unsigned long load_counter_max = 1;

        load_counter      = load_counter_temp;
        load_counter_temp = 0;
        if (load_counter > load_counter_max) {
            load_counter_max = load_counter;
        }
        _load_average  = 100 - (100 * load_counter / load_counter_max);
        last_loadcheck = millis();
    }
}

// return true if wifi is connected
//    WL_NO_SHIELD        = 255,   // for compatibility with WiFi Shield library
//    WL_IDLE_STATUS      = 0,
//    WL_NO_SSID_AVAIL    = 1,
//    WL_SCAN_COMPLETED   = 2,
//    WL_CONNECTED        = 3,
//    WL_CONNECT_FAILED   = 4,
//    WL_CONNECTION_LOST  = 5,
//    WL_DISCONNECTED     = 6
bool MyESP::isWifiConnected() {
    return (_wifi_connected);
}

// register new instance
void MyESP::begin(const char * app_hostname, const char * app_name, const char * app_version) {
    _app_hostname = strdup(app_hostname);
    _app_name     = strdup(app_name);
    _app_version  = strdup(app_version);

    _telnet_setup(); // Telnet setup
    _fs_setup();     // SPIFFS setup, do this first to get values
    _wifi_setup();   // WIFI setup
    _ota_setup();
}

/*
 * Loop. This is called as often as possible and it handles wifi, telnet, mqtt etc
 */
void MyESP::loop() {
    _calculateLoad();
    _telnetHandle(); // Telnet/Debugger

    jw.loop(); // WiFi

    // do nothing else until we've got a wifi connection
    if (WiFi.getMode() & WIFI_AP) {
        return;
    }

    ArduinoOTA.handle(); // OTA
    _mqttConnect();      // MQTT

    yield(); // ...and breath
}

MyESP myESP; // create instance
