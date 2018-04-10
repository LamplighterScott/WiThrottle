WiThrottleOutlets

DCC++ Wemos Mega+ESP8266 to WiThrottle communication

Truncated JMRI WiThrottle server implementation for DCC++ command station: GPIO Outputs instead of encoded accessory signals

SYSTEM:
 * Marklin Z 1700mm x 600 layout, four engines with decoders, 12v track power
 * Wemos Mega+ESP8266, Motor Shield, GPIO's connected to Darlington arrays (ULN2803 x3)
 * DCC++ sketch forked for only 20ms GPIO HIGH time.  Throw and Close commands use one GPIO each per turnout.
 * WiThrottle sketch forked for use of GPIO Outlets for accessory control instaeded of signal encoding

REFERENCES:
 * Valerie Valley RR https://sites.google.com/site/valerievalleyrr/
 * JMRI WiThrottle DCC++ ESP8266 https://github.com/vhar/withrottle v1.02b
 * DCC++ https://github.com/DccPlusPlus
 * ESP8266 Core https://github.com/esp8266/Arduino
 * JMRI WiFi Throttle Communications Protocol http://jmri.sourceforge.net/help/en/package/jmri/jmrit/withrottle/Protocol.shtml
 * WiThrottle official site http://www.withrottle.com/WiThrottle/Home.html
 * Download WiThrottle on the AppStore https://itunes.apple.com/us/app/withrottle-lite/id344190130
 * Engine Driver official site https://enginedriver.mstevetodd.com/
 * Download Engine Driver on the GooglePlay https://play.google.com/store/apps/details?id=jmri.enginedriver
