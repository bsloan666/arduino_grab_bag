/*
 Web Server

 A simple web server that shows the value of the analog input pins.
 using an Arduino WIZnet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * Analog inputs attached to pins A0 through A5 (optional)

 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 modified 02 Sept 2015
 by Arturo Guadalupi
 
 */

#include <SPI.h>
#include <Ethernet.h>
/*
class BEWebServer(){
  private:
    unsigned int num_pairs;
    unsigned int buf_index;
    char buffer[256];
    unsigned int var_offset;
    unsigned int val_offset;
};
*/
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ethernet WebServer Example");

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start the server
  server.begin();
  Serial.print("server is at ");
  Serial.println(Ethernet.localIP());
}

char buf[256];
char obuf[256];
char variable_array[128];
int value_array[16];
int nvalues = 0;

unsigned int index = 0;

void loop() {
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          
          parse_key_value_pairs(buf, obuf, variable_array, value_array);
          for(int i = 0; i < nvalues; i++){
            Serial.print(variable_array + i * 16);
            Serial.print(": ");
            Serial.println(value_array[i]);
          }

          client.print("<br />");
          client.println("</html>");
          clear_buffer(buf);
          clear_buffer(obuf);
          break;
        }
        if (c == '\n') {
          // you're starting a new line

          currentLineIsBlank = true;
          //index = 0;

        } else if (c != '\r') {
          // you've gotten a character on the current line
          buf[index] = c;
          index++;
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void clear_buffer(char buffer[]){
  for(int i = 0; i < 256; i++){
    buffer[i] = 0;
  }
}

void parse_key_value_pairs(char buffer[], char out_buffer[], char vars[], int vals[]){
  int var_offset = 0;
  int val_offset = 0;
  bool parsing = false;
  nvalues = 0;

  for(int i = 0; i < 256; i++){
    out_buffer[i] = buffer[i];
    if(buffer[i] == '?'){
      out_buffer[i] = '\0';
      var_offset = i + 1;
      parsing = true;
    } else if(buffer[i] == '&'){ 
      out_buffer[i] = '\0';
      strcpy(vars + nvalues * 16, out_buffer + var_offset);
      vals[nvalues] = atoi(out_buffer + val_offset);
      //dump(out_buffer + var_offset, out_buffer + val_offset);
      var_offset = i + 1;
      nvalues++;
    } else if(buffer[i] == '='){ 
      out_buffer[i] = '\0';
      val_offset = i + 1;
    } else if(buffer[i] == ' ' && parsing){ 
      out_buffer[i] = '\0';
      strcpy(vars + nvalues * 16, out_buffer + var_offset);
      vals[nvalues] = atoi(out_buffer + val_offset);
      nvalues++;
      //dump(out_buffer + var_offset, out_buffer + val_offset);
      break;
    }
  }
}

void dump(char var_buf[], char val_buf[]){
  Serial.print("DUMP: ");
  Serial.print(var_buf);
  Serial.print(": ");
  Serial.println(val_buf);
}