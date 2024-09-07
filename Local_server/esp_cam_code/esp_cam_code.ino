#include <Arduino.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"

// Data for WiFi connection
const char* ssid = "m";
const char* password = "11111111";

// Server data
String serverName = "192.168.67.244";   
String serverPath = "/";    
const int serverPort = 19999;             

// WiFi client used for connection
WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const int timerInterval = 100;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);

  // Connects to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  // ESPCam configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    delay(1000);
    ESP.restart();
  }

  // Sends photo to server
  sendPhoto(); 
}

void loop() {
  // Gets the time in millis
  unsigned long currentMillis = millis();
  // Sends an image if the interval has passed
  if (currentMillis - previousMillis >= timerInterval) {
    sendPhoto();
    previousMillis = currentMillis;
  }
}
String sendPhoto() {
  String response;    // String to store the final xx|yy response

  camera_fb_t* fb = NULL;                     // Pointer to a frame buffer structure for camera capture
  fb = esp_camera_fb_get();                   // Capture a photo using the camera
  if (!fb) {                                  // Check if the capture failed
    Serial.println("Camera capture failed");  // Print an error message if capture failed
    delay(1000);                              // Wait for 1 second
    ESP.restart();                            // Restart the ESP32 if capture failed
  }

  // Attempt to connect to the server
  else if (client.connect(serverName.c_str(), serverPort)) {
    // Prepare the multipart form-data HTTP request
   String head = "--ESPCAM\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--ESPCAM--\r\n";


    uint32_t imageLen = fb->len;                        // Get the length of the captured image
    uint32_t extraLen = head.length() + tail.length();  // Calculate the length of the HTTP headers and footers
    uint32_t totalLen = imageLen + extraLen;            // Calculate the total length of the HTTP request body

    // Send the HTTP POST request
    client.println("POST " + serverPath + " HTTP/1.1");                                 // Send the HTTP POST request line
    client.println("Host: " + serverName);                                              // Send the Host header
    client.println("Content-Length: " + String(totalLen));                              // Send the Content-Length header
    client.println("Content-Type: multipart/form-data; boundary=ESPCAM");
    client.println();                                                                   // Send a blank line to indicate the end of headers
    client.print(head);                                                                 // Send the form-data headers

    uint8_t* fbBuf = fb->buf;  // Get a pointer to the image buffer
    size_t fbLen = fb->len;    // Get the length of the image buffer
    // Send the image data in chunks
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {             // Check if a full 1024-byte chunk can be sent
        client.write(fbBuf, 1024);        // Send a 1024-byte chunk of the image data
        fbBuf += 1024;                    // Move the buffer pointer forward by 1024 bytes
      } else if (fbLen % 1024 > 0) {      // Check if there's a remaining partial chunk
        size_t remainder = fbLen % 1024;  // Calculate the size of the remaining partial chunk
        client.write(fbBuf, remainder);   // Send the remaining partial chunk
      }
    }
    client.print(tail);  // Send the form-data footer

    esp_camera_fb_return(fb);  // Return the frame buffer to free up memory

    int timeoutTimer = 1000;      // Set a timeout period of 1000 milliseconds
    long startTimer = millis();   // Record the current time
    boolean state = false;        // Flag to track when headers have been fully received

    while ((startTimer + timeoutTimer) > millis()) {
      if (client.available()) {                      // Check if data is available to read from the server
        String line = client.readStringUntil('\n');  // Read a line of data from the server

        if (line.indexOf('|') > 0) {                 // Check if the line contains the delimiter '|'
          response = line;                           // Store the response as it is in "xx|yy" format
          break;                                     // Exit the loop once the desired format is found
        }
        startTimer = millis();                       // Reset the timer after each line read
      }
    }

    client.stop();      // Close the connection to the server
    Serial.write(response.c_str());  // Print the "xx|yy" response to the Serial monitor
    return response;             // Return the "xx|yy" response
  } else {                  // If connection to the server failed
    Serial.write("0|0\n");  // Print an error code to the Serial monitor
    return "0|0";           // Return "0|0" as the response
  }
}
