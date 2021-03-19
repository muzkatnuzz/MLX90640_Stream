/*

  This is a simple MJPEG streaming webserver implemented for AI-Thinker ESP32-CAM
  and ESP-EYE modules.
  This is tested to work with VLC and Blynk video widget and can support up to 10
  simultaneously connected streaming clients.
  Simultaneous streaming is implemented with dedicated FreeRTOS tasks.

  Inspired by and based on this Instructable: $9 RTSP Video Streamer Using the ESP32-CAM Board
  (https://www.instructables.com/id/9-RTSP-Video-Streamer-Using-the-ESP32-CAM-Board/)

  Board: AI-Thinker ESP32-CAM or ESP-EYE
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Partrition: Minimal SPIFFS
   PSRAM: Enabled
*/

// ESP32 has two cores: APPlication core and PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0

#include "Arduino.h"
#include "Wire.h"
#include "ESPAsyncWebServer.h"
#include "WiFiClient.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "img_converters.h"

// TODO check if dependencies are needed...
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// Wire defaults
uint16_t I2C_SDA = 0;
uint16_t I2C_SCL = 0;

#define MAX_CLIENTS 10

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8                  //Default shift for MLX90640 in open air

paramsMLX90640 mlx90640;

// forward declarations
void camCB(void *pvParameters);
char *allocateMemory(char *aPtr, size_t aSize);
void streamCB(void *pvParameters);
void handleJPGSstream(AsyncWebServerRequest *request);
void handleJPG(AsyncWebServerRequest *request);

AsyncWebServer *server;

// ===== rtos task handles =========================
// Streaming is implemented with 3 tasks:
TaskHandle_t tMjpeg; // handles client connections to the webserver
TaskHandle_t tCam;   // handles getting picture frames from the camera and storing them locally

uint8_t noActiveClients; // number of active clients

// frameSync semaphore is used to prevent streaming buffer as it is replaced with the next frame
SemaphoreHandle_t frameSync = NULL;

// We will try to achieve 4 FPS frame rate
const int FPS = 4;

// We will handle web client requests every 100 ms (10 Hz)
const int WSINTERVAL = 100;

// ======== Colorize temperature readings ==========================
//low range of the sensor (this will be blue on the screen)
const float MINTEMP = 20;

//high range of the sensor (this will be red on the screen)
const float MAXTEMP = 35;

//the colors we will be using
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

uint16_t displayPixelWidth, displayPixelHeight;

// ======== Server Connection Handler Task ==========================
void mjpegCB(void *pvParameters)
{
  log_d("mjpegCB: Executing on core %d", xPortGetCoreID());

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creating frame synchronization semaphore and initializing it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive(frameSync);

  //=== setup section  ==================

  //  Creating RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
      camCB,     // callback
      "cam",     // name
      10 * 1024, // stack size
      NULL,      // parameters
      2,         // priority
      &tCam,     // RTOS task handle
      PRO_CPU);  // core

  //  Registering webserver handling routines
  server->on("/stream", HTTP_GET, handleJPGSstream);
  server->on("/jpg", HTTP_GET, handleJPG);

  // TODO should be done by host Starting webserver
  // server.begin();

  noActiveClients = 0;

  Serial.printf("\nmjpegCB: free heap (start)  : %d\n", ESP.getFreeHeap());
  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // TODO check if handleClient is needed for async webserver
    // server.handleClient();

    //  After every server client handling request, we let other tasks run and then pause
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/*
/ Get frame data from camera
/ Size of result should be at least 768
/ The call will only be successful on the core on which the wire.begin call was done
*/
int getFrame(float *result)
{
  //log_d("getFrame: Executing on core %d", xPortGetCoreID());
  for (byte x = 0; x < 2; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    //log_d("GetFrameData from address: %d", MLX90640_address);
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.printf("GetFrameData Error: %d", status);
      return status;
    }

    //log_d("GetFrameData status: %d", status);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, result);
  }

  return 0;
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}

// start with some initial colors
float minTemp = 20.0;
float maxTemp = 40.0;

byte red, green, blue;
// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;

/***************************************************************************************
** Function name:           color565
** Description:             convert three 8 bit RGB levels to a 16 bit colour value
***************************************************************************************/
uint16_t color565(byte r, byte g, byte b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// source: https://github.com/netzbasteln/MLX90640-Thermocam/blob/master/TC_ESP32_MLX90640.ino
// Get color for temp value.
uint16_t calculateColor(float val)
{
  /*
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation
    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html
  */

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > minTemp) & (val < a))
  {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c))
  {
    green = 255;
  }
  else if (val > c)
  {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a))
  {
    green = 0;
  }

  if (val <= b)
  {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d))
  {
    blue = 0;
  }
  else if (val > d)
  {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return color565(red, green, blue);
}

// Use mapping table to map given temperature to color 
uint16_t mapColor(float val)
{
  float t = val;

  t = min(t, MAXTEMP);
  t = max(t, MINTEMP);

  // convert measured temperatures to color
  uint8_t colorIndex = map(t, MINTEMP, MAXTEMP, 0, 255);
  colorIndex = constrain(colorIndex, 0, 255);

  // replace measured temperature by color
  uint16_t color = camColors[colorIndex];
  return color;
}

// Current frame information
volatile uint32_t frameNumber;
volatile size_t camSize;  // size of the current frame, byte
volatile char *camBuf; // pointer to the current frame

// ==== RTOS task to grab frames from the camera =========================
void camCB(void *pvParameters)
{
  log_d("camCB: Executing on core %d", xPortGetCoreID());
  TickType_t xLastWakeTime;

  //  A running interval associated with currently desired frame rate
  const TickType_t xFrequency = (1000 / FPS) / portTICK_PERIOD_MS;

  //  Pointer to the frames
  char *fbs = NULL;
  frameNumber = 0;

  // init camera on same core where frames are collected
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1)
      ;
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  //Set refresh rate
  //A rate of 0.5Hz takes 4Sec per reading because we have to read two frames to get complete picture
  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
  MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    float mlx90640To[32 * 24];
    getFrame(mlx90640To);

    uint16_t mlx90640ToColors[32 * 24];

    // convert to actual colors
    for (uint8_t h = 0; h < 24; h++)
    {
      for (uint8_t w = 0; w < 32; w++)
      {
        // choose which color conversion fits better - mapping table or calculation
        if (true)
        {
          mlx90640ToColors[h * 32 + w] = mapColor(mlx90640To[h * 32 + w]);
        }
        else
        {
          mlx90640ToColors[h * 32 + w] = calculateColor(mlx90640To[h * 32 + w]);
        }
      }
    }

    //log_d("Allocate Memory. Largest heap size: %zu", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)); // as from https://github.com/espressif/esp32-camera/blob/master/conversions/to_jpg.cpp
    fbs = allocateMemory(fbs, 32 * 24 * sizeof(uint16_t));
    //log_d("Memcopy. Largest heap size: %zu", heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)); // as from https://github.com/espressif/esp32-camera/blob/master/conversions/to_jpg.cpp

    //  Copy current frame into local buffer
    memcpy(fbs, mlx90640ToColors, 32 * 24 * sizeof(uint16_t));

    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //  Do not allow frame copying while switching the current frame
    xSemaphoreTake(frameSync, xFrequency);
    camBuf = fbs;
    camSize = 32 * 24 * sizeof(uint16_t);
    frameNumber++;
    //  Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive(frameSync);

    //  Immediately let other (streaming) tasks run
    taskYIELD();

    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera. We can save some juice
    //  by suspedning the tasks
    if (noActiveClients == 0)
    {
      Serial.printf("mjpegCB: free heap           : %d\n", ESP.getFreeHeap());
      Serial.printf("mjpegCB: min free heap       : %d\n", ESP.getMinFreeHeap());
      Serial.printf("mjpegCB: max alloc free heap : %d\n", ESP.getMaxAllocHeap());
      Serial.printf("mjpegCB: tCam stack wtrmark  : %d\n", uxTaskGetStackHighWaterMark(tCam));
      Serial.flush();
      vTaskSuspend(NULL); // passing NULL means "suspend yourself"
    }
  }
}

// ==== Memory allocator that takes advantage of PSRAM if present =======================
char *allocateMemory(char *aPtr, size_t aSize)
{
  //  Since current buffer is too smal, free it
  if (aPtr != NULL)
    free(aPtr);

  char *ptr = NULL;
  ptr = (char *)ps_malloc(aSize);

  // If the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL)
  {
    Serial.println("Out of memory!");
    delay(5000);
    ESP.restart();
  }
  return ptr;
}

// ==== STREAMING ======================================================
const char HEADER[] = "HTTP/1.1 200 OK\r\n"
                      "Access-Control-Allow-Origin: *\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

struct streamInfo
{
  uint32_t frame;
  AsyncClient *client;
  TaskHandle_t task;
  char *buffer;
  size_t len;
};

// ==== Handle connection request from clients ===============================
void handleJPGSstream(AsyncWebServerRequest *request)
{
  if (noActiveClients >= MAX_CLIENTS)
    return;
  Serial.printf("handleJPGSstream start: free heap  : %d\n", ESP.getFreeHeap());

  streamInfo *info = new streamInfo;

  info->frame = frameNumber - 1;
  info->client = request->client();
  info->buffer = NULL;
  info->len = 0;

  //  Creating task to push the stream to all connected clients
  int rc = xTaskCreatePinnedToCore(
      streamCB,
      "streamCB",
      3 * 1024,
      (void *)info,
      2,
      &info->task,
      APP_CPU);
  if (rc != pdPASS)
  {
    Serial.printf("handleJPGSstream: error creating RTOS task. rc = %d\n", rc);
    Serial.printf("handleJPGSstream: free heap  : %d\n", ESP.getFreeHeap());
    //    Serial.printf("stk high wm: %d\n", uxTaskGetStackHighWaterMark(tSend));
    delete info;
  }

  noActiveClients++;

  // Wake up streaming tasks, if they were previously suspended:
  if (eTaskGetState(tCam) == eSuspended)
    vTaskResume(tCam);
}

// ==== Actually stream content to all connected clients ========================
void streamCB(void *pvParameters)
{
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  streamInfo *info = (streamInfo *)pvParameters;

  if (info == NULL)
  {
    Serial.println("streamCB: a NULL pointer passed");
  }
  //  Immediately send this client a header
  info->client->write(HEADER, hdrLen);
  info->client->write(BOUNDARY, bdrLen);
  taskYIELD();

  xLastWakeTime = xTaskGetTickCount();
  xFrequency = pdMS_TO_TICKS(1000 / FPS);

  for (;;)
  {
    //  Only bother to send anything if there is someone watching
    if (info->client->connected())
    {
      if (info->frame != frameNumber)
      {
        //log_d("JPEG compression:");
        // conversion to jpg
        //unsigned long st = millis();
        uint8_t *jpeg;
        size_t jpeg_length;

        // fmt2jpg uses malloc with jpg_buf_len = 64*1024;
        //log_d("Starting jpeg conversion:  %d", xPortGetCoreID());
        bool jpeg_converted = fmt2jpg((uint8_t *)camBuf, camSize, 32, 24, PIXFORMAT_RGB565, 80, &jpeg, &jpeg_length);
        if (!jpeg_converted)
        {
          log_e("JPEG compression failed");
          jpeg_length = 0;
          jpeg = nullptr;
          continue;
        }
        //log_i("JPEG: %lums, %uB", millis() - st, jpeg_length);

        xSemaphoreTake(frameSync, portMAX_DELAY);
        if (info->buffer == NULL)
        {
          info->buffer = allocateMemory(info->buffer, jpeg_length);
          info->len = jpeg_length;
        }
        else
        {
          if (jpeg_length > info->len)
          {
            info->buffer = allocateMemory(info->buffer, jpeg_length);
            info->len = jpeg_length;
          }
        }

        memcpy(info->buffer, (const void *)jpeg, info->len);
        xSemaphoreGive(frameSync);
        taskYIELD();

        // free memory consumed by jpeg conversion
        free(jpeg);
        jpeg = nullptr;

        info->frame = frameNumber;
        info->client->write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", info->len);
        info->client->write(buf, strlen(buf));
        info->client->write((char *)info->buffer, (size_t)info->len);
        info->client->write(BOUNDARY, bdrLen);
        // TODO check if flush is needed for async
        //info->client.flush();
      }
    }
    else
    {
      //  client disconnected - clean up.
      noActiveClients--;
      Serial.printf("streamCB: Stream Task stack wtrmark  : %d\n", uxTaskGetStackHighWaterMark(info->task));
      Serial.flush();
      // TODO check if flush is needed for async
      //info->client.flush();
      info->client->stop();
      if (info->buffer)
      {
        free(info->buffer);
        info->buffer = NULL;
      }
      delete info;
      info = NULL;
      vTaskDelete(NULL);
    }
    //  Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n"
                       "Content-disposition: inline; filename=capture.jpg\r\n"
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

// ==== Serve up one JPEG frame =============================================
void handleJPG(AsyncWebServerRequest *request)
{
  AsyncClient *client = request->client();

  if (!client->connected())
    return;
  float frame[768];
  getFrame(frame);
  client->write(JHEADER, jhdLen);
  client->write((char *)frame, sizeof(frame));
}

// ==== configure method to be called within setup ==================================================================
// adds /jpg and /stream to server
void configureCamera(AsyncWebServer *webServer, uint16_t sda = 0, uint16_t scl = 0)
{
  server = webServer;
  I2C_SDA = sda;
  I2C_SCL = scl;

  log_d("configureCamera: Executing on core %d", xPortGetCoreID());

  // Start mainstreaming RTOS task
  xTaskCreatePinnedToCore(
      mjpegCB,
      "mjpeg",
      3 * 1024,
      NULL,
      2,
      &tMjpeg,
      APP_CPU);

  // TODO check example from adafruit:
  //   mlx.setMode(MLX90640_CHESS);
  // mlx.setResolution(MLX90640_ADC_18BIT);
  // mlx.setRefreshRate(MLX90640_8_HZ);
  // Wire.setClock(1000000); // max 1 MHz
}

void startCamera()
{
  // TODO maybe move mjpegCB task start here
}
