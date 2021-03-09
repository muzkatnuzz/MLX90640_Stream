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
    float mlx90640To[768];
    getFrame(mlx90640To);
    
    log_d("Allocate Memory. Heap size: %d", esp_get_free_heap_size());
    fbs = allocateMemory(fbs, 768 * sizeof(float));
    log_d("Memcopy. Heap size: %d", esp_get_free_heap_size());
    
    //  Copy current frame into local buffer
    memcpy(fbs, mlx90640To, sizeof(mlx90640To));

    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //  Do not allow frame copying while switching the current frame
    xSemaphoreTake(frameSync, xFrequency);
    camBuf = fbs;
    camSize = sizeof(mlx90640To);
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
      free(fbs);
      fbs, camBuf = NULL; 
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
        log_d("JPEG compression:");
        // conversion to jpg
        unsigned long st = millis();
        uint8_t *jpeg;
        size_t jpeg_length;

        log_d("Starting jpeg conversion:  %d", xPortGetCoreID());
        bool jpeg_converted = fmt2jpg((uint8_t *)camBuf, camSize, 32, 24, PIXFORMAT_RGB565, 80, &jpeg, &jpeg_length);
        if (!jpeg_converted)
        {
          log_e("JPEG compression failed");
          jpeg_length = 0;
          jpeg = NULL;
          continue;
        }
        log_i("JPEG: %lums, %uB", millis() - st, jpeg_length);

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
}

void startCamera()
{
  // TODO maybe move mjpegCB task start here
}
