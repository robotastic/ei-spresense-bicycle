#include <bicycle-street-fomo_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <Camera.h>
#include <SDHCI.h>


#include <RTC.h>
//#include <GNSS.h>

//#include <ArduinoMqttClient.h>
//#include <LTE.h>

#define RAW_WIDTH CAM_IMGSIZE_QVGA_H
#define RAW_HEIGHT CAM_IMGSIZE_QVGA_V
#define INPUT_WIDTH EI_CLASSIFIER_INPUT_WIDTH
#define INPUT_HEIGHT EI_CLASSIFIER_INPUT_HEIGHT
#define MY_TIMEZONE_IN_SECONDS (-4 * 60 * 60) // EST
#define DETECTION_THRESHOLD 0.80
#define DETECTION_RESET_THRESHOLD 0.50
#define SAVE_THRESHOLD 0.20


// APN name
#define APP_LTE_APN "iot.truphone.com" // replace your APN

/* APN authentication settings
 * Ignore these parameters when setting LTE_NET_AUTHTYPE_NONE.
 */
#define APP_LTE_USER_NAME "" // replace with your username
#define APP_LTE_PASSWORD  "" // replace with your password

// APN IP type
#define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V4V6) // IP : IPv4v6

// APN authentication type
 #define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_NONE) // Authentication : NONE

/* RAT to use
 * Refer to the cellular carriers information
 * to find out which RAT your SIM supports.
 * The RAT set on the modem can be checked with LTEModemVerification::getRAT().
 */

#define APP_LTE_RAT (LTE_NET_RAT_CATM) // RAT : LTE-M (LTE Cat-M1)




static SDClass SD;
//SpGnss Gnss;
static bool got_time = false;
static uint8_t *raw_image = NULL;
static uint8_t *input_image = NULL;
static ei_impulse_result_t ei_result = {0};
int take_picture_count = 0;
int bicycle_count = 0;

// MQTT broker
#define BROKER_NAME        "io.adafruit.com"
#define BROKER_PORT        1883               // port 8883 is the default for MQTT over TLS.
#define MQTT_TOPIC         "robotastic/feeds/bike"
// MQTT publish interval settings
#define PUBLISH_INTERVAL_SEC   5   // MQTT publish interval in sec
#define MAX_NUMBER_OF_PUBLISH  60  // Maximum number of publish
/*
LTE lteAccess;
LTEClient client;
MqttClient mqttClient(client);*/
SDClass theSD;
int numOfPubs = 0;
unsigned long lastPubSec = 0;
char broker[] = BROKER_NAME;
int port = BROKER_PORT;
char topic[]  = MQTT_TOPIC;


#define ALIGN_PTR(p, a)                                                        \
  ((p & (a - 1)) ? (((uintptr_t)p + a) & ~(uintptr_t)(a - 1)) : p)

/**
   Print error message
*/
void printError(enum CamErr err) {
  Serial.print("Error: ");
  switch (err) {
  case CAM_ERR_NO_DEVICE:
    Serial.println("No Device");
    break;
  case CAM_ERR_ILLEGAL_DEVERR:
    Serial.println("Illegal device error");
    break;
  case CAM_ERR_ALREADY_INITIALIZED:
    Serial.println("Already initialized");
    break;
  case CAM_ERR_NOT_INITIALIZED:
    Serial.println("Not initialized");
    break;
  case CAM_ERR_NOT_STILL_INITIALIZED:
    Serial.println("Still picture not initialized");
    break;
  case CAM_ERR_CANT_CREATE_THREAD:
    Serial.println("Failed to create thread");
    break;
  case CAM_ERR_INVALID_PARAM:
    Serial.println("Invalid parameter");
    break;
  case CAM_ERR_NO_MEMORY:
    Serial.println("No memory");
    break;
  case CAM_ERR_USR_INUSED:
    Serial.println("Buffer already in use");
    break;
  case CAM_ERR_NOT_PERMITTED:
    Serial.println("Operation not permitted");
    break;
  default:
    break;
  }
}

/*
void checkClock() {

    SpNavData  NavData;

    // Get the UTC time
    Gnss.getNavData(&NavData);
    SpGnssTime *time = &NavData.time;

    // Check if the acquired UTC time is accurate
    if (time->year >= 2000) {
      RtcTime now = RTC.getTime();
      // Convert SpGnssTime to RtcTime
      RtcTime gps(time->year, time->month, time->day,
                  time->hour, time->minute, time->sec, time->usec * 1000);
#ifdef MY_TIMEZONE_IN_SECONDS
      // Set the time difference
      gps += MY_TIMEZONE_IN_SECONDS;
#endif
      int diff = now - gps;
      if (abs(diff) >= 1) {
        RTC.setTime(gps);
      }
      Gnss.stop();
      Gnss.end();
      got_time = true;
    }
  
}

*/
/**
 * @brief      Convert rgb565 data to rgb888
 *
 * @param[in]  src_buf  The rgb565 data
 * @param      dst_buf  The rgb888 data
 * @param      src_len  length of rgb565 data
 */

bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len) {
  uint8_t hb, lb;
  uint32_t pix_count = src_len / 2;

  for (uint32_t i = 0; i < pix_count; i++) {
    // The Spresense Image Data appears to be written with a different Endiannes
    // compared to the one on the Nicla Vision
    lb = *src_buf++;
    hb = *src_buf++;

    *dst_buf++ = hb & 0xF8;
    *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
    *dst_buf++ = (lb & 0x1F) << 3;
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (raw_image[pixel_ix] << 16) +
                          (raw_image[pixel_ix + 1] << 8) +
                          raw_image[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  // and done!
  return 0;
}


/*
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (input_image[pixel_ix] << 16) +
                          (input_image[pixel_ix + 1] << 8) +
                          input_image[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  // and done!
  return 0;
}

*/
bool create_dir(String path) {
  if (!SD.exists(path)) {
    if (!SD.mkdir(path)) {
      ei_printf("Failed to create dir: %s\n", path);
      return false;
    } else {
      return true;
    }
  }
  return true;
}

String create_topic_directory(String topic) {
  RtcTime now = RTC.getTime();
  char month_s[3];
  char day_s[3];
  bool success;
  String path = "/";

  sprintf(month_s, " %02d", now.month());
  sprintf(day_s, " %02d", now.day());
  path = String("/") + month_s;
  if (!create_dir(path)) {
    return String("/");
  }
  path = String("/") + month_s + "/" + day_s;
  if (!create_dir(path)) {
    return String("/");
  }
  path = String("/") + month_s + "/" + day_s + "/" + topic;
  if (!create_dir(path)) {
    return String("/");
  }
  return path;
}

void save_json(char * filename) {
    char line[100];
    bool first_obj = true;
    if (SD.begin()) {
    File myFile = SD.open(filename, FILE_WRITE);
    myFile.println("{ \"results\": [");

  for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
    auto bb = ei_result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }
    if (!first_obj) {
    sprintf(line,",\n{\n\"label\": \"%s\",", bb.label);
    } else {
      sprintf(line,"{\n\"label\": \"%s\",", bb.label);
    }
    myFile.println(line);
    sprintf(line,"\"confidence\": \"%s\",", bb.value);
    myFile.println(line);
    sprintf(line,"\"x\": \"%s\",", bb.x);
    myFile.println(line);
    sprintf(line,"\"y\": \"%s\",", bb.y);
    myFile.println(line);
    sprintf(line,"\"width\": \"%s\",", bb.width);
    myFile.println(line);
    sprintf(line,"\"height\": \"%s\"\n}", bb.height);
    myFile.print(line);
    first_obj = false;
  }
    myFile.println("\n]}");
    myFile.close();

    } else {
    Serial.println("failed to save images, check that "
                   "SD card is connected properly");
  }
}

void save_results(float high_score) {
  RtcTime now = RTC.getTime();
  char csv_filename[400];
  char input_filename[400];
  char json_filename[400];
  char raw_filename[400];
  char line[1200];
  String path;

  if (SD.begin()) {

    
    
    
    
   int short_score = floor(high_score * 100);
    sprintf(csv_filename,"/log.csv");
    path = create_topic_directory("detections");
    sprintf(input_filename, "%s/%d-%d %02d-%02d %02d_%02d_%02d.888", path.c_str(),take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second());
    sprintf(json_filename, "%s/%d-%d %02d-%02d %02d_%02d_%02d.json", path.c_str(),take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second());
    path = create_topic_directory("raw");
    sprintf(raw_filename, "%s/%d-%d %02d-%02d %02d_%02d_%02d-raw.888", path.c_str(),take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second());
    ei_printf("INFO: saving %s to SD card... size: %d\n", input_filename,
              INPUT_HEIGHT * INPUT_WIDTH * 3);
    SD.remove(input_filename);
    File myFile = SD.open(input_filename, FILE_WRITE);
    myFile.write(input_image, INPUT_WIDTH * INPUT_HEIGHT * 3);
    myFile.close();

    save_json(json_filename);


    ei_printf("INFO: saving %s to SD card... size: %d\n", raw_filename,
              RAW_HEIGHT * RAW_WIDTH * 3);
    SD.remove(raw_filename);
    myFile = SD.open(raw_filename, FILE_WRITE);
    myFile.write(raw_image, RAW_WIDTH * RAW_HEIGHT * 3);
    myFile.close();

    myFile = SD.open(csv_filename, FILE_WRITE);
    sprintf(line, "%d, %d, %02d, %02d, %02d:%02d:%02d, %s, %s\n", take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second(), input_filename, json_filename);
    myFile.write(line);
    myFile.close();
  } else {
    Serial.println("failed to save images, check that "
                   "SD card is connected properly");
  }

}

/*
void send_updated_count() {
      // Publish to broker
    Serial.print("Sending message to topic: ");
    Serial.println(topic);
    Serial.print("Publish: ");
    String jsonString = "{\"value\":" + String(bicycle_count)  + "}";
    Serial.println(jsonString);

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic);
    mqttClient.print(jsonString);
    //mqttClient.print(out);
    mqttClient.endMessage();
}
*/
void analyze_results() {


  /* if the confidence goes above the DETECTION_THRESHOLD, set tracking_object to TRUE and add to the bicycle count
      the tracking_object is used to help make sure a bicycle doesn't get counted twice. It gets reset to false
      when the confidence is below DETECTION_RESET_THRESHOLD */
  bool save_image = false;
  bool spotted_object = false;
  static bool tracking_object = false;
  bool reset_tracking = true;
  float high_score = 0.0;
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d "
            "ms.): \n",
            ei_result.timing.dsp, ei_result.timing.classification,
            ei_result.timing.anomaly);

  bool bb_found = ei_result.bounding_boxes[0].value > 0;

  for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
    auto bb = ei_result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }
    if (bb.value > high_score) {
      high_score = bb.value;
    }
    if (bb.value >= SAVE_THRESHOLD) {
      save_image = true;
    }

    if (bb.value >= DETECTION_RESET_THRESHOLD) {
      reset_tracking = false;
    }
    if (bb.value >= DETECTION_THRESHOLD) {
      spotted_object = true;
    }

    ei_printf("    %s (", bb.label);
    ei_printf_float(bb.value);
    ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y,
              bb.width, bb.height);
  }

  if (!bb_found) {
    ei_printf("    No objects found\n");
  } 

  if (save_image) {
    save_results(high_score);
  }

  if (spotted_object) {
    if (!tracking_object) {
      tracking_object = true;
      bicycle_count++;
      //send_updated_count();
      ei_printf("Detected new bicycle, count is: %d\n", bicycle_count);
      // add something here to write to a file
    }
  }

  if (reset_tracking) {
    tracking_object = false;
  }
}
/*
void connect_lte_mqtt() {
  char apn[LTE_NET_APN_MAXLEN] = APP_LTE_APN;
  LTENetworkAuthType authtype = APP_LTE_AUTH_TYPE;
  char user_name[LTE_NET_USER_MAXLEN] = APP_LTE_USER_NAME;
  char password[LTE_NET_PASSWORD_MAXLEN] = APP_LTE_PASSWORD;

 
  Serial.println("Starting HTTP client."); 


  Serial.println("=========== APN information ===========");
  Serial.print("Access Point Name  : ");
  Serial.println(apn);
  Serial.print("Authentication Type: ");
  Serial.println(authtype == LTE_NET_AUTHTYPE_CHAP ? "CHAP" :
                 authtype == LTE_NET_AUTHTYPE_NONE ? "NONE" : "PAP");
  if (authtype != LTE_NET_AUTHTYPE_NONE) {
    Serial.print("User Name          : ");
    Serial.println(user_name);
    Serial.print("Password           : ");
    Serial.println(password);
  }

 while (true) {

    // Power on the modem and Enable the radio function. 

    if (lteAccess.begin() != LTE_SEARCHING) {
      Serial.println("Could not transition to LTE_SEARCHING.");
      Serial.println("Please check the status of the LTE board.");
      for (;;) {
        sleep(1);
      }
    }

    // * The connection process to the APN will start.
    // * If the synchronous parameter is false,
    // * the return value will be returned when the connection process is started.
    if (lteAccess.attach(APP_LTE_RAT,
                         apn,
                         user_name,
                         password,
                         authtype,
                         APP_LTE_IP_TYPE) == LTE_READY) {
      Serial.println("attach succeeded.");
      break;
    }

    // * If the following logs occur frequently, one of the following might be a cause:
    // * - APN settings are incorrect
    // * - SIM is not inserted correctly
    // * - If you have specified LTE_NET_RAT_NBIOT for APP_LTE_RAT,
    // *   your LTE board may not support it.
    // * - Rejected from LTE network
     
    Serial.println("An error has occurred. Shutdown and retry the network attach process after 1 second.");
    lteAccess.shutdown();
    sleep(1);
  }

    // Set local time (not UTC) obtained from the network to RTC.
  RTC.begin();
  unsigned long currentTime;
  while(0 == (currentTime = lteAccess.getTime())) {
    sleep(1);
  }
  RtcTime rtc(currentTime);
  RTC.setTime(rtc);

  
    char mqtt_username[] = "robotastic"; //bike-detect.azure-devices.net/woodley_place/?api-version=2021-04-12";
  char mqtt_password[] = ""; //"SharedAccessSignature sr=bike-detect.azure-devices.net%2Fdevices%2Fwoodley_place&sig=VESTPm64tMMelVK7NmevjHKWR%2B6274DqFlOCpac05VI%3D&se=2022598854";
  mqttClient.setUsernamePassword(mqtt_username, mqtt_password);
  mqttClient.setId("woodley_place");
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);
  while(!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    sleep(3);
  }
  Serial.println("You're connected to the MQTT broker!");
}

*/
void setup() {
  CamErr err;

  Serial.begin(115200);
  Serial.println("INFO: wildlife_camera initializing on wakeup...");

  /* Initialize SD */

  while (!SD.begin()) {
    Serial.println("Insert SD card.");
  }

  err = theCamera.begin(0, CAM_VIDEO_FPS_5, RAW_WIDTH, RAW_HEIGHT,
                        CAM_IMAGE_PIX_FMT_RGB565);
  theCamera.setAutoISOSensitivity(true);
  theCamera.setAutoWhiteBalance(true);

  err = theCamera.setStillPictureImageFormat(RAW_WIDTH, RAW_HEIGHT,
                                             CAM_IMAGE_PIX_FMT_RGB565);
  if (err)
    printError(err);

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tNo. of classes: %d\n",
            sizeof(ei_classifier_inferencing_categories) /
                sizeof(ei_classifier_inferencing_categories[0]));
  ei_printf("\tRaw Image Width: %d Height: %d\n", RAW_WIDTH, RAW_HEIGHT);

  raw_image = (uint8_t *)malloc(RAW_WIDTH * RAW_HEIGHT * 3 + 32);
  raw_image = (uint8_t *)ALIGN_PTR((uintptr_t)raw_image, 32);
  if (raw_image == NULL) {
    ei_printf("raw_image MALLOC FAILED!!\n");
    exit(0);
  }



  input_image = (uint8_t *)malloc(INPUT_WIDTH * INPUT_HEIGHT * 3 + 32);
  input_image = (uint8_t *)ALIGN_PTR((uintptr_t)input_image, 32);
  if (input_image == NULL) {
    ei_printf("input_image MALLOC FAILED!!\n");
    exit(0);
  }

  // Initialize RTC at first
  //RTC.begin();

  // Initialize and start GNSS library
 /* int ret;
  ret = Gnss.begin();
  assert(ret == 0);

  ret = Gnss.start();
  assert(ret == 0);*/

  //connect_lte_mqtt();
  ei_printf("Complete setup\n");

}

void loop() {
  // put your main code here, to run repeatedly:
  ei::signal_t signal;


/*
    // Wait for GNSS data
  if (!got_time && Gnss.waitUpdate()) {
    checkClock();
  }

*/
  ei_printf("Top of the loop\n");
  CamImage img = theCamera.takePicture();
  if (!img.isAvailable()) {
    ei_printf("ERR: Take Picture Failed!\n");
    return false; // fast path if image is no longer ready
  }

  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;
  ei_printf("Trying to convert\n");
  bool converted =
      RBG565ToRGB888(img.getImgBuff(), raw_image, img.getImgSize());
  if (!converted) {
    ei_printf("ERR: Conversion failed\n");
    return false;
  } else {
    ei_printf("Successfully converted to RGB888\n");
  }

 /* ei::image::processing::crop_and_interpolate_rgb888(
      raw_image, RAW_WIDTH, RAW_HEIGHT, input_image,
      EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
*/
    ei::image::processing::crop_and_interpolate_rgb888(
      raw_image, RAW_WIDTH, RAW_HEIGHT, raw_image,
      EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
  ei_printf("Cropped and converted\n");
  EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, false);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERROR: Failed to run classifier (%d)\n", err);
    return;
  }
  // print the predictions
  analyze_results();

  take_picture_count++;
}
