#include <bicycle-street-fomo_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <Camera.h>
#include <SDHCI.h>


#include <RTC.h>
#include <GNSS.h>


#define RAW_WIDTH CAM_IMGSIZE_QVGA_H
#define RAW_HEIGHT CAM_IMGSIZE_QVGA_V
#define INPUT_WIDTH EI_CLASSIFIER_INPUT_WIDTH
#define INPUT_HEIGHT EI_CLASSIFIER_INPUT_HEIGHT
#define MY_TIMEZONE_IN_SECONDS (-4 * 60 * 60) // EST
#define DETECTION_THRESHOLD 0.80
#define DETECTION_RESET_THRESHOLD 0.50
#define SAVE_THRESHOLD 0.20


static SDClass theSD;
SpGnss Gnss;
static bool got_time = false;
static uint8_t *raw_image = NULL;
static uint8_t *input_image = NULL;
static ei_impulse_result_t ei_result = {0};
int take_picture_count = 0;
int bicycle_count = 0;

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


void updateClock()
{
  static RtcTime old;
  RtcTime now = RTC.getTime();

  // Display only when the second is updated
  if (now != old) {
    printClock(now);
    old = now;
  }
}

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
  String path;
  sprintf(month_s, " %02d", now.month());
  sprintf(day_s, " %02d", now.day());
  path = "/" + month_s;
  if (!create_dir(path)) {
    return NULL;
  }
  path = "/" + month_s + "/" + day_s;
  if (!create_dir(path)) {
    return NULL;
  }
  path = "/" + month_s + "/" + day_s + "/" + topic;
  if (!create_dir(path)) {
    return NULL;
  }
  return path;
}

void save_results(float high_score) {
  RtcTime now = RTC.getTime();
  String path;

  if (theSD.begin()) {
    int short_score = floor(high_score * 100);
    path = create_topic_directory("detections");
    sprintf(filename, "%s/%d-%d %02d-%02d %02d_%02d_%02d.888", path.c_str(),take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second());

    ei_printf("INFO: saving %s to SD card... size: %d\n", filename,
              INPUT_HEIGHT * INPUT_WIDTH * 3);
    theSD.remove(filename);
    File myFile = theSD.open(filename, FILE_WRITE);
    myFile.write(input_image, INPUT_WIDTH * INPUT_HEIGHT * 3);
    myFile.close();

    path = create_topic_directory("raw");
    sprintf(filename, "%s/%d-%d %02d-%02d %02d_%02d_%02d-raw.888", path.c_str(),take_picture_count, short_score, now.month(), now.day(),
         now.hour(), now.minute(), now.second());

    ei_printf("INFO: saving %s to SD card... size: %d\n", filename,
              RAW_HEIGHT * RAW_WIDTH * 3);
    theSD.remove(filename);
    myFile = theSD.open(filename, FILE_WRITE);
    myFile.write(raw_image, RAW_WIDTH * RAW_HEIGHT * 3);
    myFile.close();
  } else {
    Serial.println("failed to save images, check that "
                   "SD card is connected properly");
  }

}


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
      ei_printf("Detected new bicycle, count is: %d\n", bicycle_count);
      // add something here to write to a file
    }
  }

  if (reset_tracking) {
    tracking_object = false;
  }
}

void setup() {
  CamErr err;

  Serial.begin(115200);
  Serial.println("INFO: wildlife_camera initializing on wakeup...");

  /* Initialize SD */

  while (!theSD.begin()) {
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
  RTC.begin();

  // Initialize and start GNSS library
  int ret;
  ret = Gnss.begin();
  assert(ret == 0);

  ret = Gnss.start();
  assert(ret == 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  ei::signal_t signal;
  char filename[400];


    // Wait for GNSS data
  if (!got_time && Gnss.waitUpdate()) {
    checkClock();
  }


  CamImage img = theCamera.takePicture();
  if (!img.isAvailable()) {
    return false; // fast path if image is no longer ready
  }

  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  bool converted =
      RBG565ToRGB888(img.getImgBuff(), raw_image, img.getImgSize());
  if (!converted) {
    ei_printf("ERR: Conversion failed\n");
    return false;
  } else {
    ei_printf("Successfully converted to RGB888\n");
  }

  ei::image::processing::crop_and_interpolate_rgb888(
      raw_image, RAW_WIDTH, RAW_HEIGHT, input_image,
      EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);

  EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, false);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERROR: Failed to run classifier (%d)\n", err);
    return;
  }
  // print the predictions
  analyze_results();

  take_picture_count++;
}
