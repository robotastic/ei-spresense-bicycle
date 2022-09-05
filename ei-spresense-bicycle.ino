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
#define LED0_ON                                                                \
  digitalWrite(LED0, HIGH); /* Macros for Switch on/off Board LEDs */
#define LED0_OFF digitalWrite(LED0, LOW);
#define LED1_ON digitalWrite(LED1, HIGH);
#define LED1_OFF digitalWrite(LED1, LOW);
#define LED2_ON digitalWrite(LED2, HIGH);
#define LED2_OFF digitalWrite(LED2, LOW);
#define LED3_ON digitalWrite(LED3, HIGH);
#define LED3_OFF digitalWrite(LED3, LOW);

static SDClass theSD;
SpGnss Gnss;
bool got_time = false;

static uint8_t *raw_image = NULL;
static uint8_t *input_image = NULL;
static ei_impulse_result_t ei_result = {0};
int take_picture_count = 0;

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
  bool spotted_object = false;
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
  float score = 0;
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

  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d "
            "ms.): \n",
            ei_result.timing.dsp, ei_result.timing.classification,
            ei_result.timing.anomaly);

  bool bb_found = ei_result.bounding_boxes[0].value > 0;

  LED1_OFF
  LED2_OFF
  LED3_OFF

  spotted_object = true;

  for (size_t ix = 0; ix < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ix++) {
    auto bb = ei_result.bounding_boxes[ix];
    if (bb.value > score) {
      score = bb.value;
    }

    if (bb.value == 0) {
      continue;
    }
    if (bb.value > 0.2) {
      spotted_object = true;
    }
    if (bb.value > 0.7) {
      LED1_ON
    }
    if (bb.value > 0.8) {
      LED2_ON
    }
    if (bb.value > 0.9) {
      LED3_ON
    }
    ei_printf("    %s (", bb.label);
    ei_printf_float(bb.value);
    ei_printf(") [ x: %u, y: %u, width: %u, height: %u ]\n", bb.x, bb.y,
              bb.width, bb.height);
  }

  if (!bb_found) {
    ei_printf("    No objects found\n");
  }

  if (theSD.begin()) {
    int short_score = floor(score * 100);

    sprintf(filename, "%d-%d.888", take_picture_count, short_score);

    ei_printf("INFO: saving %s to SD card... size: %d\n", filename,
              96 * 96 * 3);
    theSD.remove(filename);
    File myFile = theSD.open(filename, FILE_WRITE);
    myFile.write(raw_image, 96 * 96 * 3);
    myFile.close();
  } else {
    Serial.println("failed to compress and save image, check that camera and "
                   "SD card are connected properly");
  }
  take_picture_count++;
}
