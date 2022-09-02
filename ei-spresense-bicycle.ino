#include <bicycle-street-fomo_inferencing.h>

/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */

#include <Camera.h>
#include <LowPower.h>
#include <SDHCI.h>
#include <stdint.h>
#include <stdlib.h>

#define DEBUG true
#define TAKE_PHOTOS true

static SDClass theSD;
#define DWORD_ALIGN_PTR(a)                                                     \
  ((a & 0x3) ? (((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

/* Defines to center crop and resize a image to the Impulse image size the
   speresense hardware accelerator

   NOTE: EI_CLASSIFIER_INPUT width and height must be less than RAW_HEIGHT *
   SCALE_FACTOR, and must simultaneously meet the requirements of the spresense
   api:
   https://developer.sony.com/develop/spresense/developer-tools/api-reference/api-references-arduino/group__camera.html#ga3df31ea63c3abe387ddd1e1fd2724e97
*/

#define LED0_ON                                                                \
  digitalWrite(LED0, HIGH); /* Macros for Switch on/off Board LEDs */
#define LED0_OFF digitalWrite(LED0, LOW);
#define LED1_ON digitalWrite(LED1, HIGH);
#define LED1_OFF digitalWrite(LED1, LOW);
#define LED2_ON digitalWrite(LED2, HIGH);
#define LED2_OFF digitalWrite(LED2, LOW);
#define LED3_ON digitalWrite(LED3, HIGH);
#define LED3_OFF digitalWrite(LED3, LOW);

#define PICTURE_SIGNALLING_ON LED0_ON   /* Switch on LED when take a photo */
#define PICTURE_SIGNALLING_OFF LED0_OFF /* Switch off LED when taken photo */

//#define RAW_WIDTH EI_CLASSIFIER_INPUT_WIDTH   // CAM_IMGSIZE_QVGA_H
//#define RAW_HEIGHT EI_CLASSIFIER_INPUT_HEIGHT // CAM_IMGSIZE_QVGA_V

#define SCALE_FACTOR 1
#define RAW_WIDTH CAM_IMGSIZE_QVGA_H
#define RAW_HEIGHT CAM_IMGSIZE_QVGA_V
#define CLIP_WIDTH (EI_CLASSIFIER_INPUT_WIDTH * SCALE_FACTOR)
#define CLIP_HEIGHT (EI_CLASSIFIER_INPUT_HEIGHT * SCALE_FACTOR)
#define OFFSET_X  ((RAW_WIDTH - CLIP_WIDTH) / 2)
#define OFFSET_Y  ((RAW_HEIGHT - CLIP_HEIGHT) / 2)


// enable for very verbose logging from edge impulse sdk
#define DEBUG_NN false
#define GRAYSCALE false

#define CLASSIFIER_THRESHOLD 0.7
#define CLASSIFIER_ANIMAL_INDEX 0
#define FEATURE_SIZE 9216



/* static variables */
static CamImage sized_img;
//float features[FEATURE_SIZE];

static ei_impulse_result_t ei_result = {0};
int take_picture_count = 0;
bool spotted_object = false;
float score = 0;
/* prototypes */
void printError(enum CamErr err);
void CamCB(CamImage img);


/**
   @brief      Convert monochrome data to rgb values

   @param[in]  mono_data  The mono data
   @param      r          red pixel value
   @param      g          green pixel value
   @param      b          blue pixel value
*/
static inline void mono_to_rgb(uint8_t mono_data, uint8_t *r, uint8_t *g,
                               uint8_t *b) {
  uint8_t v = mono_data;
  *r = *g = *b = v;
}
/*
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}*/

void r565_to_rgb(uint16_t color, uint8_t *r, uint8_t *g, uint8_t *b) {
  *r = (color & 0xF800) >> 8;
  *g = (color & 0x07E0) >> 3;
  *b = (color & 0x1F) << 3;
}


/**
 * @brief      Convert RGB565 raw camera buffer to RGB888
 *
 * @param[in]   offset       pixel offset of raw buffer
 * @param[in]   length       number of pixels to convert
 * @param[out]  out_buf      pointer to store output image
 */

// this is from the Nano BLE example
int ei_camera_cutout_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 2; 
    size_t bytes_left = length;
    size_t out_ptr_ix = 0;
     // grab the value and convert to r/g/b
    uint8_t *buffer = sized_img.getImgBuff();
    // read byte for byte
    while (bytes_left != 0) {

        uint16_t pixel = (buffer[pixel_ix] << 8) | buffer[pixel_ix+1];
        uint8_t r, g, b;
        r = ((pixel >> 11) & 0x1f) << 3;
        g = ((pixel >> 5) & 0x3f) << 2;
        b = (pixel & 0x1f) << 3;

        // then convert to out_ptr format
        float pixel_f = (r << 16) + (g << 8) + b;
        out_ptr[out_ptr_ix] = pixel_f;

        // and go to the next pixel
        out_ptr_ix++;
        pixel_ix+=2;
        bytes_left--;
    }

    // and done!
    return 0;
}

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

/**
   @brief run inference on the static sized_image buffer using the provided
   impulse
*/
static void ei_wildlife_camera_classify(bool debug) {
  ei::signal_t signal;
  spotted_object = false;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_cutout_get_data; //&raw_feature_get_data; //&ei_camera_cutout_get_data;

  score = 0;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, DEBUG_NN);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERROR: Failed to run classifier (%d)\n", err);
    return;
  }
  // print the predictions
  if (debug) {
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d "
              "ms.): \n",
              ei_result.timing.dsp, ei_result.timing.classification,
              ei_result.timing.anomaly);
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
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
#else
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
      ei_printf("    %s: ", ei_result.classification[ix].label);
      ei_printf_float(ei_result.classification[ix].value);
      ei_printf("\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(ei_result.anomaly);
    ei_printf("\n");
#endif
#endif
  }

  return;
}

/**
 * @brief callback that checks for the presence of an animal in the camera
 * preview window, and then executes ei_wildlife_camera_snapshot() if found
 */
void CamCB(CamImage img) {}

/**
   @brief initialize the wildlife camera for continuous monitoring of video feed
*/
void ei_wildlife_camera_start_continuous(bool debug) {
  CamErr err;
  ei_printf("Starting the camera:\n");
  err = theCamera.begin(1, CAM_VIDEO_FPS_5, RAW_WIDTH, RAW_HEIGHT,
                        CAM_IMAGE_PIX_FMT_RGB565);

  // err = theCamera.begin(1, CAM_VIDEO_FPS_5, EI_CLASSIFIER_INPUT_WIDTH,
  // EI_CLASSIFIER_INPUT_HEIGHT,CAM_IMAGE_PIX_FMT_YUV422);
  if (err && debug)
    printError(err);

  theCamera.setAutoISOSensitivity(true);
  theCamera.setAutoWhiteBalance(true);

  ei_printf("Starting sending data:\n");

  // start streaming the preview images to the classifier
  // err = theCamera.startStreaming(true, CamCB);

  if (err)
    printError(err);

  ei_printf("Set format:\n");
  // still image format must be jpeg to allow for compressed storage/transmit

  err = theCamera.setStillPictureImageFormat(RAW_WIDTH, RAW_HEIGHT,
                                             CAM_IMAGE_PIX_FMT_YUV422); //CAM_IMAGE_PIX_FMT_RGB565);
  // CAM_IMAGE_PIX_FMT_YUV422);
  // CAM_IMAGE_PIX_FMT_RGB565);
  // CAM_IMAGE_PIX_FMT_GRAY);
  // CAM_IMAGE_PIX_FMT_RGB565);
  if (err)
    printError(err);

  Serial.println("INFO: started wildlife camera recording");
}

void ei_capture_classify() {
  CamErr err;
  char filename[400];
  bool debug = true;
  // snapshot and save a jpeg
  CamImage img = theCamera.takePicture();

  if (!img.isAvailable())
    return; // fast path if image is no longer ready

  ei_printf("resize:\n");
  err = img.clipAndResizeImageByHW(sized_img
                                   , OFFSET_X, OFFSET_Y
                                   , OFFSET_X + CLIP_WIDTH - 1
                                   , OFFSET_Y + CLIP_HEIGHT - 1
                                   , EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
  if (err) printError(err);
  err = sized_img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  if (err) printError(err);
  /*memcpy(features, img.getImgBuff(), img.getImgSize());

  uint8_t r, g, b;
  for (int i = 0; i < FEATURE_SIZE; i++) {
    r565_to_rgb(features[i], &r, &g, &b);
    // then convert to out_ptr format
    float pixel_f = (r << 16) + (g << 8) + b;
    features[i] = pixel_f;
  }*/

ei_wildlife_camera_classify(true);

  LED0_ON

  if (spotted_object && theSD.begin() && sized_img.isAvailable()) {
    int short_score = floor(score * 100);
    /*sprintf(filename, "%d-%d.rgb", take_picture_count, short_score);
    if (debug)
      ei_printf("INFO: saving %s to SD card... size: %d", filename, sizeof(features));
    theSD.remove(filename);
    File myFile = theSD.open(filename, FILE_WRITE);
    myFile.write((const char*) features, sizeof(features));
    myFile.close();*/

    sprintf(filename, "%d-%d.565", take_picture_count, short_score);
    if (debug)
      ei_printf("INFO: saving %s to SD card... size: %d\n", filename, sized_img.getImgSize());
    theSD.remove(filename);
    File myFile = theSD.open(filename, FILE_WRITE);
    myFile.write(sized_img.getImgBuff() ,sized_img.getImgSize());
    myFile.close();
  } else if (debug) {
    Serial.println("failed to compress and save image, check that camera and "
                   "SD card are connected properly");
  }
  LED0_OFF

  


  take_picture_count++;
}


void printClockMode() {
  clockmode_e mode = LowPower.getClockMode();

  Serial.println("--------------------------------------------------");
  Serial.print("clock mode: ");
  switch (mode) {
  case CLOCK_MODE_156MHz:
    Serial.println("156MHz");
    break;
  case CLOCK_MODE_32MHz:
    Serial.println("32MHz");
    break;
  case CLOCK_MODE_8MHz:
    Serial.println("8MHz");
    break;
  }
}

void setup() {

  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("INFO: wildlife_camera initializing on wakeup...");
  }

  /* Initialize SD */

  while (!theSD.begin()) {

    if (DEBUG)
      Serial.println("Insert SD card.");
  }

  // summary of inferencing settings (from model_metadata.h)
  ei_printf("Inferencing settings:\n");
  ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT);
  ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  ei_printf("\tNo. of classes: %d\n",
            sizeof(ei_classifier_inferencing_categories) /
                sizeof(ei_classifier_inferencing_categories[0]));
  ei_printf("\tRaw Image Width: %d Height: %d\n", RAW_WIDTH, RAW_HEIGHT);
#if defined(CMSIS_NN)
  ei_printf("Enabled CMSIS_NN\n");
#endif
#if defined(EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN)
  ei_printf("Enabled EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN : %d\n",
            EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN);
#endif
  /*LowPower.begin();
 // Set the highest clock mode
 printClockMode();
LowPower.clockMode(CLOCK_MODE_156MHz);
printClockMode();*/
  /*if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    ei_printf("The size of your 'features' array is not correct. Expected %lu "
              "items, but had %lu\n",
              EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
              sizeof(features) / sizeof(float));
    delay(1000);
    return;
  }*/
  ei_wildlife_camera_start_continuous(DEBUG);
}

void loop() {
  // ei_capture_classify();
  ei_capture_classify();
  // sleep(500);
}
