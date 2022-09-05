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
#include "edge-impulse-sdk/dsp/image/image.hpp"
#define DEBUG true
#define TAKE_PHOTOS true

static SDClass theSD;

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
/* Constant defines -------------------------------------------------------- */
#define SCALE_FACTOR 1
#define RAW_WIDTH CAM_IMGSIZE_QVGA_H
#define RAW_HEIGHT CAM_IMGSIZE_QVGA_V
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           CAM_IMGSIZE_QVGA_H
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           CAM_IMGSIZE_QVGA_V
#define EI_CAMERA_RAW_FRAME_BYTE_SIZE             2





#define CLIP_WIDTH (EI_CLASSIFIER_INPUT_WIDTH * SCALE_FACTOR)
#define CLIP_HEIGHT (EI_CLASSIFIER_INPUT_HEIGHT * SCALE_FACTOR)
#define OFFSET_X  ((RAW_WIDTH - CLIP_WIDTH) / 2)
#define OFFSET_Y  ((RAW_HEIGHT - CLIP_HEIGHT) / 2)


// enable for very verbose logging from edge impulse sdk
#define DEBUG_NN false

/*
** @brief points to the output of the capture
*/
static uint8_t *ei_camera_capture_out = NULL;



static ei_impulse_result_t ei_result = {0};
int take_picture_count = 0;
bool spotted_object = false;
float score = 0;
/* prototypes */
void printError(enum CamErr err);
void CamCB(CamImage img);



#define ALIGN_PTR(p,a)   ((p & (a-1)) ?(((uintptr_t)p + a) & ~(uintptr_t)(a-1)) : p)

/* Edge Impulse ------------------------------------------------------------- */

typedef struct {
    size_t width;
    size_t height;
} ei_device_resize_resolutions_t;





/**
 * @brief      Convert rgb565 data to rgb888
 *
 * @param[in]  src_buf  The rgb565 data
 * @param      dst_buf  The rgb888 data
 * @param      src_len  length of rgb565 data
 */

bool RBG565ToRGB888(uint8_t *src_buf, uint8_t *dst_buf, uint32_t src_len)
{
    uint8_t hb, lb;
    uint32_t pix_count = src_len / 2;
    uint32_t count = 0;
    uint32_t dest_count = 0;

      ei_printf("\timg size: %u\n", src_len);

    
    //for(uint32_t i = 0; i < pix_count; i ++) {
    for(uint32_t i = 76770; i < pix_count; i ++) {
        hb = *src_buf++;
        lb = *src_buf++;

        /**dst_buf++ = hb & 0xF8;
        *dst_buf++ = (hb & 0x07) << 5 | (lb & 0xE0) >> 3;
        *dst_buf++ = (lb & 0x1F) << 3;*/

        ei_printf("Count: %u R: %u  G: %u B: %u\n",count, hb & 0xF8,(hb & 0x07) << 5 | (lb & 0xE0) >> 3, (lb & 0x1F) << 3 ); 
        count++;
        //
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (ei_camera_capture_out[pixel_ix] << 16) + (ei_camera_capture_out[pixel_ix + 1] << 8) + ei_camera_capture_out[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }

    // and done!
    return 0;
}

/**
 * @brief      Determine whether to resize and to which dimension
 *
 * @param[in]  out_width     width of output image
 * @param[in]  out_height    height of output image
 * @param[out] resize_col_sz       pointer to frame buffer's column/width value
 * @param[out] resize_row_sz       pointer to frame buffer's rows/height value
 * @param[out] do_resize     returns whether to resize (or not)
 *
 */
int calculate_resize_dimensions(uint32_t out_width, uint32_t out_height, uint32_t *resize_col_sz, uint32_t *resize_row_sz, bool *do_resize)
{
    size_t list_size = 6;
    const ei_device_resize_resolutions_t list[list_size] = {
        {64, 64},
        {96, 96},
        {160, 120},
        {160, 160},
        {320, 240},
    };

    // (default) conditions
    *resize_col_sz = EI_CAMERA_RAW_FRAME_BUFFER_COLS;
    *resize_row_sz = EI_CAMERA_RAW_FRAME_BUFFER_ROWS;
    *do_resize = false;

    for (size_t ix = 0; ix < list_size; ix++) {
        if ((out_width <= list[ix].width) && (out_height <= list[ix].height)) {
            *resize_col_sz = list[ix].width;
            *resize_row_sz = list[ix].height;
            *do_resize = true;
            break;
        }
    }

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
  err = theCamera.begin(0, CAM_VIDEO_FPS_5, RAW_WIDTH, RAW_HEIGHT,
                        CAM_IMAGE_PIX_FMT_RGB565);

  // err = theCamera.begin(1, CAM_VIDEO_FPS_5, EI_CLASSIFIER_INPUT_WIDTH,
  // EI_CLASSIFIER_INPUT_HEIGHT,CAM_IMAGE_PIX_FMT_YUV422);
  if (err)
    printError(err);

  theCamera.setAutoISOSensitivity(true);
  theCamera.setAutoWhiteBalance(true);

  ei_printf("Starting sending data:\n");



  ei_printf("Set format:\n");
  // still image format must be jpeg to allow for compressed storage/transmit

  err = theCamera.setStillPictureImageFormat(RAW_WIDTH, RAW_HEIGHT,
                                             CAM_IMAGE_PIX_FMT_RGB565); //CAM_IMAGE_PIX_FMT_YUV422); //CAM_IMAGE_PIX_FMT_RGB565);
  // CAM_IMAGE_PIX_FMT_YUV422);
  // CAM_IMAGE_PIX_FMT_RGB565);
  // CAM_IMAGE_PIX_FMT_GRAY);
  // CAM_IMAGE_PIX_FMT_RGB565);
  if (err)
    printError(err);

  Serial.println("INFO: started wildlife camera recording");
}

/**
   @brief run inference on the static sized_image buffer using the provided
   impulse
*/
static void ei_wildlife_camera_classify(bool debug) {

}




bool ei_camera_capture(uint32_t img_width, uint32_t img_height) {
  CamErr err;

  bool debug = true;
      bool do_resize = false;
    bool do_crop = false;

ei_printf("Start Capture\n");

    
    //ei_camera_capture_out = (uint8_t *)ALIGN_PTR((uintptr_t)ei_camera_capture_out, 32);
    
  // snapshot and save a jpeg
    unsigned long StartTime = millis();
  CamImage img = theCamera.takePicture();

  if (!img.isAvailable()){
    return false; // fast path if image is no longer ready
  }
  
    ei_printf("imgsize: %u bufsize: %u Img Pix Fmt: %d\n", img.getImgSize(), img.getImgBuffSize(), img.getPixFormat());
    bool converted = RBG565ToRGB888(img.getImgBuff(), ei_camera_capture_out, img.getImgSize());
    
    if(!converted){
        ei_printf("ERR: Conversion failed\n");
        return false;
    } else {
      ei_printf("ESuccessfully converted to RGB888\n");
    }
        
    uint32_t resize_col_sz;
    uint32_t resize_row_sz;
    // choose resize dimensions
    /*int res = calculate_resize_dimensions(img_width, img_height, &resize_col_sz, &resize_row_sz, &do_resize);
    ei_printf("Resize results: Raw Image Width: %d Height: %d Resize Col: %d, Resize Row: %d, Do Resize: %d\n", img_width, img_height, resize_col_sz, resize_row_sz, do_resize);
    if (res) {
        ei_printf("ERR: Failed to calculate resize dimensions (%d)\r\n", res);
        return false;
    }

    if ((img_width != resize_col_sz)
        || (img_height != resize_row_sz)) {
        do_crop = true;
    }
*/

    //if (do_resize) {
/*
          ei::image::processing::crop_and_interpolate_rgb888(
          ei_camera_capture_out,
          EI_CAMERA_RAW_FRAME_BUFFER_COLS,
          EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
          ei_camera_capture_out,
          img_width,
          img_height);*/
          //resize_col_sz,
          //resize_row_sz);
    //}



  //ei_wildlife_camera_classify(true);
  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;
  ei_printf("Capture and Convert Time: %u ms \n", ElapsedTime);


  
  //ei_free(ei_camera_capture_out);
ei_printf("End Capture\n");
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
/*ei_camera_capture_out = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
if (ei_camera_capture_out == NULL)
{ 
  ei_printf("MALLOC FAILED!!\n");
  exit(0);
}*/
ei_printf("Capture Buffer Size: %d \n", EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * 3 + 32);
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
  ei_sleep(2000);
  ei_printf("Start Loop\n");

  ei::signal_t signal;
  spotted_object = false;
  char filename[400];
  
 ei_printf("Start Classify\n");
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data; 
  score = 0;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT) == false) {
        ei_printf("Failed to capture image\r\n");
        return;
    }

  /*
  EI_IMPULSE_ERROR err = run_classifier(&signal, &ei_result, DEBUG_NN);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERROR: Failed to run classifier (%d)\n", err);
    return;
  }
  // print the predictions

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
*/
  ei_printf("End Classify\n");
  LED0_ON
/*
  if (spotted_object && theSD.begin() && sized_img.isAvailable()) {
    int short_score = floor(score * 100);

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
  }*/
  LED0_OFF


ei_printf("End Loop\n");
}
