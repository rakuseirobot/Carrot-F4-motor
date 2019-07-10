/*
 * neopixel.cpp
 *
 *  Created on: Jul 10, 2019
 *      Author: shun2
 */


#include <stdio.h>
#include "neopixel.hpp"
#include "stm32f4xx_hal.h"

static TIM_HandleTypeDef*  htim;
static DMA_HandleTypeDef*  hdma;
volatile static uint32_t*  pccr;
static uint32_t  dier;
static uint32_t  ccer;
static uint8_t*  srcRGB;
static int  srcRGBLen;
static uint8_t  pwmcount[48];
static int  pos;
volatile static bool  busy;


extern DMA_HandleTypeDef hdma_tim3_ch3;

static void  dmaCallback (DMA_HandleTypeDef* _hdma) {
    busy = false;
}

static void  update () {
    uint8_t*  p;
    uint8_t  c;
    ++ srcRGB;      // G
    c = *srcRGB;
    p = pwmcount + pos;
    *(p++) = (c & 0x80) ? 8 : 4;
    *(p++) = (c & 0x40) ? 8 : 4;
    *(p++) = (c & 0x20) ? 8 : 4;
    *(p++) = (c & 0x10) ? 8 : 4;
    *(p++) = (c & 0x08) ? 8 : 4;
    *(p++) = (c & 0x04) ? 8 : 4;
    *(p++) = (c & 0x02) ? 8 : 4;
    *(p++) = (c & 0x01) ? 8 : 4;
    -- srcRGB;      // R
    c = *srcRGB;
    *(p++) = (c & 0x80) ? 8 : 4;
    *(p++) = (c & 0x40) ? 8 : 4;
    *(p++) = (c & 0x20) ? 8 : 4;
    *(p++) = (c & 0x10) ? 8 : 4;
    *(p++) = (c & 0x08) ? 8 : 4;
    *(p++) = (c & 0x04) ? 8 : 4;
    *(p++) = (c & 0x02) ? 8 : 4;
    *(p++) = (c & 0x01) ? 8 : 4;
    srcRGB += 2;        // B
    c = *srcRGB;
    *(p++) = (c & 0x80) ? 8 : 4;
    *(p++) = (c & 0x40) ? 8 : 4;
    *(p++) = (c & 0x20) ? 8 : 4;
    *(p++) = (c & 0x10) ? 8 : 4;
    *(p++) = (c & 0x08) ? 8 : 4;
    *(p++) = (c & 0x04) ? 8 : 4;
    *(p++) = (c & 0x02) ? 8 : 4;
    *(p++) = (c & 0x01) ? 8 : 4;
    ++ srcRGB;
    srcRGBLen -= 3;
    pos += 24;
    if (pos >= 48)
        pos = 0;
}

static void  clear () {
    uint32_t*  p = (uint32_t*)(pwmcount + pos);
    int  i;
    for (i = 0; i < 6; ++ i)
        *(p ++) = 0;
    pos += 24;
    if (pos >= 48)
        pos = 0;
}

static void  initChannel (uint32_t channel) {
    switch (channel) {
    case TIM_CHANNEL_1:
        hdma = htim->hdma[TIM_DMA_ID_CC1];
        pccr = &htim->Instance->CCR1;
        dier = TIM_DIER_CC1DE;
        ccer = TIM_CCER_CC1E;
        break;
    case TIM_CHANNEL_2:
        hdma = htim->hdma[TIM_DMA_ID_CC2];
        pccr = &htim->Instance->CCR2;
        dier = TIM_DIER_CC2DE;
        ccer = TIM_CCER_CC2E;
        break;
    case TIM_CHANNEL_3:
        hdma = htim->hdma[TIM_DMA_ID_CC3];
        pccr = &htim->Instance->CCR3;
        dier = TIM_DIER_CC3DE;
        ccer = TIM_CCER_CC3E;
        break;
    case TIM_CHANNEL_4:
        hdma = htim->hdma[TIM_DMA_ID_CC4];
        pccr = &htim->Instance->CCR4;
        dier = TIM_DIER_CC4DE;
        ccer = TIM_CCER_CC4E;
        break;
    default:;
    }
}

static void  initDMA () {
    *pccr = 0;
    HAL_DMA_RegisterCallback (hdma, HAL_DMA_XFER_CPLT_CB_ID, dmaCallback);
    HAL_DMA_RegisterCallback (hdma, HAL_DMA_XFER_HALFCPLT_CB_ID, dmaCallback);
    htim->Instance->DIER |= dier;
    htim->Instance->CCER |= ccer;
    __HAL_TIM_MOE_ENABLE (htim);
}

void  NeoPixInit (TIM_HandleTypeDef* _htim, uint32_t channel) {
    htim = _htim;
    hdma = NULL;
    pccr = NULL;
    srcRGB = NULL;
    srcRGBLen = 0;
    pos = 0;
    busy = true;
    clear ();
    clear ();
    initChannel (channel);
    initDMA ();
};

void  NeoPixStart (uint8_t* data, int len, bool wait) {
    srcRGB = data;
    srcRGBLen = len;
    pos = 0;
    if (srcRGBLen > 0) {
        update ();
        busy = true;
        __HAL_TIM_SET_COUNTER (htim, 0);
        HAL_DMA_Start_IT (hdma, (uint32_t)pwmcount, (uint32_t)pccr, 48);
        __HAL_TIM_ENABLE (htim);
        while (srcRGBLen > 0) {
            update ();
            busy = true;    // 最適化による影響を避けるためにvolatile
            while (busy) {};
        }
        clear ();
        busy = true;
        while (busy) {};
        if (wait) {
            clear ();
            busy = true;
            while (busy) {};
            busy = true;
            while (busy) {};
        }
        __HAL_TIM_DISABLE (htim);
        HAL_DMA_Abort (hdma);
    }
}

