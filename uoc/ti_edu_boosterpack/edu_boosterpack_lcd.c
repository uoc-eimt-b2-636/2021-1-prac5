/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/
#include "edu_boosterpack_lcd.h"

/* Graphic library context */
static Graphics_Context g_sContext;

static SemaphoreHandle_t mutex_LCD;

void edu_boosterpack_lcd_init(void)
{
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);

    Graphics_clearDisplay(&g_sContext);

    // Create LCD mutex
    mutex_LCD = xSemaphoreCreateBinary();

    // Se puede acceder ya a la region excluida
    xSemaphoreGive(mutex_LCD);
}

void edu_bootsterpack_lcd_print(char* string, uint8_t coord_x, uint8_t coord_y)
{

    if (coord_x > LCD_HORIZONTAL_MAX - 1) {
        coord_x = LCD_HORIZONTAL_MAX - 1;
    }

    if (coord_y > LCD_VERTICAL_MAX - 1){
        coord_y = LCD_VERTICAL_MAX - 1;
    }

    xSemaphoreTake(mutex_LCD, portMAX_DELAY);
    Graphics_drawString(&g_sContext, (int8_t *) string, AUTO_STRING_LENGTH,
                        coord_x, coord_y, OPAQUE_TEXT);
    xSemaphoreGive(mutex_LCD);
}

void edu_bootsterpack_lcd_clear()
{
    xSemaphoreTake(mutex_LCD, portMAX_DELAY);
    Graphics_clearDisplay(&g_sContext);
    xSemaphoreGive(mutex_LCD);

}

