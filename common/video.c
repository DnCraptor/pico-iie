#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "video.h"
#include "video_char_data.h"

#define VIDEO_BYTES_PER_LINE 40
#define VIDEO_RESOLUTION_X 280
#define VIDEO_RESOLUTION_Y 192
#define VIDEO_BUFFER_SIZE (VIDEO_RESOLUTION_X)

#define TEXT_BYTES 8
#define TEXT_BYTES_MASK 0x07

#define VGA_COLORS 3
#define VGA_LINE_BYTES 50
#define SCAN_LINE_BYTES (VGA_COLORS * VGA_LINE_BYTES)
#define H_SYNC_COUNTER_START 410 //406 to 414, 20 counts = 1 pixel

#define H_BUFFER_OFFSET 4
#define H_BUFFER_VISIBLE 35

enum VGA_COLOR_INDEX
{
  VGA_RED = 0,
  VGA_GREEN,
  VGA_BLUE,
};

enum HCOLOR
{
    BLACK = 0,
    PURPLE,
    GREEN,
    GREEN_1,
    PURPLE_1,
    BLUE,
    ORANGE_0,
    ORANGE_1,
    BLUE_1,
    WHITE,
    HCOLOR_LENGTH,
};

static uint8_t vga_hcolor[VGA_COLORS][HCOLOR_LENGTH]   =
{
    {0,1,0,0,1,0,1,1,0,1}, //red
    {0,0,1,1,0,1,0,0,1,1}, //green
    {0,1,0,0,1,1,0,0,1,1}, //blue
};

static uint8_t video_buffer[VIDEO_BUFFER_SIZE] = {0};
static uint8_t vga_buffer[VGA_COLORS][VGA_LINE_BYTES];


static inline void pixel_byte(uint16_t index, uint8_t *red, uint8_t *green, uint8_t *blue)
{
  uint16_t offset = index * 8;
  *red = 0;
  *green = 0;
  *blue = 0;

  for(int i = 0; i < 8; i++)
  {
    *red |= (vga_hcolor[VGA_RED][video_buffer[offset + i]]) << (7 - i);
    *green |= (vga_hcolor[VGA_GREEN][video_buffer[offset + i]]) << (7 - i);
    *blue |= (vga_hcolor[VGA_BLUE][video_buffer[offset + i]]) << (7 - i);
  }
}

void video_scan_line_pixel_conversion(void)
{
    for(int i = 0; i < H_BUFFER_VISIBLE; i++)
    {
        pixel_byte(
            i,
            &vga_buffer[VGA_RED][i + H_BUFFER_OFFSET],
            &vga_buffer[VGA_GREEN][i + H_BUFFER_OFFSET],
            &vga_buffer[VGA_BLUE][i + H_BUFFER_OFFSET]);
    }
}

void video_scan_line_zero(void)
{
    memset(vga_buffer, 0, SCAN_LINE_BYTES);
}

void video_scan_line_buffer_transfer(uint8_t *buffer)
{
    memcpy(buffer, vga_buffer, SCAN_LINE_BYTES);
}

void video_hires_line_update(uint16_t video_line_number, uint8_t *video_line_data)
{
    uint16_t data = 0;
    uint16_t pixel_pre = 0;
    uint16_t pixel_post =  0;
    uint16_t data_extended = 0;
    uint8_t address_odd = 0;
    uint8_t color_offset = 0;
    uint8_t color = 0;
    uint8_t pixel = 0;
    uint8_t pixel_left1 = 0;
    uint8_t pixel_right1 = 0;

    if (video_line_number < VIDEO_RESOLUTION_Y)
    {
        for(int j = 0; j < VIDEO_BYTES_PER_LINE; j++)
        {
            data = video_line_data[j];
            pixel_pre = (video_line_data[j - 1] & 0x60)>>5;
            pixel_post = video_line_data[j + 1] & 3;
            address_odd = (j & 1)<<1;
            color_offset = (data & 0x80)>>5;
            data_extended = pixel_pre + ((data & 0x7F)<<2) + (pixel_post<<9);

            for(int i = 0; i < 7; i++)
            {
                color = BLACK;
                pixel = (data_extended >> (i + 2)) & 1;
                pixel_left1 = (data_extended >> (i + 1)) & 1;
                pixel_right1 = (data_extended >> (i + 3)) & 1;

                if (pixel)
                {
                    if (pixel_left1 || pixel_right1)
                    {
                        color = WHITE;
                    }
                    else
                    {
                        color = color_offset + address_odd +  (i & 1) + 1;
                    }
                }
                else
                {
                    if (pixel_left1 && pixel_right1)
                    {

                        color = color_offset + address_odd +  1 - (i & 1) + 1;
                    }
                }
                video_buffer[j*7 + i] |= color;
            }
        }
    }
}

void video_text_line_update(uint16_t video_line_number, uint8_t *video_line_data)
{
    uint16_t text_char = 0;
    uint16_t data = 0;
    uint8_t color = 0;
    uint8_t pixel_color = WHITE;
    uint8_t pixel = 0;
    uint16_t rom_char = 0;
    uint16_t rom_char_offset = 0;

    if (video_line_number < VIDEO_RESOLUTION_Y)
    {
        for(int j = 0; j < VIDEO_BYTES_PER_LINE; j++)
        {
            text_char = video_line_data[j];

            if((text_char & 0xC0) == 0x40)
            {
                // text_char &= 0x3F;
                // if(text_char < 0x20)
                // {
                //     text_char += 0x40;
                // }
            }

            rom_char = text_char * TEXT_BYTES;
            rom_char_offset = video_line_number & TEXT_BYTES_MASK;
            data = apple2e_char_rom[rom_char + rom_char_offset];

            for(int i = 0; i < 7; i++)
            {
                color = pixel_color;
                pixel = (data >> (i)) & 1;
                if (pixel)
                {
                    color = BLACK;
                }
                video_buffer[j*7 + i] |= color;
            }
        }
    }
}

void video_buffer_clear(void)
{
    memset(video_buffer, 0, VIDEO_BUFFER_SIZE);
}

void video_buffer_get(uint8_t *buffer)
{
    memcpy(buffer, video_buffer, VIDEO_BUFFER_SIZE);
}