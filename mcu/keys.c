#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "keys.h"
#include "pico/stdlib.h"

#define KEYS_DATA_PIN 18
#define KEYS_SCK_PIN 17
#define KEYS_MATRIX_MASK 0x7F
#define KEYS_MATRIX_TOTAL (KEYS_MATRIX_MASK + 1)
#define KEYS_MATRIX_VALID (KEYS_MATRIX_TOTAL - 0x10)
#define KEYS_PRESSES__RESET 0
#define KEYS_PRESSES_MAX 16

// Apple IIe keyboard codes
#define UP   0x0B
#define DOWN 0x0A
#define LEFT 0x08
#define RGHT 0x15
#define RTN  0x0D
#define TAB  0x09
#define ESC  0x1B
#define DEL  0x7F

// Matrix scan codes index
#define KEYS_SHIFT_LEFT 0x10
#define KEYS_SHIFT_RIGHT 0x1B
#define KEYS_CTRL_LEFT 0x41
#define KEYS_CTRL_RIGHT 0x09
#define KEYS_CAPS_LOCK 0x20
#define KEYS_F1 0x00 // f1 = pause
#define KEYS_F2 0x42 // f2 = resume
#define KEYS_F3 0x43 // shift + f3 = reset
#define KEYS_F4 0x44 // shift + f4 = menu
#define KEYS_F5 0x45 // shift + f5 = reboot

#define KEYS_OFFSET_CAPS_LOCK_ON  0x00
#define KEYS_OFFSET_CAPS_LOCK_OFF 0x70
#define KEYS_OFFSET_CAPS_SHIFT    0xE0

#define KEYS_SHIFT_RIGHT_BIT (1<<0)
#define KEYS_SHIFT_LEFT_BIT (1<<1)

#define KEYS_DATA_EMPTY 0
#define KEYS_DATA_READY 1
#define KEYS_PRESSED_FALSE 0
#define KEYS_PRESSED_TRUE 1
#define KEYS_USED_FALSE 0
#define KEYS_USED_TRUE 1

#define KEYS_SWITCH_CLOSED 0

uint8_t keys_clk_state = 1;
uint8_t keys_index = 0;
uint8_t keys_index_waiting = 0;
uint8_t consecutive_presses = 0;
uint8_t keys_data[KEYS_MATRIX_TOTAL] = {KEYS_DATA_EMPTY};
uint8_t keys_pressed[KEYS_MATRIX_TOTAL] = {KEYS_PRESSED_FALSE};
uint8_t keys_used[KEYS_MATRIX_TOTAL] = {KEYS_USED_FALSE};
uint8_t keys_iie_offset = 0;

static uint8_t keys_shift = 0;
static uint8_t keys_ctrl_left = 0;
static uint8_t keys_ctrl_right = 0;
static uint8_t keys_caplock = 1;

static uint8_t keys_pause = 0;
static uint8_t keys_resume = 0;
static uint8_t keys_reset = 0;
static uint8_t keys_reboot = 0;
static uint8_t keys_menu = 0;

static const uint8_t keys_iie[KEYS_MATRIX_VALID * 3] =
{
// Caps lock on (default on power up)

//0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  ' ', 0x00, 0x00, 0x00, 0x00, LEFT, DOWN, RGHT, 0x00, 0x00, //0x00
  0x00,  'Z',  'X',  'C',  'V',  'B',  'N',  'M',  ',',  '.',  '/', 0x00, 0x00,   UP, 0x00, 0x00, //0x10
  0x00,  'A',  'S',  'D',  'F',  'G',  'H',  'J',  'K',  'L',  ';', '\'',  RTN, '\\', 0x00, 0x00, //0x20
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x30
   ESC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  DEL, 0x00, 0x00, //0x40
   '`',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',  '-',  '=', 0x00, 0x00, 0x00, //0x50
   TAB,  'Q',  'W',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',  'P',  '[',  ']', 0x00, 0x00, 0x00, //0x60

// Caps lock off

//0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  ' ', 0x00, 0x00, 0x00, 0x00, LEFT, DOWN, RGHT, 0x00, 0x00, //0x00
  0x00,  'z',  'x',  'c',  'v',  'b',  'n',  'm',  ',',  '.',  '/', 0x00, 0x00,   UP, 0x00, 0x00, //0x10
  0x00,  'a',  's',  'd',  'f',  'g',  'h',  'j',  'k',  'l',  ';', '\'',  RTN, '\\', 0x00, 0x00, //0x20
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x30
   ESC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  DEL, 0x00, 0x00, //0x40
   '`',  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',  '-',  '=', 0x00, 0x00, 0x00, //0x50
   TAB,  'q',  'w',  'e',  'r',  't',  'y',  'u',  'i',  'o',  'p',  '[',  ']', 0x00, 0x00, 0x00, //0x60

// shift

//0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  ' ', 0x00, 0x00, 0x00, 0x00, LEFT, DOWN, RGHT, 0x00, 0x00, //0x00
  0x00,  'Z',  'X',  'C',  'V',  'B',  'N',  'M',  '<',  '>',  '?', 0x00, 0x00,   UP, 0x00, 0x00, //0x10
  0x00,  'A',  'S',  'D',  'F',  'G',  'H',  'J',  'K',  'L',  ':', '\"',  RTN,  '|', 0x00, 0x00, //0x20
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //0x30
   ESC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  DEL, 0x00, 0x00, //0x40
   '~',  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',  '_',  '+', 0x00, 0x00, 0x00, //0x50
   TAB,  'Q',  'W',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',  'P',  '{',  '}', 0x00, 0x00, 0x00, //0x60
};

void keys_init(void)
{
    gpio_init(KEYS_DATA_PIN);
    gpio_init(KEYS_SCK_PIN);
    gpio_set_dir(KEYS_DATA_PIN, GPIO_IN);
    gpio_set_dir(KEYS_SCK_PIN, GPIO_OUT);
}

void keys_clk_low(void)
{
    gpio_put(KEYS_SCK_PIN, 0);
}

void keys_clk_high(void)
{
    gpio_put(KEYS_SCK_PIN, 1);
}

void index_update(void)
{
    bool end_of_scan = (consecutive_presses == KEYS_PRESSES_MAX? true : false);

    if (end_of_scan)
    {
        keys_index = 0;
    }
    else
    {
        keys_index = (keys_index + 1) & KEYS_MATRIX_MASK;
    }
}

void either_shift_key_down_test(void)
{
    if (keys_index == KEYS_SHIFT_LEFT)
    {
        keys_shift |= KEYS_SHIFT_LEFT_BIT;
    }

    if (keys_index == KEYS_SHIFT_RIGHT)
    {
        keys_shift |= KEYS_SHIFT_RIGHT_BIT;
    }
}

void both_shift_keys_up_test(void)
{
    if (keys_index == KEYS_SHIFT_LEFT)
    {
        keys_shift &= ~KEYS_SHIFT_LEFT_BIT;
    }

    if (keys_index == KEYS_SHIFT_RIGHT)
    {
        keys_shift &= ~KEYS_SHIFT_RIGHT_BIT;
    }
}

void key_not_pressed(void)
{
    keys_pressed[keys_index] = KEYS_PRESSED_FALSE;
    keys_used[keys_index] = KEYS_USED_FALSE;
    keys_data[keys_index] = KEYS_DATA_EMPTY;
}

void keys_update(void)
{
    keys_clk_state ^= 1;
    if (keys_clk_state)
    {
        keys_clk_high();
        return;
    }

    keys_clk_low();

    bool key_press_detected = gpio_get(KEYS_DATA_PIN) == KEYS_SWITCH_CLOSED? true : false;

    if (key_press_detected)
    {
        consecutive_presses++;

        either_shift_key_down_test();

        if (keys_data[keys_index] == KEYS_DATA_EMPTY)
        {
            if (keys_index == KEYS_CAPS_LOCK)
            {
                keys_caplock ^= 1;
            }
            else
            {
                keys_pressed[keys_index] = KEYS_PRESSED_TRUE;
                if (keys_index < KEYS_MATRIX_VALID)
                {
                    keys_index_waiting = keys_index;
                }
            }
        }
        keys_data[keys_index] = KEYS_DATA_READY;
    }
    else
    {
        consecutive_presses = KEYS_PRESSES__RESET;
        both_shift_keys_up_test();
        key_not_pressed();
    }

    index_update();
}

uint8_t keys_data_waiting(void)
{
    if (keys_used[keys_index_waiting] == KEYS_USED_FALSE)
    {
        return keys_index_waiting;
    }
    return 0;
}

uint8_t keys_data_get(void)
{
    uint8_t key = 0;

    if (keys_caplock)
    {
        keys_iie_offset = KEYS_OFFSET_CAPS_LOCK_ON;
    }
    else
    {
        keys_iie_offset = KEYS_OFFSET_CAPS_LOCK_OFF;
    }

    if (keys_shift)
    {
        keys_iie_offset = KEYS_OFFSET_CAPS_SHIFT;
    }

    key = keys_iie[keys_index_waiting + keys_iie_offset];
    keys_used[keys_index_waiting] = KEYS_USED_TRUE;
    keys_index_waiting = 0;
    return key;
}

void keys_operation_update(KeysOperation *operation, uint8_t *data)
{
    *operation = KEYS_MAIN_NULL;
    *data = 0;

    if (keys_pause)
    {
        keys_pause = 0;
        *operation = KEYS_MAIN_PAUSE;
        return;
    }

    if (keys_resume)
    {
        keys_resume = 0;
        *operation = KEYS_MAIN_RESUME;
        return;
    }

    if (keys_reset)
    {
        keys_reset = 0;
        *operation = KEYS_MAIN_RESET;
        return;
    }

    if (keys_menu)
    {
        keys_menu = 0;
        *operation = KEYS_MAIN_MENU;
        return;
    }

    if (keys_reboot)
    {
        keys_reboot = 0;
        *operation = KEYS_MAIN_REBOOT;
        return;
    }

    if (keys_data_waiting())
    {
        *operation = KEYS_KEYBOARD_KEY;
        *data = keys_data_get();
    }
}
