#include <cstdlib>
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/flash.h>
#include <hardware/structs/vreg_and_chip_reset.h>
#include <pico/bootrom.h>
#include <pico/time.h>
#include <pico/multicore.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include "graphics.h"
#include "psram_spi.h"

extern "C" {
#include "ps2.h"
#include "nespad.h"
#include "memory.h"
#include "ff.h"
#include "debug.h"
#include "util_Wii_Joy.h"
}

static FATFS fs;
semaphore vga_start_semaphore;
#define DISP_WIDTH (320)
#define DISP_HEIGHT (240)
uint16_t SCREEN[TEXTMODE_ROWS][TEXTMODE_COLS];

pwm_config config = pwm_get_default_config();
void PWM_init_pin(uint8_t pinN, uint16_t max_lvl) {
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    pwm_config_set_clkdiv(&config, 1.0);
    pwm_config_set_wrap(&config, max_lvl); // MAX PWM value
    pwm_init(pwm_gpio_to_slice_num(pinN), &config, true);
}

void inInit(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

static uint8_t shiftsStates = 0;
static bool capsLocked = 0;

#define  ESC 0x1B
#define ENTR 0x0D
#define BKSP 0x08
#define  TAB 0x09
#define   F1 0x70
#define   F2 0x71
#define   F3 0x72
#define   F4 0x73
#define   F5 0x74
#define   F6 0x75
#define   F7 0x76
#define   F8 0x77
#define   F9 0x78
#define  F10 0x79
#define  F11 0x7A
#define  F12 0x7B
#define  DEL 0x2E

static const uint8_t key_iee[0x80] = {
    // 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
/*0*/  0x00,  ESC,  '1',  '2',  '3',  '4',  '5',  '6',  '7',  '8',  '9',  '0',  '-',  '=', BKSP,  TAB,
/*1*/   'q',  'w',  'e',  'r',  't',  'y',  'u',  'i',  'o',  'p',  '[',  ']', ENTR, 0x00,  'a',  's',
/*2*/   'd',  'f',  'g',  'h',  'j',  'k',  'l',  ';', '\'',  '`', 0x00, '\\',  'z',  'x',  'c',  'v',
/*3*/   'b',  'n',  'm',  ',',  '.',  '/', 0x00,  '*', 0x00,  ' ', 0x00,   F1,   F2,   F3,   F4,   F5,
/*4*/    F6,   F7,   F8,   F9,  F10, 0x00, 0x00,  '7',  '8',  '9',  '-', '4',  '5',  '6',  '+',  '1',
/*5*/   '2',  '3',  '0',  DEL, 0x00, 0x00, 0x00,  F11,  F12, 0x00,
};

static const uint8_t key_iee_shift[0x80] = {
    // 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
/*0*/  0x00,  ESC,  '!',  '@',  '#',  '$',  '%',  '^',  '&',  '*',  '(',  ')',  '_',  '+', BKSP,  TAB,
/*1*/   'Q',  'W',  'E',  'R',  'T',  'Y',  'U',  'I',  'O',  'P',  '{',  '}', ENTR, 0x00,  'A',  'S',
/*2*/   'D',  'F',  'G',  'H',  'J',  'K',  'L',  ':',  '"',  '~', 0x00,  '|',  'Z',  'X',  'C',  'V',
/*3*/   'B',  'N',  'M',  '<',  '>',  '?', 0x00,  '*', 0x00,  ' ', 0x00,   F1,   F2,   F3,   F4,   F5,
/*4*/    F6,   F7,   F8,   F9,  F10, 0x00, 0x00,  '7',  '8',  '9',  '-',  '4',  '5',  '6',  '+',  '1',
/*5*/   '2',  '3',  '0',  DEL, 0x00, 0x00, 0x00,  F11,  F12, 0x00,
};

extern "C" {
    #include "keyboard.h"

    bool __time_critical_func(handleScancode)(const uint32_t ps2scancode) {
        uint8_t key = ((uint8_t)ps2scancode) & 0xFF;
        switch (key)
        {
        case 0x3A:
            break;
        case 0xBA:
            capsLocked = capsLocked ? false : true;
            keyboard_toggle_led(1);
            break;
        case 0x2A:
            shiftsStates |= 1;
            break;
        case 0xAA:
            shiftsStates &= ~1;
            break;
        case 0x36:
            shiftsStates |= 2;
            break;
        case 0xB6:
            shiftsStates &= ~2;
            break;
        default:
            if (key < 0x80)
                keyboard_key_code_set(
                    capsLocked ? 
                    (!shiftsStates ? key_iee_shift[key] : key_iee[key]) :
                    (shiftsStates ? key_iee_shift[key] : key_iee[key])
                );
            break;
        }
        return true;
    }
}

void nespad_update() {
}

void __time_critical_func(render_core)() {
    const auto buffer = (uint8_t*)&SCREEN[0][0];
    graphics_set_buffer(buffer, DISP_WIDTH, DISP_HEIGHT); // TODO: APPLE_640x480
    multicore_lockout_victim_init();
    graphics_init();
    graphics_set_textbuffer(buffer);
    graphics_set_bgcolor(0x000000);
    graphics_set_offset(0, 0);
    graphics_set_flashmode(false, false);
    sem_acquire_blocking(&vga_start_semaphore);
    // 60 FPS loop
#define frame_tick (16666)
    uint64_t tick = time_us_64();
#ifdef TFT
    uint64_t last_renderer_tick = tick;
#endif
    uint64_t last_input_tick = tick;
    while (true) {
#ifdef TFT
        if (tick >= last_renderer_tick + frame_tick) {
            refresh_lcd();
            last_renderer_tick = tick;
        }
#endif
        // Every 5th frame
        if (tick >= last_input_tick + frame_tick * 5) {
            nespad_read();
            last_input_tick = tick;
            nespad_update();
        }
        tick = time_us_64();
        tight_loop_contents();
    }
    __unreachable();
}

#ifdef SOUND
static repeating_timer_t timer;
static int snd_channels = 2;
static bool __not_in_flash_func(snd_timer_callback)(repeating_timer_t *rt) {
    static uint16_t outL = 0;  
    static uint16_t outR = 0;
    register size_t idx = sound_array_idx;
    if (idx >= sound_array_fill) {
        return true;
    }
    pwm_set_gpio_level(PWM_PIN0, outR); // Право
    pwm_set_gpio_level(PWM_PIN1, outL); // Лево
    outL = outR = 0;
    if (!Sound_enabled || paused) {
        return true;
    }
    register UBYTE* uba = LIBATARI800_Sound_array;
    if (snd_channels == 2) {
        outL = uba[idx++]; idx++;
        outR = uba[idx++]; idx++;
    } else {
        outL = outR = uba[idx++]; idx++;
    }
    sound_array_idx = idx;
    ///pwm_set_gpio_level(BEEPER_PIN, 0);
    return true;
}
#endif

#define SPEAKER_ADDR 0xC030
#define SPEAKER_ADDR_ALT_MASK 0xFFF0
static uint8_t pin_state = 0;
static void speaker_update(uint8_t read, uint16_t address, uint8_t *byte) {
    // read or write toggles speaker driver
    if ((address & SPEAKER_ADDR_ALT_MASK) == SPEAKER_ADDR) {
        pin_state ^= 0xFF;
        pwm_set_gpio_level(BEEPER_PIN, pin_state);
    }
}

#include "f_util.h"
static FATFS fatfs;
bool SD_CARD_AVAILABLE = false;
static void init_fs() {
    FRESULT result = f_mount(&fatfs, "", 1);
    if (FR_OK != result) {
        printf("Unable to mount SD-card: %s (%d)", FRESULT_str(result), result);
    } else {
        SD_CARD_AVAILABLE = true;
    }
}

inline static void init_wii() {
    if (Init_Wii_Joystick()) {
        Wii_decode_joy();
        printf("Found WII joystick");
    }
}

static bool reset = false;
static uint8_t running = 1;
extern "C" {
    #include "c6502.h"
    #include "rom.h"
    #include "ram.h"
    #include "video.h"
    static C6502_interface interface_c;
}

int main() {
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);
    set_sys_clock_khz(378 * KHZ, true);
    stdio_init_all();
    keyboard_init();
    keyboard_send(0xFF);
    nespad_begin(clock_get_hz(clk_sys) / 1000, NES_GPIO_CLK, NES_GPIO_DATA, NES_GPIO_LAT);

    nespad_read();
    sleep_ms(50);

    // Joistik START button to USB FIRMWARE UPDATE mode
    if (nespad_state & DPAD_START) {
        printf("reset_usb_boot");
        reset_usb_boot(0, 0);
    }

    init_fs();
    #ifdef PSRAM
    init_psram();
    #endif

    sem_init(&vga_start_semaphore, 0, 1);
    multicore_launch_core1(render_core);
    sem_release(&vga_start_semaphore);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 6; i++) {
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(33);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
    }

    PWM_init_pin(BEEPER_PIN, (1 << 8) - 1);
#ifdef SOUND
    PWM_init_pin(PWM_PIN0, (1 << 8) - 1);
    PWM_init_pin(PWM_PIN1, (1 << 8) - 1);
#endif
#if LOAD_WAV_PIO
    //пин ввода звука
    inInit(LOAD_WAV_PIO);
#endif
    rom_init();
    ram_init();
    video_init();
    c6502_init();
//    menu_init();
    while(true) {
        if (reset == true) {
            reset = false;
            c6502_reset(&interface_c);
        }
        if (running)
        {
            c6502_update(&interface_c);
            ram_update(interface_c.rw, interface_c.address, &interface_c.data);
            rom_update(interface_c.rw, interface_c.address, &interface_c.data);
            keyboard_update(interface_c.rw, interface_c.address, &interface_c.data);
        //    game_update(interface_c.rw, interface_c.address, &interface_c.data);
            speaker_update(interface_c.rw, interface_c.address, &interface_c.data);
            video_update(interface_c.rw, interface_c.address, &interface_c.data);
        }
        tight_loop_contents();
    }

    __unreachable();
}
