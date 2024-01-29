/**
 * Copyright (c) 2021 韦东山
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "f_util.h"
#include "ff.h"
#include "hw_config.h"
#include "rtc.h"
#include "sd_card.h"

#include "ff_headers.h"
#include "ff_stdio.h"

#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"

#include "pico/multicore.h"

#include "ili9341_lcd.pio.h"

#undef assert
#define assert configASSERT
#define fopen ff_fopen
#define fwrite ff_fwrite
#define fread ff_fread
#define fclose ff_fclose
#ifndef FF_DEFINED
#define errno stdioGET_ERRNO()
#define free vPortFree
#define malloc pvPortMalloc
#endif


// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little
/* reference: https://github.com/adafruit/Adafruit_ILI9341.git */
static const uint8_t ili9341_init_seq[] = {
        4, 1, 0xEF, 0x03, 0x80, 0x02,
        4, 1, 0xCF, 0x00, 0xC1, 0x30,
        5, 1, 0xED, 0x64, 0x03, 0x12, 0x81,
        4, 1, 0xE8, 0x85, 0x00, 0x78,
        6, 1, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
        2, 1, 0xF7, 0x20,
        3, 1, 0xEA, 0x00, 0x00,
        2, 1, 0xC0, 0x23,
        2, 1, 0xC1, 0x10,
        3, 1, 0xC5, 0x3E, 0x28,
        3, 1, 0xC5, 0x3E, 0x28,
        2, 1, 0xC7, 0x86,
        2, 1, 0x36, 0xe0, /* rotate-0: 0x48; rotate-1: 0x28; rotate-2: 0x88; rotate-3: 0xe8 */
        2, 1, 0x3A, 0x55,
        3, 1, 0xB1, 0x00, 0x18,
        4, 1, 0xB6, 0x08, 0x82, 0x27,
        2, 1, 0xF2, 0x00,
        2, 1, 0x26, 0x01,
       16, 1, 0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
       16, 1, 0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
        1, 200, 0x11,
        1, 1, 0x29,
        0
};

static inline void lcd_set_dc_cs(bool dc, bool cs) {
    sleep_us(1);
    gpio_put_masked((1u << CONFIG_ILI9341_LCD_PIN_DC) | (1u << CONFIG_ILI9341_LCD_PIN_CS), !!dc << CONFIG_ILI9341_LCD_PIN_DC | !!cs << CONFIG_ILI9341_LCD_PIN_CS);
    sleep_us(1);
}

static inline void lcd_write_cmd(PIO pio, uint sm, const uint8_t *cmd, size_t count) {
    ili9341_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(0, 0);
    ili9341_lcd_put(pio, sm, *cmd++);
    if (count >= 2) {
        ili9341_lcd_wait_idle(pio, sm);
        lcd_set_dc_cs(1, 0);
        for (size_t i = 0; i < count - 1; ++i)
            ili9341_lcd_put(pio, sm, *cmd++);
    }
    ili9341_lcd_wait_idle(pio, sm);
    lcd_set_dc_cs(1, 1);
}

static inline void lcd_init(PIO pio, uint sm, const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd(pio, sm, cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static inline void ili9341_start_pixels(PIO pio, uint sm) {
    uint8_t cmd = 0x2c; // RAMWR
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);
}


extern unsigned short *WorkFrame;
/* fb from nes, 256 x 240 */



/* RGB565 MSB-RED-GREEN-BLUE-LSB */
// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F


void tft_set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

    PIO pio = pio0;
    uint sm = 0;
    uint8_t cmd;

    cmd = 0x2a;
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);


    ili9341_lcd_put(pio, sm, x0 >> 8);
    ili9341_lcd_put(pio, sm, x0 & 0xff);
    ili9341_lcd_put(pio, sm, x1 >> 8);
    ili9341_lcd_put(pio, sm, x1 & 0xff);

    cmd = 0x2b;
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);

    ili9341_lcd_put(pio, sm, y0 >> 8);
    ili9341_lcd_put(pio, sm, y0 & 0xff);
    ili9341_lcd_put(pio, sm, y1 >> 8);
    ili9341_lcd_put(pio, sm, y1 & 0xff);

    cmd = 0x2c;
    lcd_write_cmd(pio, sm, &cmd, 1);
    lcd_set_dc_cs(1, 0);

}

void tft_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{

    PIO pio = pio0;
    uint sm = 0;
    // rudimentary clipping (drawChar w/big text requires this)
    if((x >= CONFIG_ILI9341_TFTWIDTH) || (y >= CONFIG_ILI9341_TFTHEIGHT)) return;

    if((x + w - 1) >= CONFIG_ILI9341_TFTWIDTH)  w = CONFIG_ILI9341_TFTWIDTH  - x;
    if((y + h - 1) >= CONFIG_ILI9341_TFTHEIGHT) h = CONFIG_ILI9341_TFTHEIGHT - y;


    tft_set_addr_window(x, y, x+w-1, y+h-1);

    uint8_t hi = color >> 8, lo = color;

    for(y=h; y>0; y--) {
        for(x=w; x>0; x--) {
            ili9341_lcd_put(pio, sm, hi);
            ili9341_lcd_put(pio, sm, lo);
        }
    }

}

void __tft_flush()
{
    uint16_t x, y;
    uint16_t i = 0;
    uint8_t hi, lo;
    uint16_t black_color = ILI9341_BLACK;

    uint16_t *fb = (unsigned short *)&WorkFrame;
    if (fb == NULL) {
        return;
    }

    //uint16_t *fb = framebuffer;

    //memcpy((uint8_t *)framebuffer, (uint8_t *)&WorkFrame, sizeof(framebuffer));

    PIO pio = pio0;
    uint sm = 0;

    //tft_set_addr_window(0, 0, CONFIG_NES_DISP_WIDTH-1, CONFIG_NES_DISP_HEIGHT-1);

    for(y = 0; y < CONFIG_NES_DISP_HEIGHT; y++) {

        for(x = 0; x < CONFIG_NES_DISP_WIDTH; x++) {
            hi = fb[i] >> 8, lo = fb[i];
            ili9341_lcd_put(pio, sm, hi);
            ili9341_lcd_put(pio, sm, lo);
            i++;
        }

    }
}


void tft_flush()
{
#if 1
    static uint32_t frame_index = 0;
    multicore_fifo_push_blocking(frame_index++);
#else
    __tft_flush();
#endif
}

int tft_fill_screen(uint16_t color)
{
    tft_fill_rect(0, 0, CONFIG_ILI9341_TFTWIDTH, CONFIG_ILI9341_TFTWIDTH, color);
}

/*********** TEST END  *******/

extern int InfoNES_Load( const char *pszFileName );
extern void InfoNES_Main();


//static int addr = 0x20;
//#define I2C_PORT            i2c0
//#define PCD8574_SDA_PIN     (20)//(4)
//#define PCD8574_SCL_PIN     (21)//(5)

// write pin for pcf8574
static void write_iic(void)
{
    uint8_t iic_date = 0xff;
    i2c_write_blocking(CONFIG_PCF8574_I2C_PORT, CONFIG_PCF8574_I2C_ADDR, &iic_date, 1, true); // true to keep master control of bus
    sleep_ms(1000);
    iic_date = 0x00;
    i2c_write_blocking(CONFIG_PCF8574_I2C_PORT, CONFIG_PCF8574_I2C_ADDR, &iic_date, 1, true); // true to keep master control of bus
    sleep_ms(1000);
}

// read pin for pcf8574
static uint8_t infones_100ask_read_iic(void)
{
    uint8_t buffer;
    i2c_read_blocking(CONFIG_PCF8574_I2C_PORT, CONFIG_PCF8574_I2C_ADDR, &buffer, 1, true); // true to keep master control of bus
    //printf("buffer: %d\n", buffer);
    //sleep_ms(200);
    return buffer;
}

static uint8_t infones_100ask_nes_ctl_read(void)
{
    volatile uint8_t temp=0;
    gpio_put(CONFIG_NES_CONTROLLER_LATCH_PIN, 1);
    sleep_us(1);
    gpio_put(CONFIG_NES_CONTROLLER_LATCH_PIN, 0);

    for(int i = 0; i < 8; i++)
    {
        //temp >>= 1;
        if(gpio_get(CONFIG_NES_CONTROLLER_DATA_PIN) == 0)
            temp |= (1 << i);
        gpio_put(CONFIG_NES_CONTROLLER_CLOCK_PIN, 1);
        sleep_us(1);
        gpio_put(CONFIG_NES_CONTROLLER_CLOCK_PIN, 0);
        sleep_us(1);
    }

    return temp;
}


#if 0
#if CONFIG_NES_CONTROLLER
#define BUTTON_INFO(count) (1 << count)
// A、B、Start、Select、上、下、左、右
//DWORD button_info[] = {(1 << 0), (1 << 1), (1 << 2), (1 << 3), (1 << 4), (1 << 5), (1 << 6), (1 << 7)};
#elif CONFIG_PCF8574_I2C
// 上、下、左、右、A、B、Start、Select
DWORD button_info[] = {(1 << 4), (1 << 5), (1 << 6), (1 << 7), (1 << 1), (1 << 0), (1 << 3), (1 << 2)};
#endif
#endif

#define BUTTON_INFO(count) (1 << count)
/*
    A: 		0000 0001 1     (1 << 0)
    B: 		0000 0010 2     (1 << 1)
    start: 	0000 0100 8     (1 << 2)
    select:	0000 1000 4     (1 << 3)
    Up:		0001 0000 16    (1 << 4)
    Down:   0010 0000 32    (1 << 5)
    Left:	0100 0000 64    (1 << 6)
    Right:  1000 0000 128   (1 << 7)
*/
// TODO (1 << 0) No effect
int infones_100ask_get_pdwPad_state(void)
{
    /* Initialize a pad state */
    DWORD dwKeyPad = 0;
    
#if CONFIG_NES_CONTROLLER
    uint8_t nes_ctl_data = infones_100ask_nes_ctl_read();
    //printf("nes_ctl_data: %d\n", nes_ctl_data); // 默认 0，按下对应位置为1
    for (int i = 0; i < 8; i++)
    {
        if(((((uint8_t)1<<i) & nes_ctl_data) >= 1))
            dwKeyPad |= BUTTON_INFO(i);
    }

#elif CONFIG_PCF8574_I2C
    uint8_t i2c_data = infones_100ask_read_iic();
    //printf("i2c_data: %d\n", i2c_data);  // 默认 255，按下对应位置为0

    for (int i = 0; i < 8; i++)
    {
        if(((((uint8_t)1<<i) & i2c_data) == 0))
            dwKeyPad |= button_info[i];
    }
#endif
    //printf("dwKeyPad: %d\n", dwKeyPad);
    return dwKeyPad;
}



void core1_entry() 
{
    uint32_t m = 0;
    printf("core1 start0\n");

    while (1) {
        m = multicore_fifo_pop_blocking();
        //printf("c1-m [%d]\r\n", m);
        __tft_flush();
    }

}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);

    // Can't measure clk_ref / xosc as it is the ref
}



/*FatFS_SPI_example

   GPIO 16 (pin 21)  MISO/spi0_rx
   GPIO 17 (pin 22)  Chip select
   GPIO 24 (pin 18)  SCK/spi0_sclk
   GPIO 25 (pin 19) MOSI/spi0_tx
   3.3v   (pin 36)
   GND    (pin 38)

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
   The particular device used here uses the same pins for I2C and SPI, hence the
   using of I2C names
*/

typedef struct {
    FATFS fatfs;
    char const *const name;
} fatfs_dscr_t;
static fatfs_dscr_t fatfs_dscrs[2] = {{.name = "0:"}, {.name = "1:"}};
static FATFS *get_fs_by_name(const char *name) {
    for (size_t i = 0; i < count_of(fatfs_dscrs); ++i) {
        if (0 == strcmp(fatfs_dscrs[i].name, name)) {
            return &fatfs_dscrs[i].fatfs;
        }
    }
    return NULL;
}

static void run_mount(char *drive) {
    FATFS *p_fs = get_fs_by_name(drive);
    if (!p_fs) {
        printf("Unknown logical drive number: \"%s\"\n", drive);
        return;
    }
    FRESULT fr = f_mount(p_fs, drive, 1);
    if (FR_OK != fr) printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
}

static void infones_100ask_input_init(void)
{
#if CONFIG_NES_CONTROLLER
    gpio_init(CONFIG_NES_CONTROLLER_CLOCK_PIN);
    gpio_init(CONFIG_NES_CONTROLLER_LATCH_PIN);
    gpio_init(CONFIG_NES_CONTROLLER_DATA_PIN);

    gpio_set_dir(CONFIG_NES_CONTROLLER_CLOCK_PIN, GPIO_OUT);
    gpio_set_dir(CONFIG_NES_CONTROLLER_LATCH_PIN, GPIO_OUT);
    gpio_set_dir(CONFIG_NES_CONTROLLER_DATA_PIN, GPIO_IN);
#elif CONFIG_PCF8574_I2C
    // This example will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL) running at 400kHz.
    //i2c_init(CONFIG_PCF8574_I2C_PORT, 400 * 1000);
    i2c_init(CONFIG_PCF8574_I2C_PORT, 200 * 1000);
    //i2c_set_slave_mode(CONFIG_PCF8574_I2C_PORT, true, CONFIG_PCF8574_I2C_ADDR);
    gpio_set_function(CONFIG_PCF8574_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(CONFIG_PCF8574_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(CONFIG_PCF8574_I2C_SDA_PIN);
    gpio_pull_up(CONFIG_PCF8574_I2C_SCL_PIN);
#endif

}


// Audio PIN is to match some of the design guide shields. 
#define AUDIO_PIN 28  // you can change this to whatever you like

int main() {
    stdio_init_all();

    set_sys_clock_khz(CONFIG_HW_SYS_CLOCK_KHZ, true);    /* work with spi clkdiv 1.5f */

#if 0
    gpio_set_function(AUDIO_PIN, GPIO_FUNC_PWM);
    int audio_pin_slice = pwm_gpio_to_slice_num(AUDIO_PIN);
    // Setup PWM for audio output
    pwm_config config = pwm_get_default_config();
    /* Base clock 176,000,000 Hz divide by wrap 250 then the clock divider further divides
     * to set the interrupt rate. 
     * 
     * 11 KHz is fine for speech. Phone lines generally sample at 8 KHz
     * 
     * 
     * So clkdiv should be as follows for given sample rate
     *  8.0f for 11 KHz
     *  4.0f for 22 KHz
     *  2.0f for 44 KHz etc
     */
    pwm_config_set_clkdiv(&config, 8.0f); 
    pwm_config_set_wrap(&config, 250); 
    pwm_init(audio_pin_slice, &config, true);

    pwm_set_gpio_level(AUDIO_PIN, 0);
#endif

    infones_100ask_input_init();

    sd_init_driver();
    //printf("\033[2J\033[H");  // Clear Screen
    run_mount("0:");


    printf("infones start\n");
   
    measure_freqs();

    multicore_launch_core1(core1_entry);

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ili9341_lcd_program);
    ili9341_lcd_program_init(pio, sm, offset, CONFIG_ILI9341_LCD_PIN_DIN, CONFIG_ILI9341_LCD_PIN_CLK, CONFIG_SERIAL_CLK_DIV);

    gpio_init(CONFIG_ILI9341_LCD_PIN_CS);
    gpio_init(CONFIG_ILI9341_LCD_PIN_DC);
    gpio_init(CONFIG_ILI9341_LCD_PIN_RESET);
    gpio_init(CONFIG_ILI9341_LCD_PIN_BL);
    gpio_set_dir(CONFIG_ILI9341_LCD_PIN_CS, GPIO_OUT);
    gpio_set_dir(CONFIG_ILI9341_LCD_PIN_DC, GPIO_OUT);
    gpio_set_dir(CONFIG_ILI9341_LCD_PIN_RESET, GPIO_OUT);
    gpio_set_dir(CONFIG_ILI9341_LCD_PIN_BL, GPIO_OUT);

    gpio_put(CONFIG_ILI9341_LCD_PIN_CS, 1);
    gpio_put(CONFIG_ILI9341_LCD_PIN_RESET, 1);
    lcd_init(pio, sm, ili9341_init_seq);
    gpio_put(CONFIG_ILI9341_LCD_PIN_BL, 1);

    tft_set_addr_window(0, 0, CONFIG_ILI9341_TFTWIDTH - 1, CONFIG_ILI9341_TFTHEIGHT - 1);
    tft_fill_screen(ILI9341_BLACK);

    tft_set_addr_window(0, 0, CONFIG_NES_DISP_WIDTH-1, CONFIG_NES_DISP_HEIGHT-1);


    while (1) {
        if(InfoNES_Load(NULL) == 0) {
            InfoNES_Main();
        }
    }

    while (1) {
        printf("error!!!\n");
        sleep_ms(1000);
    }

    return 0;
}
