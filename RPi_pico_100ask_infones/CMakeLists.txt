cmake_minimum_required(VERSION 3.16.0)

project(RPi_pico_100ask_infones C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(RPi_pico_100ask_infones)

pico_generate_pio_header(RPi_pico_100ask_infones ${CMAKE_CURRENT_LIST_DIR}/ili9341_lcd.pio)

target_sources(RPi_pico_100ask_infones PRIVATE RPi_pico_100ask_infones.c
                infones/InfoNES.c  
                infones/InfoNES_Mapper.c  
                infones/InfoNES_pAPU.c  
                infones/InfoNES_System.c  
                infones/K6502.c  
                 )
				 
add_subdirectory(./lib/FatFs_SPI_lib build)


add_definitions(
                # System Settings
                -D CONFIG_HW_SYS_CLOCK_KHZ=200000

                # ILI9341 LCD Settings
                -D CONFIG_SERIAL_CLK_DIV=1.5f      # 125.f 1.f 1.4f 1.5f 2.f 6.25f
                -D CONFIG_ILI9341_TFTWIDTH=320
                -D CONFIG_ILI9341_TFTHEIGHT=240

                # NES Display Settings
                -D CONFIG_NES_DISP_WIDTH=256
                -D CONFIG_NES_DISP_HEIGHT=240

                # Pin mappings for ILI9341 LCD
                -D CONFIG_ILI9341_LCD_PIN_DIN=7
                -D CONFIG_ILI9341_LCD_PIN_CLK=6
                -D CONFIG_ILI9341_LCD_PIN_CS=13
                -D CONFIG_ILI9341_LCD_PIN_DC=15
                -D CONFIG_ILI9341_LCD_PIN_RESET=14
                -D CONFIG_ILI9341_LCD_PIN_BL=1

                # Pin mappings for SD card
                -D CONFIG_SD_SPI_PORT=spi0
                -D CONFIG_SD_SPI_CS_PIN=17
                -D CONFIG_SD_SPI_SCK_PIN=18
                -D CONFIG_SD_SPI_MOSI_PIN=19
                -D CONFIG_SD_SPI_MISO_PIN=16

                # Pin mappings for NES controller input (兼容国内7针、9针、MicroUSB手柄)
                -D CONFIG_NES_CONTROLLER=1
                -D CONFIG_NES_CONTROLLER_CLOCK_PIN=2    # CLOCK PIN
                -D CONFIG_NES_CONTROLLER_LATCH_PIN=3    # LATCH PIN
                -D CONFIG_NES_CONTROLLER_DATA_PIN=4     # DATA PIN

                # Pin mappings for PCF8574(IIC) input
                -D CONFIG_PCF8574_I2C=0
                -D CONFIG_PCF8574_I2C_ADDR=0x20
                -D CONFIG_PCF8574_I2C_PORT=i2c0
                -D CONFIG_PCF8574_I2C_SDA_PIN=20 # 4
                -D CONFIG_PCF8574_I2C_SCL_PIN=21 # 5
                )

pico_set_program_name(RPi_pico_100ask_infones "100ASK RPi_pico_100ask_infones")
pico_set_program_version(RPi_pico_100ask_infones "0.1")

pico_enable_stdio_usb(RPi_pico_100ask_infones 0)
pico_enable_stdio_uart(RPi_pico_100ask_infones 1)


# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(RPi_pico_100ask_infones pico_stdlib hardware_pwm hardware_i2c pico_multicore hardware_pio hardware_interp hardware_spi hardware_clocks hardware_adc FatFs_SPI_lib)

# create map/bin/hex file etc.
pico_add_extra_outputs(RPi_pico_100ask_infones)

# add url via pico_set_program_url
example_auto_set_url(RPi_pico_100ask_infones)

