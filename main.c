/*
 * Kocurovic studna
 *
 * Part of esp-open-rtos
 * Copyright (C) 2016 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */

#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp/gpio.h>

// Akivace DEBUG vypisu na UART
// #define DEBUG

// --- LED --- //
// Aktivace LED signalizace
#define LED_GPIO 2

// --- Ultrazvukovy senzor ---
#include <ultrasonic/ultrasonic.h>
#include <math.h>

// Pocet vzorku, ze kterych se bere minimalni hodnota
#define SAMPLE_ARRAY_SIZE 10

//Interval skenovani vodni hladiny
#define SURFACE_SCAN_INTERVAL_MS 200

#define TRIGGER_PIN 14
#define ECHO_PIN    12

#define MAX_DISTANCE_CM 500 // 5m max

#define WELL_RADIUS 50 // cm 
#define WELL_HEIGHT 1344 // cm
#define SENSOR_HEIGHT 900 // cm

// --- LCD displej ---
#include <i2c/i2c.h>
#include <hd44780/hd44780.h>

#define I2C_BUS 0
#define SCL_PIN 5
#define SDA_PIN 4
#define ADDR 0x27

static const uint8_t char_data[] = {
    0x04, 0x0e, 0x0e, 0x0e, 0x1f, 0x00, 0x04, 0x00,
    0x1f, 0x11, 0x0a, 0x04, 0x0a, 0x11, 0x1f, 0x00};

ultrasonic_sensor_t sensor = {
    .trigger_pin = TRIGGER_PIN,
    .echo_pin = ECHO_PIN};

hd44780_t lcd = {
    .i2c_dev.bus = I2C_BUS,
    .i2c_dev.addr = ADDR,
    .font = HD44780_FONT_5X8,
    .lines = 4,
    .pins = {
        .rs = 0,
        .e = 2,
        .d4 = 4,
        .d5 = 5,
        .d6 = 6,
        .d7 = 7,
        .bl = 3},
    .backlight = true};


void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i ++)
        sdk_os_delay_us(1000);
}


// Volume calculation in cm
double well_volume(uint32_t height) {

    double volume;

    volume = M_PI * (WELL_RADIUS * WELL_RADIUS) * height;

    // Export volume in cm3 
    return volume;
}

// Nalezeni nejmenisho prvku v poli
uint32_t get_min(uint32_t numbers[])
{
    uint32_t minimum = numbers[0];

    for (uint8_t c = 1; c < SAMPLE_ARRAY_SIZE; c++)
    {
        if (numbers[c] < minimum)
        {
            minimum = numbers[c];
        }
    }
#ifdef DEBUG
    printf("Minimum distance: %d cm, %.02f m \n", minimum, minimum / 100.0);
#endif

    return minimum;
}

// Nalezeni nejvetsiho prvku v poli
uint32_t get_max(uint32_t numbers[])
{
    uint32_t maximum = numbers[0];

    for (uint8_t c = 1; c < SAMPLE_ARRAY_SIZE; c++)
    {
        if (numbers[c] > maximum)
        {
            maximum = numbers[c];
        }
    }
#ifdef DEBUG
    printf("Maximum distance: %d cm, %.02f m \n", maximum, maximum / 100.0);
#endif

    return maximum;
}

// uint32_t well_water_distance(const ultrasonic_sensor_t *device)
int32_t well_water_distance()
{
    static int32_t distance = 0;

    distance = ultrasoinc_measure_cm(&sensor, MAX_DISTANCE_CM);

    if (distance < 0)
    {
#ifdef DEBUG
        printf("Error: ");
#endif
        switch (distance)
        {
        case ULTRASONIC_ERROR_PING:
#ifdef DEBUG
            printf("Cannot ping (device is in invalid state)\n");
#endif
            break;
        case ULTRASONIC_ERROR_PING_TIMEOUT:
#ifdef DEBUG
            printf("Ping timeout (no device found)\n");
#endif
            break;
        case ULTRASONIC_ERROR_ECHO_TIMEOUT:
#ifdef DEBUG
            printf("Echo timeout (i.e. distance too big)\n");
#endif
            break;
        }
    }
    else if (distance > MAX_DISTANCE_CM)
    {
#ifdef DEBUG
        printf("Distance to high (more than %u)\n", MAX_DISTANCE_CM);
#endif
        distance = -1;
    } else

    {
#ifdef DEBUG
        printf("Distance sample: %d cm, %.02f m \n", distance, distance / 100.0);
#endif
    }
    
    return distance;
}

uint32_t *well_status()
{
    static uint32_t distance_array[SAMPLE_ARRAY_SIZE];
    static int32_t distance = 0;
    static uint32_t surface_distance = 0;
    static uint32_t water_column = 0;
    static double volume = 0;
    static uint32_t well_water_status[3];

    uint8_t index = 0;

// Sber deseti vzorku
    while (index < SAMPLE_ARRAY_SIZE)
    {
        distance = well_water_distance();
        if ( distance > 0 )
            {
                distance_array[index] = distance;
                index++;
            }
            delay_ms(SURFACE_SCAN_INTERVAL_MS);
    }

    // Columnt of water in cm
    surface_distance = get_max(distance_array);
    water_column = WELL_HEIGHT - SENSOR_HEIGHT - surface_distance;

    // Volume of water in cm3
    volume = well_volume(water_column);

    //printf("Sloupec vody %u cm, Objem vody %.0f l / %.03f m3 (%u s)\n", water_column, volume / 1e3, volume / 1e6, sdk_system_get_time() / 1000000);

    well_water_status[0] = surface_distance;
    well_water_status[1] = water_column;
    well_water_status[2] = (uint32_t)volume / 1e3;

    return well_water_status;
}

void system_banner()
{
    printf("SDK version: %s\n", sdk_system_get_sdk_version());
    printf("Well radius: %u cm\n", WELL_RADIUS);
    printf("Well height: %u cm\n", WELL_HEIGHT);
    printf("Sensor height: %u cm\n", SENSOR_HEIGHT);
    printf("Surface scan interval: %d ms\n", SURFACE_SCAN_INTERVAL_MS);
    printf("Number of samples: %d\n", SAMPLE_ARRAY_SIZE);
}

void lcd_system_banner()
{
    char text[21];
    snprintf(text, 21, "SDK version: %s", sdk_system_get_sdk_version());
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, text);

    snprintf(text, 21, "Well radius: %.02f m", (float)WELL_RADIUS/100);
    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, text);

    snprintf(text, 21, "Well height: %.02f m", (float)WELL_HEIGHT/100);
    hd44780_gotoxy(&lcd, 0, 2);
    hd44780_puts(&lcd, text);

    snprintf(text, 21, "Sensor height: %.01f m", (float)SENSOR_HEIGHT / 100);
    hd44780_gotoxy(&lcd, 0, 3);
    hd44780_puts(&lcd, text);
}

void lcd_information_banner()
{
    hd44780_gotoxy(&lcd, 0, 0);
    hd44780_puts(&lcd, "Sloupec vody:     cm");

    hd44780_gotoxy(&lcd, 0, 1);
    hd44780_puts(&lcd, "Objem vody:      l");

    hd44780_gotoxy(&lcd, 0, 2);
    hd44780_puts(&lcd, "Objem vody:      m3");
}

void lcd_show_information(uint32_t data[]) {

    char text[21];

    snprintf(text, 21, "%u", data[1]);
    // text[sizeof(text) - 1] = 0;
    hd44780_gotoxy(&lcd, 14, 0);
    hd44780_puts(&lcd, text);

    snprintf(text, 21, "%u", data[2]);
    // text[sizeof(text) - 1] = 0;
    hd44780_gotoxy(&lcd, 12, 1);
    hd44780_puts(&lcd, text);

    snprintf(text, 21, "%.02f", data[2]/1e3);
    // text[sizeof(text) - 1] = 0;
    hd44780_gotoxy(&lcd, 12, 2);
    hd44780_puts(&lcd, text);
}

void user_init() {
    uart_set_baud(0, 115200);

    uint32_t *surface_info;

    // Vypis zakladnich informaci na terminal
    system_banner();

// Inicializace ultrazvuku
    ultrasoinc_init(&sensor);

// Inicializce I2C
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);

    hd44780_init(&lcd);
    hd44780_upload_character(&lcd, 0, char_data);
    hd44780_upload_character(&lcd, 1, char_data + 8);

// Zobrazeni systemovych informaci na zacatku
    lcd_system_banner();
    delay_ms(3000);
    hd44780_clear(&lcd);
// Zobrazeni statickych informaci k zobrazovanym hodnotam
    lcd_information_banner();

#ifdef LED_GPIO
    // Inicializace LED GPIO
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
#endif

    while (true)
    {
        // Nacteni dat o vysce hladiny
#ifdef LED_GPIO
        gpio_write(LED_GPIO, 1);
#endif
        surface_info = well_status();
#ifdef LED_GPIO
        gpio_write(LED_GPIO, 0);
#endif
        printf("Vzdalenost senzoru %u cm, Sloupec vody %u cm, Objem vody %u l / %.03f m3 (%u s)\n", surface_info[0], surface_info[1], surface_info[2], (double)surface_info[2] / 1e3, sdk_system_get_time() / 1000000);

        // Zobrazeni aktualnich dat na displeji
        lcd_show_information(surface_info);
    }
        
}
