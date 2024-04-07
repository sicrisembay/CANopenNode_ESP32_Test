#include <stdio.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_dma_utils.h"
#include "esp_timer.h"
#include "CANopenNode_ESP32.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"
#include "lvgl.h"
#include "ui.h"

static const char *TAG = "main";

/* LCD Interface (i80) */
#define LCD_PIXEL_CLOCK_HZ 10000000
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_NUM_PCLK 17
#define LCD_PIN_NUM_CS 15
#define LCD_PIN_NUM_DC 4
#define LCD_PIN_NUM_RST 8
#define LCD_PIN_NUM_BK_LIGHT 36
#define LCD_PIN_NUM_RD 16
#define LCD_I80_BUS_WIDTH 8
#define LCD_PIN_NUM_DATA0 18
#define LCD_PIN_NUM_DATA1 20
#define LCD_PIN_NUM_DATA2 19
#define LCD_PIN_NUM_DATA3 11
#define LCD_PIN_NUM_DATA4 10
#define LCD_PIN_NUM_DATA5 9
#define LCD_PIN_NUM_DATA6 38
#define LCD_PIN_NUM_DATA7 3
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8
/* The pixel number in horizontal and vertical */
#define LCD_H_RES 320
#define LCD_V_RES 480
#define LCD_BUFFER_HRES_MULTIPLE (100)
#define LCD_BUFFER_PX_CNT (LCD_H_RES * LCD_BUFFER_HRES_MULTIPLE)

/* Touch Interface (i2c) */
#define TP_I2C_NUM 0
#define TP_PIN_NUM_SCL 41
#define TP_PIN_NUM_SDA 37
#define TP_PIN_NUM_INT 40
#define TP_PIN_NUM_RST 42

#define LVGL_TICK_PERIOD_MS 2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (8192)
#define LVGL_TASK_PRIORITY 2

static SemaphoreHandle_t lvgl_mux = NULL;

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static bool lvgl_lock(int timeout_ms)
{
    /*
     * Convert timeout in milliseconds to FreeRTOS ticks
     * If `timeout_ms` is set to -1, the program will block until the condition is met
     */
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void lvgl_port_task(void *arg)
{
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    if (lvgl_lock(-1))
    {
        ui_init();
        lvgl_unlock();
    }

    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            /* Release the mutex */
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void init_i80_16bit_bus(esp_lcd_panel_io_handle_t *io_handle, void *user_ctx)
{
    ESP_LOGI(TAG, "Initialize i80 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .wr_gpio_num = LCD_PIN_NUM_PCLK,
        .data_gpio_nums = {
            LCD_PIN_NUM_DATA0,
            LCD_PIN_NUM_DATA1,
            LCD_PIN_NUM_DATA2,
            LCD_PIN_NUM_DATA3,
            LCD_PIN_NUM_DATA4,
            LCD_PIN_NUM_DATA5,
            LCD_PIN_NUM_DATA6,
            LCD_PIN_NUM_DATA7,
        },
        .bus_width = LCD_I80_BUS_WIDTH,
        .max_transfer_bytes = LCD_BUFFER_PX_CNT * sizeof(uint16_t),
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 1,
        },
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = user_ctx,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, io_handle));
}

static void init_lcd_panel(esp_lcd_panel_io_handle_t io_handle, esp_lcd_panel_handle_t *panel)
{
    esp_lcd_panel_handle_t panel_handle = NULL;

    /*
     * ILI9488 is NOT a distinct driver, but a special case of ST7789
     * (essential registers are identical). A few lines further down in this code,
     * it's shown how to issue additional device-specific commands.
     */
    ESP_LOGI(TAG, "Install LCD driver of ili9488 (st7789 compatible)");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    /*
     * ILI9488 is very similar to ST7789 and shares the same driver.
     * Anything unconventional (such as this custom gamma table) can
     * be issued here in user code and need not modify the driver.
     */
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, false);
    esp_lcd_panel_set_gap(panel_handle, 0, 0);
    esp_lcd_panel_swap_xy(panel_handle, false);
    esp_lcd_panel_mirror(panel_handle, true, false);

    *panel = panel_handle;
}

static void init_lcd_touch(esp_lcd_touch_handle_t *tp_handle)
{
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TP_PIN_NUM_SDA,
        .scl_io_num = TP_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* Initialize I2C */
    ESP_ERROR_CHECK(i2c_param_config(TP_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(TP_I2C_NUM, i2c_conf.mode, 0, 0, 0));
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP;
    /* Touch IO handle */
    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TP_I2C_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TP_PIN_NUM_RST,
        .int_gpio_num = TP_PIN_NUM_INT,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    *tp_handle = tp;
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    /* Configure LCD Backlight GPIO */
    gpio_config_t io_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&io_config));
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
    /* Configure LCD RD GPIO (always HIGH) */
    io_config.mode = GPIO_MODE_OUTPUT;
    io_config.pin_bit_mask = 1ULL << LCD_PIN_NUM_RD;
    ESP_ERROR_CHECK(gpio_config(&io_config));
    gpio_set_level(LCD_PIN_NUM_RD, 1);
    /* Configure LCD Bus (i80) */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    init_i80_16bit_bus(&io_handle, &disp_drv);
    /* Configure LCD panel */
    esp_lcd_panel_handle_t lcd_handle = NULL;
    init_lcd_panel(io_handle, &lcd_handle);
    /* Configure LCD touch */
    esp_lcd_touch_handle_t tp_handle = NULL;
    init_lcd_touch(&tp_handle);
#if CONFIG_USE_CANOPENNODE
    CO_ESP32_init();
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, false));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;
    uint32_t malloc_flags = 0;
    ESP_ERROR_CHECK(esp_dma_malloc(LCD_BUFFER_PX_CNT * sizeof(lv_color_t), malloc_flags, (void *)&buf1, NULL));
    ESP_ERROR_CHECK(esp_dma_malloc(LCD_BUFFER_PX_CNT * sizeof(lv_color_t), malloc_flags, (void *)&buf2, NULL));
    assert(buf1);
    assert(buf2);
    ESP_LOGI(TAG, "buf1@%p, buf2@%p", buf1, buf2);
    /* Initialize LVGL draw buffers */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_BUFFER_PX_CNT);
    /* Register display driver to LVGL */
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = lcd_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    /* Tick interface for LVGL (using esp_timer to generate 2ms periodic event) */
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
    /* Register input device (touch) to LVGL */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = tp_handle;
    lv_indev_drv_register(&indev_drv);

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handle, true));
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
}
