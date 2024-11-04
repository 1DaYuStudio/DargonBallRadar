#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "LCD_Driver/ST7701S.h"
#include "demos/lv_demos.h"
#include "max98357.h"
#include "esp_random.h"

static const char *TAG = "example";

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (32 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       6
#define EXAMPLE_PIN_NUM_HSYNC          39
#define EXAMPLE_PIN_NUM_VSYNC          40
#define EXAMPLE_PIN_NUM_DE             41
#define EXAMPLE_PIN_NUM_PCLK           42
#define EXAMPLE_PIN_NUM_DATA0          17 // B0
#define EXAMPLE_PIN_NUM_DATA1          18 // B1
#define EXAMPLE_PIN_NUM_DATA2          8  // B2
#define EXAMPLE_PIN_NUM_DATA3          3  // B3
#define EXAMPLE_PIN_NUM_DATA4          46 // B4
#define EXAMPLE_PIN_NUM_DATA5          9  // G0
#define EXAMPLE_PIN_NUM_DATA6          10 // G1
#define EXAMPLE_PIN_NUM_DATA7          11 // G2
#define EXAMPLE_PIN_NUM_DATA8          12 // G3
#define EXAMPLE_PIN_NUM_DATA9          13 // G4
#define EXAMPLE_PIN_NUM_DATA10         14 // G5
#define EXAMPLE_PIN_NUM_DATA11         21  // R0
#define EXAMPLE_PIN_NUM_DATA12         47  // R1
#define EXAMPLE_PIN_NUM_DATA13         48 // R2
#define EXAMPLE_PIN_NUM_DATA14         45 // R3
#define EXAMPLE_PIN_NUM_DATA15         38 // R4
#define EXAMPLE_PIN_NUM_DISP_EN        -1
// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              480
#define EXAMPLE_LCD_V_RES              480

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB             2
#else
#define EXAMPLE_LCD_NUM_FB             1
#endif 

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

#define SPI_SDA 1
#define SPI_SCL 2
#define SPI_CS  5

#define  BUTTON_GPIO   4

lv_obj_t * icon1,*icon2,*icon3;
uint32_t random_x,random_y;

void led_task(void *pvParameter) {
    static uint16_t tick = 0;
    while (true)
    {
        if(tick == 0){
            tick = 1;
            lv_obj_clear_flag(icon1, LV_OBJ_FLAG_HIDDEN);//清空隐藏标志位=显示图片
            lv_obj_clear_flag(icon2, LV_OBJ_FLAG_HIDDEN);//清空隐藏标志位=显示图片
            lv_obj_clear_flag(icon3, LV_OBJ_FLAG_HIDDEN);//清空隐藏标志位=显示图片
        }
        else
        {
            tick = 0;
            lv_obj_add_flag(icon1, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(icon2, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(icon3, LV_OBJ_FLAG_HIDDEN);
        }

        vTaskDelay(pdMS_TO_TICKS(750));
    }
}

void i2s_music_task(void *pvParameter) {
             
    while (true)
    {
        i2s_update();      
    }
}

void button_task(void *pvParameter) {
    static uint16_t tick = 0;
    static uint8_t press = 0;
    while (true)
    {
        if(gpio_get_level(BUTTON_GPIO) == 0 )
        {
           if(press == 0)
           {
                tick++;
                if(tick >= 5)
                {
                    tick = 5;
                    press = 1; 
                }   
           }
        }
        else
        {
            if(press == 1)
            {
                tick--;
                if(tick == 0)
                {
                    press = 0;  
                    i2s_play_sound_button();  
                }
            } 
            else tick = 0;   
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void heartbeat_task(void *pvParameter) {
    while (true)
    {
        i2s_play_sound_bip();
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}


#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}


LV_IMG_DECLARE(test2);
LV_IMG_DECLARE(ball);

void app_main(void)
{   
    int random;
    ST7701S_handle st7701s = ST7701S_newObject(SPI_SDA, SPI_SCL, SPI_CS, SPI3_HOST, SPI_METHOD);
    
    ST7701S_screen_init(st7701s, 1);
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif
    
/********************* RGB LCD panel driver *********************/
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_PLL240M,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            .hsync_back_porch = 10,
            .hsync_front_porch = 50,
            .hsync_pulse_width = 8,
            .vsync_back_porch = 18,
            .vsync_front_porch = 8,
            .vsync_pulse_width = 2,        
            .flags.pclk_active_neg = false,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

/********************* LVGL *********************/

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));


lv_obj_t * icon = lv_img_create(lv_scr_act());
    lv_img_set_src(icon, &test2);

    random = esp_random();
    random_x = random % 150;
    random = esp_random();
    random_y = random %150;

    icon1 = lv_img_create(lv_scr_act());
    lv_img_set_src(icon1, &ball);
    lv_obj_center(icon1); // 将图片居中
    lv_obj_align(icon1, LV_ALIGN_CENTER, random_x, random_y);
    
    random = esp_random();
    random_x = -random % 150;
    random = esp_random();
    random_y = random %150;

    icon2 = lv_img_create(lv_scr_act());
    lv_img_set_src(icon2, &ball);
    lv_obj_center(icon2); // 将图片居中
    lv_obj_align(icon2, LV_ALIGN_CENTER, random_x, random_y);

    random = esp_random();
    random_x = -random % 150;
    random = esp_random();
    random_y = -random %150;

    icon3 = lv_img_create(lv_scr_act());
    lv_img_set_src(icon3, &ball);
    lv_obj_center(icon3); // 将图片居中
    lv_obj_align(icon3, LV_ALIGN_CENTER, random_x, random_y);

    i2s_create();

    gpio_config_t gpioInitCfg = {
		.mode = GPIO_MODE_INPUT,		//配置GPIO口为输入输出模式
		.pin_bit_mask = 1ULL << BUTTON_GPIO,		//掩码方式配置GPIO口输出，48可改为自己的GPIO
		.intr_type = GPIO_INTR_DISABLE,	//不使用引脚中断
		.pull_down_en = GPIO_PULLDOWN_DISABLE,	//不使能下拉
		.pull_up_en = GPIO_PULLUP_ENABLE		//不使能上拉
	};
	//配置GPIO
	ESP_ERROR_CHECK(gpio_config(&gpioInitCfg));

    xTaskCreatePinnedToCore(i2s_music_task, "i2s_music_task", 4096, NULL, 5, NULL,1);
    xTaskCreatePinnedToCore(button_task, "button_task", 4096, NULL, 5, NULL,1);
    xTaskCreatePinnedToCore(heartbeat_task, "heartbeat_task", 4096, NULL, 5, NULL,1);
    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, 5, NULL,1);
    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}
