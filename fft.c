#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "audio_sys.h"
#include "board.h"
#include "esp_peripherals.h"
#include "i2s_stream.h"
#include "esp_dsp.h"
#include <malloc.h>
#include <math.h>




static const char *TAG = "ESPEAR";
#define RECORD_TIME_SECONDS (10)

#define BUFFER_PROCESS_SIZE 1024
#define BLINK_GPIO GREEN_LED_GPIO
static float result_data[BUFFER_PROCESS_SIZE];
int16_t *audio_buffer;
int16_t *wind_buffer;




audio_element_err_t cb_fft(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait,void *context)
{   
    audio_buffer = buffer;
    while (audio_buffer < buffer+len){
        dsps_mul_s16_ansi(audio_buffer, wind_buffer, audio_buffer, BUFFER_PROCESS_SIZE, 1, 1, 1, 15);
        dsps_fft2r_sc16_ae32(audio_buffer, BUFFER_PROCESS_SIZE);
        dsps_bit_rev_sc16_ansi(audio_buffer, BUFFER_PROCESS_SIZE);
        float largest = audio_buffer[0]*audio_buffer[0];
        int index = 0;
        for (int i = 0 ; i < BUFFER_PROCESS_SIZE ; i++) {

            float spectrum_sqr = audio_buffer[i] * audio_buffer[i];
            if( spectrum_sqr>largest){
                largest = spectrum_sqr;
                index = i;
            }
            float spectrum_dB = 10 * log10f(0.1 + spectrum_sqr);
            // Multiply with sime coefficient for better view data on screen
            spectrum_dB = 4 * spectrum_dB;
            // Apply moving average of spectrum
            result_data[i] = 0.8 * result_data[i] + 0.2 * spectrum_dB;
        }
        ESP_LOGI(TAG, "largest at index %i, value %f", index, largest);
        gpio_set_level(BLINK_GPIO, largest > 10000000.f && index >=112 && index<=114);
        audio_buffer += BUFFER_PROCESS_SIZE*sizeof(int16_t);

    }
    
    return len;
}

void init_fft()
{
    esp_err_t ret = dsps_fft2r_init_sc16(NULL, BUFFER_PROCESS_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Not possible to initialize FFT esp-dsp from library!");
        return;
    }
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);


    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    int audio_chunksize = BUFFER_PROCESS_SIZE;
    wind_buffer = (int16_t *)memalign(16, (audio_chunksize + 16) * sizeof(int16_t));


    // Generate window and convert it to int16_t
    dsps_wind_hann_f32(result_data, audio_chunksize);
    for (int i = 0 ; i < audio_chunksize; i++) {
        wind_buffer[i] = (int16_t)(result_data[i] * 32767);
    }

    init_fft();

    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_reader;

    ESP_LOGI(TAG, "[ 1 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    
    // change input to aux in
    audio_hal_deinit(board_handle->audio_hal);
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_codec_cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE2;
    board_handle->audio_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8388_DEFAULT_HANDLE);

    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[2.0] Create audio pipeline for listening");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);


    ESP_LOGI(TAG, "[2.1] Create i2s stream to read audio data from codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    i2s_cfg.type = AUDIO_STREAM_READER;
    i2s_cfg.std_cfg.clk_cfg.sample_rate_hz=16000;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg);

    audio_element_set_write_cb(i2s_stream_reader, cb_fft, NULL);
    

    ESP_LOGI(TAG, "[2.2] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");

    ESP_LOGI(TAG, "[2.3] Link it together");
    const char *link_tag_main[1] = {"i2s"};
    audio_pipeline_link(pipeline, &link_tag_main[0], 1);

    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[3.1] Listening event from pipeline");
    audio_pipeline_set_listener(pipeline, evt);


    ESP_LOGI(TAG, "[ 4 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[ 5 ] Listen for all pipeline events, record for %d Seconds", RECORD_TIME_SECONDS);
    int second_recorded = 0;
    while (1) {
        audio_event_iface_msg_t msg;
        if (audio_event_iface_listen(evt, &msg, 1) != ESP_OK) {
            audio_element_info_t info;
            audio_element_getinfo(i2s_stream_reader, &info);
            int new_dur = info.byte_pos / (info.channels*(info.bits/8)*info.sample_rates);
            if(new_dur > second_recorded){
                second_recorded = new_dur;
                ESP_LOGI(TAG, "[ * ] Recording ... %d", second_recorded);
                if (second_recorded >= RECORD_TIME_SECONDS) {
                    break;
                }
            }
        }
    }
    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    audio_pipeline_unregister(pipeline, i2s_stream_reader);

    
    /* Terminal the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);


    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_reader);
}