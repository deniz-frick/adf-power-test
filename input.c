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
#include <malloc.h>


static const char *TAG = "ESPEAR";
#define RECORD_TIME_SECONDS (10)

audio_element_err_t cb_nop(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait,void *context){
    return len;
}


void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

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
    audio_element_set_write_cb(i2s_stream_reader, cb_nop, NULL);

    

    ESP_LOGI(TAG, "[2.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");

    ESP_LOGI(TAG, "[2.4] Link it together");
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
        if (audio_event_iface_listen(evt, &msg, 10) != ESP_OK) {
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