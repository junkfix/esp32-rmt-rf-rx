#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_rx.h"

#define RF_PIN 2

#define P_HIGH 1040
#define P_LOW 340
#define P_MARGIN 70
#define P_SKIPMIN 250

#define RF_DEBUG 1

// https://github.com/junkfix/esp32-rmt-rf-rx

void recvRF(void* param){
	const uint16_t tlow = (P_HIGH - P_LOW - (2 * P_MARGIN));
	const uint16_t thigh = (P_HIGH - P_LOW + (2 * P_MARGIN));
	
	rmt_channel_handle_t rx_channel = NULL;
	rmt_symbol_word_t symbols[64];
	rmt_rx_done_event_data_t rx_data;
	
	rmt_receive_config_t rx_config = {
		.signal_range_min_ns = 2000,
		.signal_range_max_ns = 1250000,
	};
	
	rmt_rx_channel_config_t rx_ch_conf = {
		.gpio_num = static_cast<gpio_num_t>(RF_PIN),
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = 1000000,
		.mem_block_symbols = 64,
	};
	
	rmt_new_rx_channel(&rx_ch_conf, &rx_channel);
	QueueHandle_t rx_queue = xQueueCreate(1, sizeof(rx_data));
	assert(rx_queue);
	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = rfrx_done,
	};

	rmt_rx_register_event_callbacks(rx_channel, &cbs, rx_queue);
	rmt_enable(rx_channel);
	rmt_receive(rx_channel, symbols, sizeof(symbols), &rx_config);
	for(;;){
		//if(isOTA){break;}
		if (xQueueReceive(rx_queue, &rx_data, pdMS_TO_TICKS(1000)) == pdPASS) {
			
			char buf[128];
			size_t len = rx_data.num_symbols;
			uint32_t code = 0;
			rmt_symbol_word_t *cur = rx_data.received_symbols;
			int16_t diff = 0;
			uint16_t low = 0, high = 0, dur0 = 0, dur1 = 0, err = 0;
			
			if (len > 23){
				for (uint8_t i=0; i < 24 ; i++ ) {
					dur0 = (uint16_t)cur[i].duration0;
					dur1 = (uint16_t)cur[i].duration1;
					if(!(cur[i].level0 && !cur[i].level1 && dur0 >= P_SKIPMIN && dur1 >= P_SKIPMIN)){
						code = 0;
						break;
					}
					if((dur0 - dur1) > 0){
						code = ( code | (1ULL << (23 - i)));
						high += dur0;
						low += dur1;
					}else{
						high += dur1;
						low += dur0;
					}
					
					diff = abs(dur0 - dur1);
					if((diff < tlow) || (diff > thigh)){
						err++;
					}
				}
				if(code){
					high /= 24;
					low  /= 24;
					snprintf(buf, 50, "RF%d 0x%x E=%d H%d L%d", len, code, err, high, low );
					Serial.println((const char *)buf);
				}

				if(RF_DEBUG){
					char tbuf[20];
					snprintf(tbuf, 20, "Rf%d: ", len);
					strcpy(buf, tbuf);
					for (uint8_t i=0; i < len ; i++ ) {
						if(strlen(buf)>100){
							Serial.print((const char *)buf); buf[0]='\0';
						}
						int d0 = cur[i].duration0; if(!cur[i].level0){d0 *= -1;}
						int d1 = cur[i].duration1; if(!cur[i].level1){d1 *= -1;}
						snprintf(tbuf, 20, "%d,%d ", d0, d1);
						strcat(buf, tbuf);
					}
					Serial.println((const char *)buf); Serial.println();
				}
			}

			rmt_receive(rx_channel, symbols, sizeof(symbols), &rx_config);
		}
	}
	rmt_disable(rx_channel);
	rmt_del_channel(rx_channel);
	vTaskDelete(NULL);
}

bool rfrx_done(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *udata){
	BaseType_t high_task_wakeup = pdFALSE;
	QueueHandle_t drx_queue = (QueueHandle_t)udata;
	xQueueSendFromISR(drx_queue, edata, &high_task_wakeup);
	return high_task_wakeup == pdTRUE;
}

void setup(){
	Serial.begin(115200);
	xTaskCreatePinnedToCore(recvRF, "recvRF", 2048, NULL, 10, NULL, 1);
}

void loop() {
	delay(2000);
}
