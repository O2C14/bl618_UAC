#include "usbd_core.h"
#include "usbd_audio.h"
#include "bflb_dma.h"

uint8_t *get_buffer(void);
void audio_v2_init(void);
void audio_v2_test(void);
void reload_dma(uint32_t src_addr,uint32_t nbytes);
void cherryusb_audio_main_task(void);
uint8_t get_dma0_transfer_done(uint8_t state);
uint32_t get_buffer_size(void);

void start_i2s(void);
void stop_i2s(void);