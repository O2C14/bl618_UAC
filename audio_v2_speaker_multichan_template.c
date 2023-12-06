#include "UAC2.h"
#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     500
#define USBD_LANGID_STRING 1033

/**/
#ifdef CONFIG_USB_HS
#define EP_INTERVAL 0x04
#else
#define EP_INTERVAL 0x01
#endif



#define AUDIO_OUT_EP 0x01

#define AUDIO_OUT_CLOCK_ID 0x01
#define AUDIO_OUT_FU_ID    0x03

#define AUDIO_FREQ      48000
#define HALF_WORD_BYTES 2  //2 half word (one channel)
#define SAMPLE_BITS     16 //16 bit per channel

#define BMCONTROL (AUDIO_V2_FU_CONTROL_MUTE | AUDIO_V2_FU_CONTROL_VOLUME)

#define OUT_CHANNEL_NUM 2

#if OUT_CHANNEL_NUM == 1
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000000
#elif OUT_CHANNEL_NUM == 2
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000003
#elif OUT_CHANNEL_NUM == 3
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x00000007
#elif OUT_CHANNEL_NUM == 4
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000000f
#elif OUT_CHANNEL_NUM == 5
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000001f
#elif OUT_CHANNEL_NUM == 6
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000003F
#elif OUT_CHANNEL_NUM == 7
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x0000007f
#elif OUT_CHANNEL_NUM == 8
#define OUTPUT_CTRL      DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL), DBVAL(BMCONTROL)
#define OUTPUT_CH_ENABLE 0x000000ff
#endif

//#define AUDIO_OUT_PACKET ((uint32_t)(((AUDIO_FREQ * HALF_WORD_BYTES * OUT_CHANNEL_NUM) / 1000 + HALF_WORD_BYTES)*2))

#define AUDIO_OUT_PACKET ((uint32_t)((AUDIO_FREQ * HALF_WORD_BYTES * OUT_CHANNEL_NUM) / 1000))

#define USB_AUDIO_CONFIG_DESC_SIZ (9 +                                                     \
                                   AUDIO_V2_AC_DESCRIPTOR_INIT_LEN +                       \
                                   AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                  \
                                   AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                \
                                   AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(OUT_CHANNEL_NUM) + \
                                   AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC +               \
                                   AUDIO_V2_AS_DESCRIPTOR_INIT_LEN)

#define AUDIO_AC_SIZ (AUDIO_V2_SIZEOF_AC_HEADER_DESC +                        \
                      AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +                  \
                      AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +                \
                      AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(OUT_CHANNEL_NUM) + \
                      AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

const uint8_t audio_v2_descriptor[] = {
    //USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0001, 0x01),
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0001, 0x00),
    USB_CONFIG_DESCRIPTOR_INIT(USB_AUDIO_CONFIG_DESC_SIZ, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_V2_AC_DESCRIPTOR_INIT(0x00, 0x02, AUDIO_AC_SIZ, AUDIO_CATEGORY_HEADSET, 0x00, 0x00),
    AUDIO_V2_AC_CLOCK_SOURCE_DESCRIPTOR_INIT(AUDIO_OUT_CLOCK_ID, 0x03, 0x03),
    AUDIO_V2_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x02, AUDIO_TERMINAL_STREAMING, 0x01, OUT_CHANNEL_NUM, OUTPUT_CH_ENABLE, 0x0000),
    AUDIO_V2_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_OUT_FU_ID, 0x02, OUTPUT_CTRL),
    AUDIO_V2_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x04, AUDIO_OUTTERM_SPEAKER, 0x03, 0x01, 0x0000),
    AUDIO_V2_Interface_Descriptor(0x01,0x00),
    //AUDIO_V2_AS_DESCRIPTOR_INIT_fix(0x01,0x01, 0x02, OUT_CHANNEL_NUM, OUTPUT_CH_ENABLE, HALF_WORD_BYTES, SAMPLE_BITS, AUDIO_OUT_EP, 0x0D, AUDIO_OUT_PACKET, EP_INTERVAL),
    AUDIO_V2_AS_DESCRIPTOR_INIT_fix(0x01,0x01, 0x02, OUT_CHANNEL_NUM, OUTPUT_CH_ENABLE, HALF_WORD_BYTES, SAMPLE_BITS, AUDIO_OUT_EP, 0x0D, AUDIO_OUT_PACKET, EP_INTERVAL),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'U', 0x00,                  /* wcChar10 */
    'A', 0x00,                  /* wcChar11 */
    'C', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'E', 0x00,                  /* wcChar15 */
    'M', 0x00,                  /* wcChar16 */
    'O', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '1', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '3', 0x00,                  /* wcChar5 */
    '1', 0x00,                  /* wcChar6 */
    '0', 0x00,                  /* wcChar7 */
    '0', 0x00,                  /* wcChar8 */
    '3', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

static const uint8_t default_sampling_freq_table[] = {
    AUDIO_SAMPLE_FREQ_NUM(5),
    AUDIO_SAMPLE_FREQ_4B(44100),AUDIO_SAMPLE_FREQ_4B(44100),AUDIO_SAMPLE_FREQ_4B(0x00),
    AUDIO_SAMPLE_FREQ_4B(48000),AUDIO_SAMPLE_FREQ_4B(48000),AUDIO_SAMPLE_FREQ_4B(0x00),
    AUDIO_SAMPLE_FREQ_4B(96000),AUDIO_SAMPLE_FREQ_4B(96000),AUDIO_SAMPLE_FREQ_4B(0x00),
    AUDIO_SAMPLE_FREQ_4B(192000),AUDIO_SAMPLE_FREQ_4B(192000),AUDIO_SAMPLE_FREQ_4B(0x00),
    AUDIO_SAMPLE_FREQ_4B(384000),AUDIO_SAMPLE_FREQ_4B(384000),AUDIO_SAMPLE_FREQ_4B(0x00),
};
#define AUDIO_BUFFER_COUNT      16
/* Static Variables */
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t s_speaker_out_buffer[AUDIO_BUFFER_COUNT][AUDIO_OUT_PACKET];
static uint32_t s_speaker_out_buffer_size[AUDIO_BUFFER_COUNT];
static volatile uint32_t s_speaker_i2s_mclk_hz;
static volatile bool s_speaker_rx_flag;
static volatile uint8_t s_speaker_out_buffer_front;
static volatile uint8_t s_speaker_out_buffer_rear;
static volatile bool s_speaker_dma_transfer_req;
static volatile bool s_speaker_dma_transfer_done;
static volatile uint32_t s_speaker_sample_rate;
static volatile int32_t s_speaker_volume_percent;
static volatile bool s_speaker_mute;

static struct usbd_interface intf0;
static struct usbd_interface intf1;

static void usbd_audio_iso_out_callback(uint8_t ep, uint32_t nbytes);
static struct usbd_endpoint audio_out_ep = {
    .ep_cb = usbd_audio_iso_out_callback,
    .ep_addr = AUDIO_OUT_EP
};

struct audio_entity_info audio_entity_table[] = {
    {
        .bEntityId = AUDIO_OUT_CLOCK_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_CLOCK_SOURCE,
        .ep = AUDIO_OUT_EP
    },
    {
        .bEntityId = AUDIO_OUT_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_OUT_EP
    },
};

/* Static Functions Declaration */
static bool speaker_out_buff_is_empty(void);

/* Extern Functions Definition */
void audio_v2_init(void)
{
    usbd_desc_register(audio_v2_descriptor);
    usbd_add_interface(usbd_audio_init_intf(&intf0, 0x200, audio_entity_table, 2));
    usbd_add_interface(usbd_audio_init_intf(&intf1, 0x200, audio_entity_table, 2));
    usbd_add_endpoint(&audio_out_ep);

    usbd_initialize();
}
void usbd_audio_open(uint8_t intf)
{
    s_speaker_rx_flag = 1;
    s_speaker_out_buffer_front = 0;
    s_speaker_out_buffer_rear = 0;
    s_speaker_dma_transfer_req = true;
    /* setup first out ep read transfer */
    usbd_ep_start_read(AUDIO_OUT_EP, (uint8_t *)&s_speaker_out_buffer[s_speaker_out_buffer_rear][0], AUDIO_OUT_PACKET);
    USB_LOG_RAW("OPEN SPEAKER\r\n");
    reload_dma(s_speaker_out_buffer,sizeof(s_speaker_out_buffer));
}
uint8_t i2s_state = 0;
void usbd_audio_close(uint8_t intf)
{
    s_speaker_rx_flag = 0;
    stop_i2s();
    i2s_state = 0;
    USB_LOG_RAW("CLOSE SPEAKER\r\n");
}

void usbd_event_handler(uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_audio_set_volume(uint8_t ep, uint8_t ch, int volume)
{
    if (ep == AUDIO_OUT_EP) {
        printf("%02X volume:%d\r\n",ch,volume);
    }
}

int usbd_audio_get_volume(uint8_t ep, uint8_t ch)
{
    int volume = 0;

    if (ep == AUDIO_OUT_EP) {
        volume = s_speaker_volume_percent;
    }

    return volume;
}

void usbd_audio_set_mute(uint8_t ep, uint8_t ch, bool mute)
{

}

bool usbd_audio_get_mute(uint8_t ep, uint8_t ch)
{
    bool mute = false;

    if (ep == AUDIO_OUT_EP) {
        mute = s_speaker_mute;
    }

    return mute;
}

void usbd_audio_set_sampling_freq(uint8_t ep, uint32_t sampling_freq)
{
    uint8_t state;

    if (ep == AUDIO_OUT_EP) {
        s_speaker_sample_rate = sampling_freq;
        //设置i2s采样

        //设置dac采样

        //同步dma指针
        //s_speaker_out_buffer_front = s_speaker_out_buffer_rear;
        s_speaker_dma_transfer_req = true;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t ep)
{
    uint32_t freq = 0;

    if (ep == AUDIO_OUT_EP) {
        freq = s_speaker_sample_rate;
    }

    return freq;
}

void usbd_audio_get_sampling_freq_table(uint8_t ep, uint8_t **sampling_freq_table)
{
    if (ep == AUDIO_OUT_EP) {
        *sampling_freq_table = (uint8_t *)default_sampling_freq_table;
    }
}
uint8_t rx_times = 0;

/* Static Function Definition */
static void usbd_audio_iso_out_callback(uint8_t ep, uint32_t nbytes)
{
    if (s_speaker_rx_flag&&(nbytes>=190)) {
        s_speaker_out_buffer_size[s_speaker_out_buffer_rear] = nbytes;
        s_speaker_out_buffer_rear++;
        if (s_speaker_out_buffer_rear >= AUDIO_BUFFER_COUNT) {
            s_speaker_out_buffer_rear = 0;
        }
        usbd_ep_start_read(ep, &s_speaker_out_buffer[s_speaker_out_buffer_rear][0], AUDIO_OUT_PACKET);
        //cherryusb_audio_main_task();
        if(i2s_state == 0){
            rx_times+=1;
            if(rx_times == 3){
                start_i2s();
                rx_times = 0;
                i2s_state = 1;
            }
        }
    
    }
    

}
static bool speaker_out_buff_is_empty(void)
{
    bool empty = false;

    if ((s_speaker_out_buffer_front == s_speaker_out_buffer_rear) && s_speaker_rx_flag) {
        empty = true;
    }

    return empty;
}
uint8_t *get_buffer(void){
    return &s_speaker_out_buffer[s_speaker_out_buffer_front][0];
}
uint32_t get_buffer_size(void){
    return AUDIO_OUT_PACKET;
}