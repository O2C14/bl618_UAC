#include "shell.h"
#include "board.h"
#include "bflb_gpio.h"
#include "bflb_l1c.h"
#include "bflb_mtimer.h"

#include "bflb_i2c.h"
#include "bl616_common.h"
#include "bl616_glb.h"
#include "bflb_dma.h"
#include "bsp_es9038q2m.h"
#include "bflb_i2s.h"
// #include "fhm_onechannel_16k_20.h"
#include "math.h"
#include "1k_sin.h"

#include "UAC2.h"

#include "usbd_core.h" //usb

static struct bflb_device_s *i2s0;
static struct bflb_device_s *dma0_ch0;
static struct bflb_device_s *uart0;
static union All_Registers_bits *cfg;
// uint16_t pcm_data[48000];
static uint8_t init_state = 0;
static uint32_t last_tick = 0;
static struct bflb_dma_channel_lli_pool_s tx_llipool[100];
static struct bflb_dma_channel_lli_transfer_s tx_transfers[2];
static uint32_t tx_done = 0;
void use_tick()
{
    uint32_t now = bflb_mtimer_get_time_us();
    printf("tc done ,use ticks:%d\r\n", now - last_tick);
    last_tick = now;
}
void dma0_transfer_done(void *arg)
{ /**/
    tx_done += 1;
    use_tick();
}
uint8_t get_dma0_transfer_done(uint8_t state)
{
    return bflb_dma_channel_get_tcint_status(dma0_ch0);
}
void dma0_ch1_isr(void *arg)
{
    printf("rx done\r\n");
}
static struct bflb_device_s *gpio;
void i2s_gpio_init()
{

    // RCLK LRCLK FS应该是三个不同的东西,但是这就是要连es9038q2m的LRCLK
    /* I2S_RCLK LRCLK FS*/
    bflb_gpio_init(gpio, GPIO_PIN_13, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_DO BLUE*/
    bflb_gpio_init(gpio, GPIO_PIN_15, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2S_BCLK RED*/
    bflb_gpio_init(gpio, GPIO_PIN_20, GPIO_FUNC_I2S | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
}
static struct bflb_i2s_config_s i2s_cfg = {
    .bclk_freq_hz = 48000 * 16 * 2, /* bclk = Sampling_rate * frame_width * channel_num */
    .role = I2S_ROLE_MASTER,
    .format_mode = I2S_MODE_LEFT_JUSTIFIED,
    .channel_mode = I2S_CHANNEL_MODE_NUM_2,
    .frame_width = I2S_SLOT_WIDTH_16,
    .data_width = I2S_SLOT_WIDTH_16,
    .fs_offset_cycle = 0,

    .tx_fifo_threshold = 0,
};

static struct bflb_dma_channel_config_s tx_config = {
    .direction = DMA_MEMORY_TO_PERIPH,
    .src_req = DMA_REQUEST_NONE,
    .dst_req = DMA_REQUEST_I2S_TX,
    .src_addr_inc = DMA_ADDR_INCREMENT_ENABLE,
    .dst_addr_inc = DMA_ADDR_INCREMENT_DISABLE,
    .src_burst_count = DMA_BURST_INCR1,
    .dst_burst_count = DMA_BURST_INCR1,
    .src_width = DMA_DATA_WIDTH_16BIT,
    .dst_width = DMA_DATA_WIDTH_16BIT,
};

void i2s_dma_init()
{

    printf("i2s init\r\n");
    i2s0 = bflb_device_get_by_name("i2s0");
    /* i2s init */
    bflb_i2s_init(i2s0, &i2s_cfg);
    /* enable dma */
    bflb_i2s_link_txdma(i2s0, true);

    printf("dma init\r\n");
    dma0_ch0 = bflb_device_get_by_name("dma0_ch0");

    bflb_dma_channel_init(dma0_ch0, &tx_config);

    //bflb_dma_channel_irq_attach(dma0_ch0, dma0_transfer_done, NULL);

    tx_transfers[0].src_addr = (uint32_t)ek_sin;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
    tx_transfers[0].nbytes = sizeof(ek_sin);

    /*
        tx_transfers[0].src_addr = (uint32_t)(get_buffer());
        tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
        tx_transfers[0].nbytes = get_buffer_size();
    */

    printf("dma lli init\r\n");
    uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);

    bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
    // 这是连续循环模式

    printf("tx dma lli num: %d \r\n", num);
    bflb_dma_channel_start(dma0_ch0);
}

void reload_dma(uint32_t src_addr, uint32_t nbytes)
{
    tx_transfers[0].src_addr = src_addr;
    tx_transfers[0].dst_addr = (uint32_t)DMA_ADDR_I2S_TDR;
    tx_transfers[0].nbytes = nbytes;
    uint32_t num = bflb_dma_channel_lli_reload(dma0_ch0, tx_llipool, 100, tx_transfers, 1);
    bflb_dma_channel_lli_link_head(dma0_ch0, tx_llipool, num);
}
void start_i2s(void)
{
    bflb_dma_channel_start(dma0_ch0);
    bflb_i2s_link_txdma(i2s0, true);
}
void stop_i2s(void)
{
    bflb_i2s_link_txdma(i2s0, false);
    bflb_dma_channel_stop(dma0_ch0);
    
}
void mclk_out_init(uint8_t refClkDiv)
{
    GLB_Config_AUDIO_PLL_To_491P52M();
    GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_AUDIO);

    /*!< output MCLK,
        Will change the clock source of i2s,
        It needs to be called before i2s is initialized
        clock source 24.576M */

    /*!< MCLK = 24.576 / (5+1) = 4.096MHz */
    GLB_Set_I2S_CLK(ENABLE, refClkDiv, GLB_I2S_DI_SEL_I2S_DI_INPUT, GLB_I2S_DO_SEL_I2S_DO_OUTPT);
    GLB_Set_Chip_Clock_Out2_Sel(GLB_CHIP_CLK_OUT_2_I2S_REF_CLK);

    /* MCLK CLKOUT */
    // bflb_gpio_init(gpio, GPIO_PIN_23, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
}
int seti2sclock(int argc, char **argv)
{
    if (init_state == 1)
    {
        uint32_t clock_1 = Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_I2S);
        printf("clock %u\r\n", clock_1);
        return;
    }
    i2s_gpio_init();
    if (argc >= 1)
    {
        printf("%s", argv[1]);
        if (strcmp(argv[1], "384") == 0)
        {
            GLB_Config_AUDIO_PLL_To_384M();
        }
        else if (strcmp(argv[1], "400") == 0)
        {
            GLB_Config_AUDIO_PLL_To_400M();
        }
        else if (strcmp(argv[1], "451") == 0)
        {
            GLB_Config_AUDIO_PLL_To_451P58M(); // 44100x16x2x4x8
        }
        else if (strcmp(argv[1], "491") == 0)
        {
            GLB_Config_AUDIO_PLL_To_491P52M(); // 48000x16x2x4x8
        }
        else if (strcmp(argv[1], "240") == 0)
        {
            GLB_Config_AUDIO_PLL_To_240M();
        }
        else if (strcmp(argv[1], "245") == 0)
        {
            GLB_Config_AUDIO_PLL_To_245P76M();
        }
        uint8_t refClkDiv = 0;
        if (argc >= 2)
        {
            printf("%s\r\n", argv[2]);
            refClkDiv = argv[2][0] - '0';
        }
        GLB_PER_Clock_UnGate(GLB_AHB_CLOCK_AUDIO);
        GLB_Set_I2S_CLK(ENABLE, refClkDiv, GLB_I2S_DI_SEL_I2S_DI_INPUT, GLB_I2S_DO_SEL_I2S_DO_OUTPT);
        GLB_Set_Chip_Clock_Out3_Sel(GLB_CHIP_CLK_OUT_3_I2S_REF_CLK);
    }

    i2s_dma_init();
    bflb_i2s_feature_control(i2s0, I2S_CMD_DATA_ENABLE, I2S_CMD_DATA_ENABLE_TX);

    {
        bflb_mtimer_delay_ms(500);
        uint32_t clock_1 = Clock_Peripheral_Clock_Get(BL_PERIPHERAL_CLOCK_I2S);
        uint32_t divs = BL_GET_REG_BITS_VAL(BL_RD_REG(GLB_BASE, GLB_DBI_CFG0), GLB_DBI_CLK_DIV);
        printf("clock %u div %u\r\n", clock_1 * divs, divs);
    }
    init_state = 1;
    return 0;
}

int i2c_cmd(int argc, char **argv)
{
    if (argc >= 1)
    {
        if (strcmp(argv[1], "dump") == 0)
        {
            es9038q2m_Reg_Dump();
        }
        if (argc >= 2)
        {
            if (strcmp(argv[1], "read") == 0)
            {
                uint8_t address = 0;
                address += (argv[2][0] - '0') * 100;
                address += (argv[2][1] - '0') * 10;
                address += argv[2][2] - '0';
                print_regsiter(cfg->datas[address]);
            }
            if (argc >= 3)
            {
                if (strcmp(argv[1], "write") == 0)
                {
                    uint8_t data = 0;
                    data =
                        (((argv[3][0] - '0') & 1) << 7) |
                        (((argv[3][1] - '0') & 1) << 6) |
                        (((argv[3][2] - '0') & 1) << 5) |
                        (((argv[3][3] - '0') & 1) << 4) |
                        (((argv[3][4] - '0') & 1) << 3) |
                        (((argv[3][5] - '0') & 1) << 2) |
                        (((argv[3][6] - '0') & 1) << 1) |
                        ((argv[3][7] - '0') & 1);
                    uint8_t address = 0;
                    address += (argv[2][0] - '0') * 100;
                    address += (argv[2][1] - '0') * 10;
                    address += argv[2][2] - '0';
                    cfg->datas[address] = data;
                    es9038q2m_Reg_over_write();
                }
            }
        }
    }
}
void control_dma0(int argc, char **argv)
{
    uint8_t state = 1;
    if (argc >= 1)
    {
        if (argc >= 2)
        {
            if (strcmp(argv[2], "start") == 0)
            {
                state = 1;
            }
            else if (strcmp(argv[2], "stop") == 0)
            {
                state = 0;
            }
        }
        if (strcmp(argv[1], "dma0_ch0") == 0)
        {
            if (state)
            {
                //bflb_i2s_link_txdma(i2s0, true);
                bflb_dma_channel_start(dma0_ch0);
            }
            else
            {
                //bflb_i2s_link_txdma(i2s0, false);
                bflb_dma_channel_stop(dma0_ch0);
            }
        }
    }
}
SHELL_CMD_EXPORT(seti2sclock, seti2sclock test);
// seti2sclock 491 7
// i2c_cmd dump

SHELL_CMD_EXPORT(i2c_cmd, i2c_cmd test);
SHELL_CMD_EXPORT(control_dma0, control_dma0 test);
int main(void)
{

    int ch;
    board_init();
    gpio = bflb_device_get_by_name("gpio");
    ///* MCLK CLKOUT GREEN */
    bflb_gpio_init(gpio, GPIO_PIN_14, GPIO_FUNC_CLKOUT | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    // i2s_gpio_init();

    // mclk_out_init(5);
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_OUTPUT | GPIO_PULLDOWN | GPIO_SMT_EN | GPIO_DRV_3);
    bflb_gpio_reset(gpio, GPIO_PIN_27);
    // GPIO_PIN_27做为es9038q2m的硬重置引脚

    /* I2C0_SCL */
    bflb_gpio_init(gpio, GPIO_PIN_28, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);
    /* I2C0_SDA */
    bflb_gpio_init(gpio, GPIO_PIN_29, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_3);

    es9038q2m_I2C_Init();
    bflb_gpio_set(gpio, GPIO_PIN_27);
    bflb_mtimer_delay_ms(100);

    es9038q2m_Reg_Dump();

    cfg = get_reg();
    cfg->canwrite.System_Registers |= 0b00000001;
    cfg->canwrite.Input_selection = 0b00010000;
    cfg->canwrite.Soft_Start_Configuration = 0b10000010;
    // 踩过的坑,es9038q2m要用这个寄存器来启动模拟输出
    // i2c_cmd write 001 01010000
    // i2c_cmd dump

    /*
    cfg->canwrite.De_emphasis = 0x48;
    cfg->canwrite.GPIO1_2_Configuration = 0xff;
    cfg->canwrite.Master_Mode = 0x00;
    cfg->canwrite.Soft_Start_Configuration = 0x0c;
    cfg->canwrite.General_Configuration = 0x54;
    cfg->canwrite.General_Configuration_2 = 0x40;
    */
    //cfg->canwrite.General_Configuration = 0b11010111;//18db gain
    //cfg->canwrite.Volume_Control[0] = 80;
    //cfg->canwrite.Volume_Control[1] = 80;
    cfg->canwrite.Volume_Control[0] = 40;
    cfg->canwrite.Volume_Control[1] = 40;
    es9038q2m_Reg_over_write();
    es9038q2m_Reg_Dump();
    //
    char *clock_cfg[3] = {"seti2sclock", "491", "7"}; // 这是8分频48000x16x2x4x8=49.152mhz,4是固定的
    seti2sclock(2, clock_cfg);
    //bflb_i2s_link_txdma(i2s0, false);
    uart0 = bflb_device_get_by_name("uart0");
    audio_v2_init();
    shell_init();

    /*//1k正弦波生成器
    float PI = 3.141592653589793238462643383279502884193993;
    float Sampling_rate = 48.0 * 1000.0;
    float freq_hz = 1.0 * 1000.0;
    int bits = 16;
    for (int n = 0; n < 48000; n += 1)
    {
        pcm_data[n] = (uint16_t)((int)(((float)(1 << (bits - 1)) * 1.8) * (1.0 + sin(2.0 * PI * ((freq_hz / Sampling_rate) * ((float)n))))));
        if ((n + 1) % 5 == 0)
        {
        }
        // std::cout <<(short)((__int64)(((float)(1 << (bits - 1))) *(1.0+sin(2.0 * PI * ((freq_hz / Sampling_rate) * (n))))))<<",";
        //  printf("%lf,",((double)(1<<bits))*sin(2.0*PI*(freq_hz/Sampling_rate)*((double)n)));
    }
*/

    /**
    uint32_t clock_1 = Clock_Get_Audio_PLL_Output();
    uint32_t div_1 = Clock_Get_I2S_Div_Val();
    printf("clock %u div %u\r\n",clock_1,div_1);
    */

    // uint8_t refClkDiv = 1;
    while (1)
    { /*
      printf("%02x\r\n", refClkDiv);
      mclk_out_init(refClkDiv - 1);
      bflb_mtimer_delay_ms(2000);
      refClkDiv += 1;
      if (refClkDiv == 10)
      {
          refClkDiv = 1;
      }*/

        // cherryusb_audio_main_task();
        /**/
        // reload_dma((uint32_t)(get_buffer()), get_buffer_size());
        //  bflb_mtimer_delay_ms(50);
        if ((ch = bflb_uart_getchar(uart0)) != -1)
        {
            shell_handler(ch);
        }
    }
}
