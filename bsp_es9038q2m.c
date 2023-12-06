#include "bflb_i2c.h"
#include "bsp_es9038q2m.h"

#define es9038q2m_I2C_SLAVE_ADDR 0x90>>1

static struct bflb_device_s *i2c0;
static struct bflb_i2c_msg_s msgs[2];
static union All_Registers_bits Regp;
/****************************************************************************/ /**
 * @brief  es9038q2m_I2C_Init
 *
 * @param  None
 *
 * @return None
 *
*******************************************************************************/
void es9038q2m_I2C_Init(void)
{
    i2c0 = bflb_device_get_by_name("i2c0");
    if (i2c0 == NULL) {
        printf("I2C_Init_fail\r\n");
        return -1;


    }
    bflb_i2c_init(i2c0, 20000);
    return 0;
}
void print_regsiter(uint8_t data){
    printf("%d%d%d%d%d%d%d%d",
        (data>> 7) & 1,
        (data>> 6) & 1,
        (data>> 5) & 1,
        (data>> 4) & 1,
        (data>> 3) & 1,
        (data>> 2) & 1,
        (data>> 1) & 1,
        data& 1);
}
/****************************************************************************/ /**
 * @brief  es9038q2m write register
 *
 * @param  addr: Register address
 * @param  data: data
 *
 * @return None
 *
*******************************************************************************/
int es9038q2m_Write_Reg(uint8_t Reg, uint8_t data)
{
    msgs[0].addr = es9038q2m_I2C_SLAVE_ADDR;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = &Reg;
    msgs[0].length = 1;
    msgs[1].addr = es9038q2m_I2C_SLAVE_ADDR;
    msgs[1].flags = 0;
    msgs[1].buffer = &data;
    msgs[1].length = 1;

    return bflb_i2c_transfer(i2c0, msgs, 2);
}

/****************************************************************************/ /**
 * @brief  es9038q2m_Read_Reg
 *
 * @param  addr: Register address
 * @param  rdata: data
 *
 * @return None
 *
*******************************************************************************/
int es9038q2m_Read_Reg(uint8_t Reg, uint8_t *rdata)
{
    msgs[0].addr = es9038q2m_I2C_SLAVE_ADDR;
    msgs[0].flags = I2C_M_NOSTOP;
    msgs[0].buffer = &Reg;
    msgs[0].length = 1;

    msgs[1].flags = I2C_M_READ;
    msgs[1].buffer = rdata;
    msgs[1].length = 1;

    return bflb_i2c_transfer(i2c0, msgs, 2);
}
union All_Registers_bits *get_reg(void){
    return &Regp;
}
void es9038q2m_Reg_Dump(void)
{
    int i;
    uint8_t tmp;
    for (i = 0; i <= 102; i++) {
        if(i>54&&i<64){continue;}
        int i2state = es9038q2m_Read_Reg(i, &tmp);
        if (i2state != 0) {
            printf("iic read err,code:%d\r\n",i2state);
        }else{
            Regp.datas[i] = tmp;
        }

        printf("Register %03d state:", i);
        print_regsiter(tmp);
        printf("\r\n");
    }
    //Regp.canwrite.Input_selection;
    //print_regsiter(Regp.canwrite.Input_selection);
    //printf("\r\n");
    //printf("%d\r\n",sizeof(Regp));
}

void es9038q2m_Reg_over_write(void)
{
    int i;
    for (i = 0; i <= 102; i++) {
        if(i > 54 && i < 64){ continue; }
        int i2state = es9038q2m_Write_Reg(i, Regp.datas[i]);
        if (i2state != 0) {
            printf("iic read err,code:%d\r\n", i2state );
        }
    }
}



void set_vol(){
    
}