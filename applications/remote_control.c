#include "main.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "string.h"
#include "cmsis_os.h"

#define RC_CH_VALUE_OFFSET ((uint16_t)1024)

uint8_t temp = 0;
RC_Ctl_t remoteCtrl[2] = {0};
int is_Zimiao;  // 自瞄标志位

RC_ctrl_t rc_ctrl;
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

void remoteReceive(const uint8_t *sbusBuf)
{
    remoteCtrl[temp].rc.rockerrx = ((sbusBuf[0] | (sbusBuf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    remoteCtrl[temp].rc.rockerry = (((sbusBuf[1] >> 3) | (sbusBuf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    remoteCtrl[temp].rc.rockerlx = (((sbusBuf[2] >> 6) | (sbusBuf[3] << 2) | (sbusBuf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET;
    remoteCtrl[temp].rc.rockerly = ((((sbusBuf[4] >> 1) | (sbusBuf[5] << 7))) & 0x07ff) - RC_CH_VALUE_OFFSET;

    remoteCtrl[temp].rc.dial = (((sbusBuf[16] | (sbusBuf[17] << 8))) & 0x07ff) - RC_CH_VALUE_OFFSET;

    remoteCtrl[temp].rc.switchLeft = ((sbusBuf[5] >> 4) & 0x000C) >> 2;
    remoteCtrl[temp].rc.switchRight = ((sbusBuf[5] >> 4) & 0x0003);

    remoteCtrl[temp].mouse.x = (sbusBuf[6] | (sbusBuf[7] << 8));
    remoteCtrl[temp].mouse.y = (sbusBuf[8] | sbusBuf[9] << 8);
    remoteCtrl[temp].mouse.press_l = sbusBuf[12];
    remoteCtrl[temp].mouse.press_r = sbusBuf[13];
}

void USART3_IRQHandler(void)
{
//	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    if(huart3.Instance->SR & UART_FLAG_RXNE)  //接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
						//获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
					if(this_time_rx_len == RC_FRAME_LENGTH)
					{
            sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
					}
            
					remoteCtrl[0].rc.rockerlx = rc_ctrl.rc.ch[0];
					remoteCtrl[0].rc.rockerly = rc_ctrl.rc.ch[1];
					remoteCtrl[0].rc.rockerrx = rc_ctrl.rc.ch[2];
					remoteCtrl[0].rc.rockerry = rc_ctrl.rc.ch[3];
					remoteCtrl[0].rc.dial = rc_ctrl.rc.ch[4];
					remoteCtrl[0].rc.switchLeft = rc_ctrl.rc.s[0];
					remoteCtrl[0].rc.switchRight = rc_ctrl.rc.s[1];
					remoteCtrl[0].mouse.x = rc_ctrl.mouse.x;
					remoteCtrl[0].mouse.y = rc_ctrl.mouse.y;
					remoteCtrl[0].mouse.z = rc_ctrl.mouse.z;
					remoteCtrl[0].mouse.press_l = rc_ctrl.mouse.press_l;
					remoteCtrl[0].mouse.press_r = rc_ctrl.mouse.press_r;
					remoteCtrl[0].key.v = rc_ctrl.key.v;
					is_Zimiao = (remoteCtrl[0].rc.switchLeft == 2) ? 1 : 0;
        }
        else
        {
					HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);
						//获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
					if(this_time_rx_len == RC_FRAME_LENGTH)
					{
            sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
					}
					
					remoteCtrl[0].rc.rockerlx = rc_ctrl.rc.ch[0];
					remoteCtrl[0].rc.rockerly = rc_ctrl.rc.ch[1];
					remoteCtrl[0].rc.rockerrx = rc_ctrl.rc.ch[2];
					remoteCtrl[0].rc.rockerry = rc_ctrl.rc.ch[3];
					remoteCtrl[0].rc.dial = rc_ctrl.rc.ch[4];
					remoteCtrl[0].rc.switchLeft = rc_ctrl.rc.s[0];
					remoteCtrl[0].rc.switchRight = rc_ctrl.rc.s[1];
					remoteCtrl[0].mouse.x = rc_ctrl.mouse.x;
					remoteCtrl[0].mouse.y = rc_ctrl.mouse.y;
					remoteCtrl[0].mouse.z = rc_ctrl.mouse.z;
					remoteCtrl[0].mouse.press_l = rc_ctrl.mouse.press_l;
					remoteCtrl[0].mouse.press_r = rc_ctrl.mouse.press_r;
					remoteCtrl[0].key.v = rc_ctrl.key.v;
					is_Zimiao = (remoteCtrl[0].rc.switchLeft == 2) ? 1 : 0;
        }
    }

}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}
