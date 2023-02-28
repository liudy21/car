#include "zigbee_edc24.h"
#include "jy62.h"
#include "main.h"
#include "usart.h"

#define MAX_MSG_LEN 200
#define MAX_INFO_LEN 100
#define MAX_STATUS_LEN 150

uint8_t zigbeeMessage[MAX_MSG_LEN];    //原始数据
uint8_t gameInfoMessage[MAX_INFO_LEN]; //游戏信息数据�?0x01)
uint8_t gameStatusMessage[MAX_STATUS_LEN]; //游戏状态数据（0x05�?
uint8_t zigbeeSend[2][6]={0x55,0xAA,0x00,0x00,0x00,0x00,
                          0x55,0xAA,0x02,0x00,0x00,0x00};        //小车可能发送的信息�?0x00:请求游戏信息 0x02:设置充电桩）

/*uint8_t zigbeeMessage_test_1[MSG_LEN_TEST];
uint8_t zigbeeMessage_test_2[MSG_LEN_TEST];
uint8_t last_receive=0;*/

uint8_t receive_flag=0;
UART_HandleTypeDef* zigbee_huart;
extern UART_HandleTypeDef* jy62_huart;
extern uint8_t jy62_flag;
extern uint8_t receive;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  //HAL_UART_DeInit(huart);
  if (huart == zigbee_huart)
  {
    __HAL_UNLOCK(zigbee_huart);
    receive_flag=0;
    HAL_UARTEx_ReceiveToIdle_DMA(zigbee_huart,zigbeeMessage,MAX_MSG_LEN);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,uint16_t Size)
{
    if(huart == zigbee_huart)
    {
        /*rx_size+=Size;
        if(rx_size<MAX_MSG_LEN-1)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(zigbee_huart,zigbeeMessage+rx_size,MAX_MSG_LEN-rx_size);
        }*/
        receive_flag=1;
    }
}

static uint8_t CalculateChecksum(const uint8_t data[], int32_t count) 
{
  uint8_t checksum = 0;
  for (int32_t i = 0; i < count; ++i) 
  {
    checksum ^= data[i];
  }
  return checksum;
}

static float change_float_data(uint8_t* dat)
{
    float float_data;
    float_data=*((float*)dat);
    return float_data;
}

//接口函数
void zigbee_Init(UART_HandleTypeDef *huart)
{
    memset(zigbeeMessage,0x00,MAX_MSG_LEN);
    memset(gameInfoMessage,0x00,MAX_INFO_LEN);
    memset(gameStatusMessage,0x00,MAX_STATUS_LEN);
    zigbee_huart = huart;
    receive_flag=0;
    HAL_UARTEx_ReceiveToIdle_DMA(zigbee_huart,zigbeeMessage,MAX_MSG_LEN);
}

void zigbeeMessageRecord()
{
    uint16_t msgIndex=0;
    for(msgIndex=0;msgIndex<MAX_MSG_LEN;msgIndex++)
    {
        if(zigbeeMessage[msgIndex]==0x55&&zigbeeMessage[msgIndex+1]==0xAA)
        {
            int16_t tmpnum;
            //tmpnum=(int32_t)(zigbeeMessage[msgIndex+1]+(zigbeeMessage[msgIndex+2]<<8)+(zigbeeMessage[msgIndex+3]<<16)+(zigbeeMessage[msgIndex+4]<<24));
            tmpnum=*((int16_t*)(&zigbeeMessage[msgIndex+3]));
            if(tmpnum<0||(tmpnum>MAX_INFO_LEN&&tmpnum>MAX_STATUS_LEN)) continue;
            if(msgIndex+tmpnum+6>=MAX_MSG_LEN) break;
            uint8_t tmpchecksum;
            tmpchecksum=CalculateChecksum(&zigbeeMessage[msgIndex+6],tmpnum);
            if(tmpchecksum==zigbeeMessage[msgIndex+5])
            {
                if(zigbeeMessage[msgIndex+2]==0x01)
                {
                    //u1_printf("0x01\n");
                    memset(gameInfoMessage,0x00,MAX_INFO_LEN);
                    for(int32_t i=0;i<tmpnum;i++)
                    {
                        gameInfoMessage[i]=zigbeeMessage[msgIndex+6+i];
                        //u1_printf("%2x ",gameInfoMessage[i]);
                    }
                    //u1_printf("\n");
                    continue;
                }
                else if(zigbeeMessage[msgIndex+2]==0x05)
                {
                    //u1_printf("0x05\n");
                    memset(gameStatusMessage,0x00,MAX_STATUS_LEN);
                    for(int32_t i=0;i<tmpnum;i++)
                    {
                        gameStatusMessage[i]=zigbeeMessage[msgIndex+6+i];
                        //u1_printf("%2x ",gameStatusMessage[i]);
                    }
                    //u1_printf("\n");
                    continue;
                }
            }
        }
    }
    memset(zigbeeMessage,0x00,MAX_MSG_LEN);
    receive_flag=0;
    HAL_UARTEx_ReceiveToIdle_DMA(zigbee_huart,zigbeeMessage,MAX_MSG_LEN);
}

int32_t getGameTime()
{
    int32_t time;
    //time=(int32_t)(gameStatusMessage[1]+(gameStatusMessage[2]<<8)+(gameStatusMessage[3]<<16)+(gameStatusMessage[4]<<24));
    time=*((int32_t*)(&gameStatusMessage[1]));
    return time;
}

GameStatus_edc24 getGameStatus()
{
    uint8_t status;
    status=gameStatusMessage[0];
    if(status==0x00)
    {
        return GameStandby;
    }
    else
    {
        return GameGoing;
    }
}

float getScore()
{
    return change_float_data(&gameStatusMessage[5]);
}

Position_edc24 getVehiclePos()
{
    Position_edc24 pos;
    //pos.x=(int32_t)(gameStatusMessage[9]+(gameStatusMessage[10]<<8)+(gameStatusMessage[11]<<16)+(gameStatusMessage[12]<<24));
    //pos.y=(int32_t)(gameStatusMessage[13]+(gameStatusMessage[14]<<8)+(gameStatusMessage[15]<<16)+(gameStatusMessage[16]<<24));
    pos.x=*((int16_t*)(&gameStatusMessage[9]));
    pos.y=*((int16_t*)(&gameStatusMessage[11]));
    return pos;
}

int32_t getRemainDist()
{
    int32_t dist;
    //dist=(int32_t)(gameStatusMessage[17]+(gameStatusMessage[18]<<8)+(gameStatusMessage[19]<<16)+(gameStatusMessage[20]<<24));
    dist=*((int32_t*)(&gameStatusMessage[13]));
    return dist;
}

uint8_t getOrderNum()
{
    return gameStatusMessage[17];
}

Order_edc24 getOneOrder(uint8_t orderNo)
{
    Order_edc24 order;
    //order.depPos.x=(int32_t)(gameStatusMessage[22+28*orderNo]+(gameStatusMessage[23+28*orderNo]<<8)+(gameStatusMessage[24+28*orderNo]<<16)+(gameStatusMessage[25+28*orderNo]<<24));
    //order.depPos.y=(int32_t)(gameStatusMessage[26+28*orderNo]+(gameStatusMessage[27+28*orderNo]<<8)+(gameStatusMessage[28+28*orderNo]<<16)+(gameStatusMessage[29+28*orderNo]<<24));
    //order.desPos.x=(int32_t)(gameStatusMessage[30+28*orderNo]+(gameStatusMessage[31+28*orderNo]<<8)+(gameStatusMessage[32+28*orderNo]<<16)+(gameStatusMessage[33+28*orderNo]<<24));
    //order.desPos.y=(int32_t)(gameStatusMessage[34+28*orderNo]+(gameStatusMessage[35+28*orderNo]<<8)+(gameStatusMessage[36+28*orderNo]<<16)+(gameStatusMessage[37+28*orderNo]<<24));
    order.depPos.x=*((int16_t*)(&gameStatusMessage[18+18*orderNo]));
    order.depPos.y=*((int16_t*)(&gameStatusMessage[20+18*orderNo]));
    order.desPos.x=*((int16_t*)(&gameStatusMessage[22+18*orderNo]));
    order.desPos.y=*((int16_t*)(&gameStatusMessage[24+18*orderNo]));
    //int32_t time;
    //time=(int32_t)(gameStatusMessage[38+28*orderNo]+(gameStatusMessage[39+28*orderNo]<<8)+(gameStatusMessage[40+28*orderNo]<<16)+(gameStatusMessage[41+28*orderNo]<<24));
    order.timeLimit=*((int32_t*)(&gameStatusMessage[26+18*orderNo]));
    order.commission=change_float_data(&gameStatusMessage[30+18*orderNo]);
    //order.orderId=(int32_t)(gameStatusMessage[46+28*orderNo]+(gameStatusMessage[47+28*orderNo]<<8)+(gameStatusMessage[48+28*orderNo]<<16)+(gameStatusMessage[49+28*orderNo]<<24));
    order.orderId=*((int16_t*)(&gameStatusMessage[34+18*orderNo]));
    return order;
}

Order_edc24 getLatestPendingOrder()
{
    int32_t tmpnum=(int32_t)gameStatusMessage[21];
    Order_edc24 order;
    //order.depPos.x=(int32_t)(gameStatusMessage[22+28*tmpnum]+(gameStatusMessage[23+28*tmpnum]<<8)+(gameStatusMessage[24+28*tmpnum]<<16)+(gameStatusMessage[25+28*tmpnum]<<24));
    //order.depPos.y=(int32_t)(gameStatusMessage[26+28*tmpnum]+(gameStatusMessage[27+28*tmpnum]<<8)+(gameStatusMessage[28+28*tmpnum]<<16)+(gameStatusMessage[29+28*tmpnum]<<24));
    //order.desPos.x=(int32_t)(gameStatusMessage[30+28*tmpnum]+(gameStatusMessage[31+28*tmpnum]<<8)+(gameStatusMessage[32+28*tmpnum]<<16)+(gameStatusMessage[33+28*tmpnum]<<24));
    //order.desPos.y=(int32_t)(gameStatusMessage[34+28*tmpnum]+(gameStatusMessage[35+28*tmpnum]<<8)+(gameStatusMessage[36+28*tmpnum]<<16)+(gameStatusMessage[37+28*tmpnum]<<24));
    order.depPos.x=*((int16_t*)(&gameStatusMessage[18+18*tmpnum]));
    order.depPos.y=*((int16_t*)(&gameStatusMessage[20+18*tmpnum]));
    order.desPos.x=*((int16_t*)(&gameStatusMessage[22+18*tmpnum]));
    order.desPos.y=*((int16_t*)(&gameStatusMessage[24+18*tmpnum]));
    //int32_t time;
    //time=(int32_t)(gameStatusMessage[38+28*tmpnum]+(gameStatusMessage[39+28*tmpnum]<<8)+(gameStatusMessage[40+28*tmpnum]<<16)+(gameStatusMessage[41+28*tmpnum]<<24));
    order.timeLimit=*((int32_t*)(&gameStatusMessage[26+18*tmpnum]));
    order.commission=change_float_data(&gameStatusMessage[30+18*tmpnum]);
    //order.orderId=(int32_t)(gameStatusMessage[46+28*tmpnum]+(gameStatusMessage[47+28*tmpnum]<<8)+(gameStatusMessage[48+28*tmpnum]<<16)+(gameStatusMessage[49+28*tmpnum]<<24));
    order.orderId=*((int16_t*)(&gameStatusMessage[34+18*tmpnum]));
    return order;
}

GameStage_edc24 getGameStage()
{
    uint8_t stage;
    stage=gameInfoMessage[0];
    return (GameStage_edc24)stage;
}

Barrier_edc24 getOneBarrier(uint8_t barrierNo)
{
    Barrier_edc24 barrier;
    //barrier.pos_1.x=(int32_t)(gameInfoMessage[2+17*barrierNo]+(gameInfoMessage[3+17*barrierNo]<<8)+(gameInfoMessage[4+17*barrierNo]<<16)+(gameInfoMessage[5+17*barrierNo]<<24));
    //barrier.pos_1.y=(int32_t)(gameInfoMessage[6+17*barrierNo]+(gameInfoMessage[7+17*barrierNo]<<8)+(gameInfoMessage[8+17*barrierNo]<<16)+(gameInfoMessage[9+17*barrierNo]<<24));
    //barrier.pos_2.x=(int32_t)(gameInfoMessage[10+17*barrierNo]+(gameInfoMessage[11+17*barrierNo]<<8)+(gameInfoMessage[12+17*barrierNo]<<16)+(gameInfoMessage[13+17*barrierNo]<<24));
    //barrier.pos_2.y=(int32_t)(gameInfoMessage[14+17*barrierNo]+(gameInfoMessage[15+17*barrierNo]<<8)+(gameInfoMessage[16+17*barrierNo]<<16)+(gameInfoMessage[17+17*barrierNo]<<24));
    barrier.pos_1.x=*((int16_t*)(&gameInfoMessage[2+8*barrierNo]));
    barrier.pos_1.y=*((int16_t*)(&gameInfoMessage[4+8*barrierNo]));
    barrier.pos_2.x=*((int16_t*)(&gameInfoMessage[6+8*barrierNo]));
    barrier.pos_2.y=*((int16_t*)(&gameInfoMessage[8+8*barrierNo]));
    return barrier;
}

int32_t getHalfGameDuration()
{
    int32_t time;
    //time=(int32_t)(gameInfoMessage[86]+(gameInfoMessage[87]<<8)+(gameInfoMessage[88]<<16)+(gameInfoMessage[89]<<24));
    time=*((int32_t*)(&gameInfoMessage[42]));
    return (int32_t)time;
}

uint8_t getOwnChargingPileNum()
{
    return gameInfoMessage[46];
}

uint8_t getOppChargingPileNum()
{
    uint8_t tmp;
    tmp=gameInfoMessage[46];
    return gameInfoMessage[47+4*tmp];
}

Position_edc24 getOneOwnPile(uint8_t pileNo)
{
    Position_edc24 pos;
    //pos.x=(int32_t)(gameInfoMessage[91+8*pileNo]+(gameInfoMessage[92+8*pileNo]<<8)+(gameInfoMessage[93+8*pileNo]<<16)+(gameInfoMessage[94+8*pileNo]<<24));
    //pos.y=(int32_t)(gameInfoMessage[95+8*pileNo]+(gameInfoMessage[96+8*pileNo]<<8)+(gameInfoMessage[97+8*pileNo]<<16)+(gameInfoMessage[98+8*pileNo]<<24));
    pos.x=*((int16_t*)(&gameInfoMessage[47+4*pileNo]));
    pos.y=*((int16_t*)(&gameInfoMessage[49+4*pileNo]));
    return pos;
}

Position_edc24 getOneOppPile(uint8_t pileNo)
{
    uint8_t tmpnum;
    tmpnum=gameInfoMessage[46];
    Position_edc24 pos;
    //pos.x=(int32_t)(gameInfoMessage[92+8*tmpnum+8*pileNo]+(gameInfoMessage[93+8*tmpnum+8*pileNo]<<8)+(gameInfoMessage[94+8*tmpnum+8*pileNo]<<16)+(gameInfoMessage[95+8*tmpnum+8*pileNo]<<24));
    //pos.y=(int32_t)(gameInfoMessage[96+8*tmpnum+8*pileNo]+(gameInfoMessage[97+8*tmpnum+8*pileNo]<<8)+(gameInfoMessage[98+8*tmpnum+8*pileNo]<<16)+(gameInfoMessage[99+8*tmpnum+8*pileNo]<<24));
    pos.x=*((int32_t*)(&gameInfoMessage[48+4*tmpnum+4*pileNo]));
    pos.y=*((int32_t*)(&gameInfoMessage[48+4*tmpnum+4*pileNo]));
    return pos;
}

void reqGameInfo()
{
    HAL_UART_Transmit(zigbee_huart,zigbeeSend[0],6,HAL_MAX_DELAY);
}

void setChargingPile()
{
    HAL_UART_Transmit(zigbee_huart,zigbeeSend[1],6,HAL_MAX_DELAY);
}

/*void zigbee_init_test(UART_HandleTypeDef* huart)
{
    memset(zigbeeMessage_test_1,0x00,MSG_LEN_TEST);
    memset(zigbeeMessage_test_2,0x00,MSG_LEN_TEST);
    memset(gameInfoMessage,0x00,MAX_INFO_LEN);
    memset(gameStatusMessage,0x00,MAX_STATUS_LEN);
    zigbee_huart = huart;
    receive_flag=0;
    last_receive=1;
    HAL_UART_Receive_DMA(zigbee_huart,zigbeeMessage_test_1,MSG_LEN_TEST);
}*/