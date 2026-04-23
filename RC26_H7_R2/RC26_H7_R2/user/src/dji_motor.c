#include "dji_motor.h"
#include "math.h"
#include "cmsis_os.h"

uint8_t text[8];
HAL_StatusTypeDef flag3=HAL_ERROR;

/*******************************************************************************************
  * @Func		float Motor_PID_Calculate(DJI_MotorModule *obj, float input)
  * @Brief      计算pid
  * @Param		obj         dji电机类
  * @Param		input       输入量
  * @Retval		None
  * @Date     2025/12/26
 *******************************************************************************************/
float Motor_PID_Calculate(DJI_MotorModule *obj, float input)
{
	
	static float output;
	
	switch (obj->super_motor.mode){
		case POSITION :
			output = f_PID_Calculate(&obj->pid_spd, f_PID_Calculate(&obj->pid_pos, input, obj->total_angle), obj->speed_rpm);
            break;
		case SPEED :
			output = f_PID_Calculate(&obj->pid_spd, input, obj->speed_rpm);
			break;
        case MIT:
            break;
	}

	return output;
}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_info_t *moto_info, uint8_t rx_data)
  * @Brief          接收电机通过CAN发过来的信息
  * @Param		    
  * @Retval		    None
  * @Date           2025/12/26
 *******************************************************************************************/
void DJIget_motor_measure(DJI_MotorModule *obj, uint8_t rx_data[8])
{
	switch (obj->super_motor.model){
		
		case DJI_4in1:
		case DJI_6020:
		case DJI_3508:
			obj->temp = rx_data[6];
		case DJI_2006:
			obj->last_angle = obj->angle;
			obj->angle = (uint16_t)(rx_data[0]<<8 | rx_data[1]) ;
			obj->speed_rpm  = (int16_t)(rx_data[2]<<8 | rx_data[3]);
			obj->real_current = (int16_t)(rx_data[4]<<8 | rx_data[5])/-5;
			
			if(obj->angle - obj->last_angle > 4096)
				obj->round_cnt --;
			else if (obj->angle - obj->last_angle < -4096)
				obj->round_cnt ++;
				obj->total_angle = obj->round_cnt * 8192 + obj->angle - obj->offset_angle;
	
			break;
         default:
			break;
	}
}

/*this function should be called after system+can init */
void DJIget_moto_offset(DJI_MotorModule *obj, uint8_t rx_data[8])
{
	obj->angle = (uint16_t)(rx_data[0]<<8 | rx_data[1]) ;
	obj->offset_angle = obj->angle;
}

/*******************************************************************************************
  * @Func		set_motor_data(CAN_HandleTypeDef* hcan, uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4)
	* @Brief    控制大疆电机的电压/电流
  * @Param		标准ID
  * @Retval		None 
  * @Date     2024/12/26
 *******************************************************************************************/
HAL_StatusTypeDef DJIset_motor_data(FDCAN_HandleTypeDef* hcan, uint32_t StdId, int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
//	HAL_StatusTypeDef flag;
	FDCAN_TxHeaderTypeDef tx_header;
	uint8_t             tx_data[8];

	  tx_header.Identifier = StdId;
  	tx_header.IdType= FDCAN_STANDARD_ID;
	  tx_header.TxFrameType= FDCAN_DATA_FRAME;
	  tx_header.DataLength= 8;
    tx_header.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch=FDCAN_BRS_OFF;
    tx_header.FDFormat=FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker=0;

	tx_data[0] = (data1>>8)&0xff;
	tx_data[1] =    (data1)&0xff;
	tx_data[2] = (data2>>8)&0xff;
	tx_data[3] =    (data2)&0xff;
	tx_data[4] = (data3>>8)&0xff;
	tx_data[5] =    (data3)&0xff;
	tx_data[6] = (data4>>8)&0xff;
	tx_data[7] =    (data4)&0xff;
	
//	text[0]=tx_data[0];
//	text[1]=tx_data[1];
//	text[2]=tx_data[2];
//	text[3]=tx_data[3];
//	text[4]=tx_data[4];
//	text[5]=tx_data[5];
//	text[6]=tx_data[6];
//	text[7]=tx_data[7];
	
	flag3 = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, tx_data); //

}

/*******************************************************************************************
  * @Func	    void DJImotor_Create
  * @Brief      注册电机函数
  * @Param		obj：            dji电机类
  * @Param		command_id：     控制id
  * @Param      feedback_id ：   反馈Id
  * @Param      hcan             挂载总线
  * @Param      motorModel       电机型号
  * @Param      mode             控制模式
  * @Param      pidType          PID模式
  * @Param      pid_Param        PID参数
  * @Retval		None
  * @Date       2025/12/26
 *******************************************************************************************/
void DJImotor_Create(DJI_MotorModule *obj, 
                     uint16_t command_id, 
                     uint16_t feedback_id,
                     FDCAN_HandleTypeDef *hcan, 
                     Motor_Model motorModel,
                     Ctrl_mode mode,
                     PID_Type_e pidType, 
                     float pid_Param[PID_PARAMETER_NUM]) 
{
    
    MotorModule_Create(&obj->super_motor, feedback_id - 0x200, hcan, motorModel, mode);
    
    obj->CAN_FEEDBACK_ID = feedback_id;
    
    obj->get_moto_measure = DJIget_motor_measure;
    obj->get_moto_offset = DJIget_moto_offset;
    obj->PID_Calculate = Motor_PID_Calculate;
    
    obj->speed_rpm = 0;
    obj->real_current = 0;
    obj->given_current = 0;
    obj->temp = 0;
    obj->angle = 0;				//abs 
    obj->last_angle = 0;		//abs angl
    obj->offset_angle = 0;
    obj->round_cnt = 0;
    obj->total_angle = 0;
    obj->buf_idx = 0;
    obj->fited_angle = 0;
    obj->msg_cnt = 0;
    memset(obj->angle_buf, 0, sizeof(obj->angle_buf));
    
    switch (obj->super_motor.mode){ 
		case POSITION :
			PID_Init(&obj->pid_pos, pidType, pid_Param);
            break;
		case SPEED :
			PID_Init(&obj->pid_spd, pidType, pid_Param);
			break;
        case MIT:
            break;
	}
}

