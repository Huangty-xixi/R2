
#include "pid.h"

/**
 * @brief 初始化 PID 参数.
 * @param pid:  指向一个包含 PID 控制器相关信息的 PID_Info_TypeDef 结构体的指针
 * @param para: 指向一个包含 PID 控制器参数的浮点型数组的指针。
 * @retval pid 错误状态
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *Pid,float para[PID_PARAMETER_NUM])
{
    /* judge the pointer of PID Parameters */
    if(Pid->type == PID_Type_None || para == NULL)
    {
      return PID_FAILED_INIT;
    }

    /* 初始化 pid 参数 ------------------*/
    Pid->param.kp = para[0];
    Pid->param.ki = para[1];
    Pid->param.kd = para[2];
    Pid->param.Deadband = para[3];
    Pid->param.limitIntegral = para[4];
    Pid->param.limitOutput = para[5];

    /* clear the pid error judgement count */
    Pid->ERRORHandler.ErrorCount = 0;

    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief Clear the Pid Calculation.
 * @param pid: pointer to a PID_Info_TypeDef structure that
 *         contains the information for the PID controller.
 * @retval 无
 */
static void PID_Calc_Clear(PID_Info_TypeDef *Pid)
{
	memset(Pid->Err,0,sizeof(Pid->Err));
	Pid->Integral = 0;
		
	Pid->Pout = 0;
	Pid->Iout = 0;
	Pid->Dout = 0;
	Pid->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief 初始化 PID 控制器
 * @param pid: 指向一个包含 PID 控制器相关信息的 PID_Info_TypeDef 结构体的指针
 * @param type: PID 控制器的类型
 * @param para: pointer to a floating-point array that
 *         contains the parameters for the PID controller.
 * @retval PID 错误状态
 */
void PID_Init(PID_Info_TypeDef *Pid,PID_Type_e type,float para[PID_PARAMETER_NUM])
{
		Pid->type = type;

		Pid->PID_Calc_Clear = PID_Calc_Clear;
    Pid->PID_Param_Init = PID_Param_Init;

		Pid->PID_Calc_Clear(Pid);
    Pid->ERRORHandler.Status = Pid->PID_Param_Init(Pid, para);
}
//------------------------------------------------------------------------------


/**
  * @brief  判断 PID 错误状态
  * @param pid: 指向一个包含 PID 控制器相关信息的 PID_Info_TypeDef 结构体的指针.
  * @retval 无
  */
static void PID_ErrorHandle(PID_Info_TypeDef *Pid)
{
		/* Judge NAN/INF */
		if(isnan(Pid->Output) == true || isinf(Pid->Output)==true)
		{
				Pid->ERRORHandler.Status = PID_CALC_NANINF;
		}
}
//------------------------------------------------------------------------------

/**
  * @brief  计算 PID 控制器值
  * @param  *pid 指向一个包含 PID 控制器相关信息的 PID_Info_TypeDef 结构体的指针 
  * @param  Target  Pid 控制器的目标值
  * @param  Measure Pid 控制器的测量值
  * @retval PID 输出
  */
float f_PID_Calculate(PID_Info_TypeDef *Pid, float target,float measure)
{		
  /* update the pid error status */
  PID_ErrorHandle(Pid);
  if(Pid->ERRORHandler.Status != PID_ERROR_NONE)
  {
    Pid->PID_Calc_Clear(Pid);
    return 0;
  }
  
  /* update the target/measure */
  Pid->target = target;
  Pid->measure = measure;

  /* update the error */
	Pid->Err[2] = Pid->Err[1];
	Pid->Err[1] = Pid->Err[0];
	Pid->Err[0] = Pid->target - Pid->measure;
		
  if(fabsf(Pid->Err[0]) >= Pid->param.Deadband)
  {
		/* 更新 PID 控制器输出 */
		if(Pid->type == PID_POSITION)
		{
      /* 更新 PID 积分 */
      if(Pid->param.ki != 0)
        Pid->Integral += Pid->Err[0];
      else
        Pid->Integral = 0;

      VAL_LIMIT(Pid->Integral,-Pid->param.limitIntegral,Pid->param.limitIntegral);
      
      /* Update the Proportional Output,Integral Output,Derivative Output */
      Pid->Pout = Pid->param.kp * Pid->Err[0];
      Pid->Iout = Pid->param.ki * Pid->Integral;
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - Pid->Err[1]);
      
      /* update the PID output */
      Pid->Output = Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
		else if(Pid->type == PID_VELOCITY)
		{
      /* Update the Proportional Output,Integral Output,Derivative Output */
      Pid->Pout = Pid->param.kp * (Pid->Err[0] - Pid->Err[1]);
      Pid->Iout = Pid->param.ki * (Pid->Err[0]);
      Pid->Dout = Pid->param.kd * (Pid->Err[0] - 2.f*Pid->Err[1] + Pid->Err[2]);

      /* update the PID output */
      Pid->Output += Pid->Pout + Pid->Iout + Pid->Dout;
      VAL_LIMIT(Pid->Output,-Pid->param.limitOutput,Pid->param.limitOutput);
		}
  }

  return Pid->Output;
}
//------------------------------------------------------------------------------

