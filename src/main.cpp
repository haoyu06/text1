// Fext 和output差了可能4ms，也就是说滞后4s的output可以用来做dob来去除fext的外力影响
////阻抗的xd，desire，xn三线合一，无敌 但是为什么我给力他没效果，2024年12月12日，后续调整刚度



#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"
#include <math.h> // 引入数学库以使用sin函数
#include "Angle.h"
MI_Motor_ M1_con;

// admittance parament
float Md = 6; // 模拟质量，单位：kg   //增大加速度的敏感程度//6,15,1.8;///7,14,2往下走//6，8，2的下降程度变缓了
float Ks = 6;  // 劲度系数，单位：N/m
float Dd = 2; // 阻尼系数，单位：N*s/m  //增大则加速度去噪,角度变化圆滑
// float Kp = 1.0;                   // 单位：N/m
const float dt = 0.01; // 时间步长，单位：s

// pd parameter
// const float amplitude = 120.0f;  // 正弦波幅度
// const float frequency = 0.2f;    // 正弦波频率（调节速度）
float Kp = 1.7;  // 比例增益//1.7，0.2有点延迟和过冲,但能接受
float Kd = 0.2; // 导数增益
float last_error = 0;
float last_position = 0;
unsigned long previousMillis = 0; // 用于时间控制

int i = 0;
float xn = 0.0;   // 当前电机位置
float dxn = 0.0;  // 当前电机速度
float xd = 0.0;   // 期望位置
float dxd = 0.0;  // 期望速度
float ddxd = 0.0; // 期望加速度
float Fn = 0.0;
float output_delay1= 0.0;
float Fext = 0.0; // 外部力
float position = 0.0;
float velocity = 0.0;

void rk4_update(float &xd, float &dxd, float ddxd, float dt)
{
  float k1_v = ddxd;
  float k1_p = dxd;

  float k2_v = ddxd + 0.5 * dt * k1_v;
  float k2_p = dxd + 0.5 * dt * k1_p;

  float k3_v = ddxd + 0.5 * dt * k2_v;
  float k3_p = dxd + 0.5 * dt * k2_p;

  float k4_v = ddxd + dt * k3_v;
  float k4_p = dxd + dt * k3_p;

  // 组合四步的结果更新速度和位置
  dxd += (dt / 6.0) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v);
  xd += (dt / 6.0) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
}

void setup()
{
  Serial.begin(115200);
  Motor_CAN_Init();
  M1_con.Motor_Con_Init(MOTER_1_ID);
  M1_con.Motor_Set_Zero();
  M1_con.Change_Mode(CUR_MODE); // 运控模式：CTRL_MODE 电流模式：CUR_MODE 速度模式：SPEED_MODE 位置模式：POS_MODE
  M1_con.Motor_Enable();
}

void loop()
{
  // 电机数据更新
  M1_con.Motor_Data_Updata(20);

  if (i < numAngles)
  {
    i++;
  }
  if (i == numAngles)
  {
    i = 0;
  }
  float newPosition = angles[i] * (M_PI / 180.0) * 2; // 设置伺服电机到指定角度
  newPosition = newPosition * 10;

  // 计算位移变化
  float dx = newPosition - position; // 计算位移变化量
  position = newPosition;            // 更新当前位置
  float target_position = position;

  // 计算速度
  float newvelocity = dx / dt; // 计算速度
  float dv = newvelocity - velocity;
  velocity = newvelocity;
  // 计算加速度
  float acc = dv / dt;

  xn = M1_con.motor_rx_data.cur_angle;  // 获取当前电机位置
  dxn = M1_con.motor_rx_data.cur_speed; // 获取当前电机速度
  Fn = M1_con.motor_rx_data.cur_torque; // 获取当前电机扭矩
  
  Fext = 0.5 *output_delay1 - Fn; // 
  //output_delay1 = output;

  ddxd = ((Fext - Ks * (xn - position) - Dd * (dxn - velocity)) / Md) + acc; // 期望加速度计算Dd * (dxn - velocity)
  // ddxd = ((0 - Ks * (xn - position) - Dd * dxn) / Md);
  
  rk4_update(xd, dxd, ddxd, dt);


  static unsigned long lastTime = 0;
  float current_position = M1_con.motor_rx_data.cur_angle; // 读取当前位置
  float error = xd - current_position;        // 计算位置误差
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 转换为秒
  float derivative = (error - last_error) / deltaTime; // 用实际时间计算微分项
  lastTime = currentTime;                              // 更新上次时

  float output = Kp * error + Kd * derivative; // PD控制输出
  last_error = error;                          // 更新上一次误差

  //M1_con.Set_CurMode(output); // 发送电流指令///



  // dxd += ddxd * dt;  // 更新期望速度
  // xd += dxd * dt;    // 更新期望位置


  M1_con.Set_CurMode(output);//0.1* xd

  // 数据打印
  Serial.print("d: ");
  Serial.print(xn);
  Serial.print(",");
  Serial.print(ddxd);
  Serial.print(",");
  Serial.print(dxd);
  Serial.print(",");
  Serial.print(xd);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(Fn);
  Serial.print(",");
  Serial.print(Fext);
  Serial.print(",");
  Serial.println(target_position);

  // 检查串口输入以更新 Kp 和 Kd
  // if (Serial.available())
  // {
  //   String input = Serial.readStringUntil('\n'); // 读取一行串口输入
  //   // input.trim();  // 移除首尾空格

  //   if (input.startsWith("r:"))
  //   {
  //     // 提取参数并解析
  //     int splitIndex = input.indexOf(':') + 1;
  //     String values = input.substring(splitIndex);
  //     int commaIndex = values.indexOf(',');

  //     if (commaIndex != -1)
  //     {
  //       // 提取 Kp 和 Kd
  //       String kpValue = values.substring(0, commaIndex);
  //       String kdValue = values.substring(commaIndex + 1);

  //       // 查找下一个逗号，解析 Md、Ks 和 Dd
  //       int secondCommaIndex = values.indexOf(',', commaIndex + 1);
  //       String mdValue = values.substring(commaIndex + 1, secondCommaIndex);
  //       String ksValue = values.substring(secondCommaIndex + 1, values.indexOf(',', secondCommaIndex + 1));
  //       String ddValue = values.substring(values.lastIndexOf(',') + 1);

  //       // 转换为浮点数并赋值
  //       Kp = kpValue.toFloat();
  //       Kd = kdValue.toFloat();
  //       Md = mdValue.toFloat();
  //       Ks = ksValue.toFloat();
  //       Dd = ddValue.toFloat();

  //       // 打印更新确认
  //       Serial.print("Updated Kp: ");
  //       Serial.println(Kp);
  //       Serial.print("Updated Kd: ");
  //       Serial.println(Kd);
  //       Serial.print("Updated Md: ");
  //       Serial.println(Md);
  //       Serial.print("Updated Ks: ");
  //       Serial.println(Ks);
  //       Serial.print("Updated Dd: ");
  //       Serial.println(Dd);
  //     }
  //   }
  // }


vTaskDelay(5); // 5的时候速度很快，考虑通过插值减缓angle速度
}
