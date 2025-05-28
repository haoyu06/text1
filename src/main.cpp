#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"
#include <math.h> // 引入数学库以使用sin函数
#include "Angle.h"
#include "Current.h"
MI_Motor_ M1_con;

// admittance parament
float Md = 2; // 模拟质量，单位：kg
float Ks = 6;  // 劲度系数，单位：N/m
float Dd = 1; // 阻尼系数，单位：N*s/m
const float dt = 0.01; // 时间步长，单位：s

// PD参数
float Kp = 1.8;  // 比例增益
float Kd = 0.18; // 导数增益
float last_error = 0;
float last_position = 0;
unsigned long previousMillis = 0; // 用于时间控制

int i = 0;
int j = 0;
float xn = 0.0;   // 当前电机位置
float dxn = 0.0;  // 当前电机速度
float xd = 0.0;   // 期望位置
float dxd = 0.0;  // 期望速度
float ddxd = 0.0; // 期望加速度
float Fn = 0.0;
float output = 0.0;
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
  M1_con.Change_Mode(CUR_MODE); // 电流模式
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

  if (j < numCurrent)
  {
    j++;
  }
  if (j == numCurrent)
  {
    j = 0;
  }

  float current_ref = currents[i];
  float newPosition = angles[i] * (M_PI / 180.0) * 20; // 设置伺服电机到指定角度

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

  // 计算外部力 Fext
  Fext = Fn - current_ref; // 外部力等于电机扭矩与期望电流参考值的差  //之前是output-current_ref

  // 计算期望加速度
  ddxd = ((Fext - Ks * (xn - position) - Dd * (dxn - velocity)) / Md) + acc;
  rk4_update(xd, dxd, ddxd, dt);

  // 计算 PD 控制器的输出
  static unsigned long lastTime = 0;
  float current_position = M1_con.motor_rx_data.cur_angle; // 读取当前位置
  float error = xd - current_position;        // 计算位置误差
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 转换为秒
  float derivative = (error - last_error) / deltaTime; // 微分项
  lastTime = currentTime;                              // 更新上次时

  output = Kp * error + Kd * derivative; // PD控制输出
  last_error = error;                          // 更新上一次误差

  M1_con.Set_CurMode(output); // 设置电流模式输出

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
  Serial.print(current_ref);
  Serial.print(",");
  Serial.println(target_position);

  vTaskDelay(5); // 通过插值减缓角度速度
}
