#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"
#include <math.h>
#include "Angle.h"


MI_Motor_ M1_con;

// admittance parament//2-6-1
float Md = 1.7; // 模拟质量，单位：kg
float Ks = 2;  // 劲度系数，单位：N/m
float Dd = 1.2; // 阻尼系数，单位：N*s/m
const float dt = 0.01; // 时间步长，单位：s


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

float velocity = 0.0;


float torque = 1;      // 扭矩 (Nm) [-12, 12]
float position = 0.0;    // 位置 (rad) [-12.5, 12.5]
float speed = 2.0;        // 速度 (rad/s) [-30, 30]
float kp = 0.7;        // 比例增益 [0, 500]0.125
float kd = 0.01;          // 微分增益 [0, 5]0.0158


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




// 控制任务
static void controlTask(void *pvParameters) {


    for (;;) {


        // 更新电机数据
        M1_con.Motor_Data_Updata(5);

        // 循环索引
        if (i < numAngles) i++;
        if (i == numAngles) i = 0;
        float newPosition = angles[i] * (M_PI / 180.0)*20 ; // 期望位置

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
        // Fext = Fn - current_ref; // 外部力等于电机扭矩与期望电流参考值的差  //之前是output-current_ref

        // 计算期望加速度
        ddxd = ((Fext - Ks * (xn - position) - Dd * (dxn - velocity)) / Md) + acc;
        rk4_update(xd, dxd, ddxd, dt);

        // M1_con.Set_PosMode(newPosition,10);
        M1_con.Motor_ControlMode(torque, xd, speed, kp, kd);
        // 打印输出
        Serial.print("d: ");
        Serial.print(xn);
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
        Serial.println(newPosition);
    }
}

void setup() {
    Serial.begin(115200);
    Motor_CAN_Init();
    M1_con.Motor_Con_Init(MOTER_1_ID);
    M1_con.Motor_Set_Zero();
    // delay(100);
    // M1_con.Set_SingleParameter(0x701D, 0);
    //delay(100);
    M1_con.Change_Mode(CTRL_MODE);
    M1_con.Motor_Enable();
//mode 运控模式：CTRL_MODE 电流模式：CUR_MODE 速度模式：SPEED_MODE 位置模式：POS_MODE
    // 创建 FreeRTOS 任务
    xTaskCreatePinnedToCore(controlTask, // 任务入口
                            "ctrlTask",  // 任务名
                            4096,        // 栈大小（Bytes）
                            NULL,        // 参数
                            2,           // 优先级
                            NULL,        // 任务句柄
                            1);          // 运行在 Core 1（ESP32 专用）
}

void loop() {
    // 全部逻辑已切到 controlTask 中
}
