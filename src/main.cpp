#include <Arduino.h>
#include "TWAI_CAN_MI_Motor.h"
#include <math.h>  // 引入数学库以使用sin函数
MI_Motor_ M1_con;

const float amplitude = 120.0f;  // 正弦波幅度
const float frequency = 0.2f;    // 正弦波频率（调节速度）
float Kp = 0.08;                // 比例增益
float Kd = 0.08;                 // 导数增益
float last_error = 0;
float last_position = 0;
unsigned long previousMillis = 0;  // 用于时间控制

void setup() {
  // 初始化串口通信和电机
  Serial.begin(115200);

  Motor_CAN_Init();
  M1_con.Motor_Con_Init(MOTER_1_ID);
  M1_con.Motor_Set_Zero();
  M1_con.Change_Mode(CUR_MODE);  //运控模式：CTRL_MODE 电流模式：CUR_MODE 速度模式：SPEED_MODE 位置模式：POS_MODE
  M1_con.Motor_Enable();
}

void loop() {
  // 电机数据更新
  M1_con.Motor_Data_Updata(20);
  unsigned long currentMillis = millis();
  float timeInSeconds = currentMillis / 1000.0;     // 将毫秒转换为秒
  float time = 2 * PI * frequency * timeInSeconds;  // 时间因子，现在是以弧度为单位

  // 正弦波计算
  float sine_value = amplitude * sin(time);           // 计算正弦波值
  float target_position = sine_value * (PI / 180.0);  // 转换为弧度

  // 位置控制
  static unsigned long lastTime = 0;
  float current_position = M1_con.motor_rx_data.cur_angle;  // 读取当前位置
  float error = target_position - current_position;         // 计算位置误差
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // 转换为秒
  float derivative = (error - last_error) / deltaTime;  // 用实际时间计算微分项
  lastTime = currentTime;                               // 更新上次时

  float output = Kp * error + Kd * derivative;  // PD控制输出
  last_error = error;                           // 更新上一次误差

  M1_con.Set_CurMode(output);  // 发送电流指令

  // 数据打印
  Serial.print("d: ");
  Serial.print(M1_con.motor_rx_data.cur_angle);
  Serial.print(",");
  Serial.println(target_position);

  // 检查串口输入以更新 Kp 和 Kd
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // 读取一行串口输入
    // input.trim();  // 移除首尾空格
    if (input.startsWith("r:")) {
      // 提取参数并解析
      int splitIndex = input.indexOf(':') + 1;
      String values = input.substring(splitIndex);
      int commaIndex = values.indexOf(',');

      if (commaIndex != -1) {
        String kpValue = values.substring(0, commaIndex);
        String kdValue = values.substring(commaIndex + 1);

        // 转换为浮点数并赋值
        Kp = kpValue.toFloat();
        Kd = kdValue.toFloat();

        // 打印更新确认
        Serial.print("Updated Kp: ");
        Serial.println(Kp);
        Serial.print("Updated Kd: ");
        Serial.println(Kd);
      }
    }
  }

  vTaskDelay(50);
}
