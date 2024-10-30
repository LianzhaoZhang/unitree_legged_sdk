#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

class StandUpController {
public:
    StandUpController(uint8_t level): 
        safe(LeggedType::Go1),
        udp(level, 8090, "192.168.123.10", 8007)
    {
        udp.InitCmdData(cmd);
        if (!initDataRecord()) {
            std::cout << "Init data record failed, EXIT!";
        }
    }

    void UDPRecv();
    void UDPSend();
    void RobotControl();
    bool initDataRecord();
    void recordData();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float dt = 0.002;     // 控制周期2ms
    int motiontime = 0;
    
private:
    // 机器人各腿的初始关节角度
    float init_pos[12] = {0};
    // 站立时的目标关节角度 - 基于Go1的运动学参数调整
    float stand_pos[12] = {
        0.0, 0.67, -1.3,    // 右前腿 (FR)
        0.0, 0.67, -1.3,    // 左前腿 (FL)
        0.0, 0.67, -1.3,    // 右后腿 (RR)
        0.0, 0.67, -1.3     // 左后腿 (RL)
    };
    
    // PD参数
    float kp = 60.0;    // 位置增益
    float kd = 3.0;     // 速度增益

    // 用于轨迹插值的计数器和参数
    int stand_duration = 1000;  // 起立动作持续时间(控制周期数)
    int delay_duration = 1000;   // 延迟等待时间(控制周期数，2s)

    std::chrono::steady_clock::time_point startTime_;
    std::unique_ptr<std::ofstream> footForceReal_;
    
    // 轨迹插值函数
    float linearInterpolation(float init, float target, float rate) {
        rate = std::min(std::max(rate, 0.0f), 1.0f);
        return init * (1 - rate) + target * rate;
    }
};

bool StandUpController::initDataRecord() {
    startTime_ = std::chrono::steady_clock::now();  // 使用chrono记录开始时间
    footForceReal_ = std::make_unique<std::ofstream>("/home/zlz/Downloads/UnitreeTest/unitree_legged_sdk/datasave/foot_force_standup_liedown.csv");
    
    if(footForceReal_ && footForceReal_->is_open()) {
        *footForceReal_ << "Time,RF_Force,LF_Force,RH_Force,LH_Force\n";
        return true;
    }
    return false;
}

void StandUpController::recordData() {
    // 使用chrono计算当前时间
    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsedTime = currentTime - startTime_;
    
    if(footForceReal_ && footForceReal_->is_open()) {
        *footForceReal_ << elapsedTime.count() << ","
                       << state.footForce[0] << ","
                       << state.footForce[1] << ","
                       << state.footForce[2] << ","
                       << state.footForce[3] << "\n";
    }
}

void StandUpController::UDPRecv() {
    udp.Recv();
}

void StandUpController::UDPSend() {
    udp.Send();
}

void StandUpController::RobotControl() {
    motiontime++;
    udp.GetRecv(state);

    recordData();

    // 第一个周期，记录初始位置
    if (motiontime <= 10) {
        for(int i = 0; i < 12; i++) {
            init_pos[i] = state.motorState[i].q;
        }
        return;
    }

    // 延迟等待阶段：保持初始位置
    if (motiontime > 10 && motiontime <= delay_duration + 10) {
        for(int i = 0; i < 12; i++) {
            cmd.motorCmd[i].q = init_pos[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = kp;
            cmd.motorCmd[i].Kd = kd;
        }
        if (motiontime == delay_duration + 10) {
            std::cout << "Starting stand up motion..." << std::endl;
        }
        return;
    }

    // 执行站立动作
    if (motiontime > delay_duration + 10 && motiontime <= stand_duration + delay_duration + 10) {
        float rate = (float)(motiontime - delay_duration - 10) / stand_duration;
        
        // 为所有关节计算插值位置
        for(int i = 0; i < 12; i++) {
            float q_des = linearInterpolation(init_pos[i], stand_pos[i], rate);
            
            cmd.motorCmd[i].q = q_des;
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = kp;
            cmd.motorCmd[i].Kd = kd;
            
            // 设置前髋关节的额外重力补偿
            if(i == 0 || i == 3) {  // FR_0, FL_0
                cmd.motorCmd[i].tau = -0.65f;
            }
        }
    }
    
    // 完成站立后保持姿态
    if (motiontime > stand_duration + delay_duration + 10) {
        for(int i = 0; i < 12; i++) {
            cmd.motorCmd[i].q = stand_pos[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = kp;
            cmd.motorCmd[i].Kd = kd;
            
            if(i == 0 || i == 3) {  // FR_0, FL_0
                cmd.motorCmd[i].tau = -0.65f;
            }
        }
    }

    // 安全检查
    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, 1);
    
    udp.SetSend(cmd);
}

int main() {
    std::cout << "Stand-up control program for Go1" << std::endl
              << "Make sure the robot is on the ground!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    StandUpController controller(LOWLEVEL);
    
    LoopFunc loop_control("control_loop", controller.dt, 
                         boost::bind(&StandUpController::RobotControl, &controller));
    LoopFunc loop_udpSend("udp_send", controller.dt, 3,
                         boost::bind(&StandUpController::UDPSend, &controller));
    LoopFunc loop_udpRecv("udp_recv", controller.dt, 3,
                         boost::bind(&StandUpController::UDPRecv, &controller));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1) {
        sleep(10);
    };

    return 0;
}