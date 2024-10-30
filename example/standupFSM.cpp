#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <termios.h>
#include <fcntl.h>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono;

// 改进的非阻塞键盘输入函数
class KeyboardInput {
public:
    KeyboardInput() {
        tcgetattr(STDIN_FILENO, &oldSettings);
        newSettings = oldSettings;
        
        newSettings.c_lflag &= ~(ICANON | ECHO);
        newSettings.c_cc[VMIN] = 0;
        newSettings.c_cc[VTIME] = 0;
        
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    ~KeyboardInput() {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
    }
    
    char getInput() {
        char buf = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
        if (read(STDIN_FILENO, &buf, 1) < 0) {
            return 0;
        }
        return buf;
    }
    
private:
    struct termios oldSettings, newSettings;
};

class StandUpController {
public:
    StandUpController(uint8_t level): safe(LeggedType::Go1),
                                     udp(level, 8090, "192.168.123.10", 8007),
                                     keyInput()
    {
        udp.InitCmdData(cmd);
        if (!initDataRecord()) {
            std::cout << "Init data record failed, EXIT!" << std::endl;
            exit(1);
        }
        std::cout << "Init StandUpController finished." << std::endl;
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
    KeyboardInput keyInput;

    // 数据记录相关
    std::chrono::steady_clock::time_point startTime_;
    std::unique_ptr<std::ofstream> footForceReal_;

    float init_pos[12] = {0};
    float stand_pos[12] = {
        0.0, 0.67, -1.3,    // 右前腿 (FR)
        0.0, 0.67, -1.3,    // 左前腿 (FL)
        0.0, 0.67, -1.3,    // 右后腿 (RR)
        0.0, 0.67, -1.3     // 左后腿 (RL)
    };
    
    float kp = 60.0;
    float kd = 3.0;

    // 添加延迟等待时间
    int delay_duration = 1000;   // 延迟等待时间(控制周期数，2s)
    int standup_duration = 800;  // 站起持续时间
    int liedown_duration = 800;  // 趴下持续时间

    enum MotionState {
        INIT,           // 初始状态
        WAITING,        // 新增：等待状态
        STANDING_UP,    // 站起状态
        STANDING,       // 站立状态
        LYING_DOWN      // 趴下状态
    } currentState = INIT;
    
    float linearInterpolation(float init, float target, float rate) {
        rate = std::min(std::max(rate, 0.0f), 1.0f);
        return init * (1 - rate) + target * rate;
    }
};

bool StandUpController::initDataRecord() {
    startTime_ = std::chrono::steady_clock::now();
    footForceReal_ = std::make_unique<std::ofstream>("/home/zlz/Downloads/UnitreeTest/unitree_legged_sdk/datasave/foot_force_standup_fsm.csv");
    
    if(footForceReal_ && footForceReal_->is_open()) {
        *footForceReal_ << "Time,RF_Force,LF_Force,RH_Force,LH_Force,State\n";
        return true;
    }
    return false;
}

void StandUpController::recordData() {
    auto currentTime = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsedTime = currentTime - startTime_;
    
    if(footForceReal_ && footForceReal_->is_open()) {
        *footForceReal_ << elapsedTime.count() << ","
                       << state.footForce[0] << ","
                       << state.footForce[1] << ","
                       << state.footForce[2] << ","
                       << state.footForce[3] << ","
                       << static_cast<int>(currentState) << "\n";
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

    // 记录数据
    recordData();
    
    char c = keyInput.getInput();
    if(c == 'q' && currentState == STANDING) {
        currentState = LYING_DOWN;
        motiontime = 0;
        std::cout << "Changing to LYING_DOWN state." << std::endl;
    }
    
    // 初始化状态：记录初始位置
    if (currentState == INIT) {
        if (motiontime <= 10) {
            for(int i = 0; i < 12; i++) {
                init_pos[i] = state.motorState[i].q;
            }
        } else {
            currentState = WAITING;  // 转换到等待状态
            motiontime = 0;
            std::cout << "Change to WAITING State." << std::endl;
        }
        return;
    }

    // 等待状态：保持初始位置一段时间
    if (currentState == WAITING) {
        for(int i = 0; i < 12; i++) {
            cmd.motorCmd[i].q = init_pos[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = kp;
            cmd.motorCmd[i].Kd = kd;
        }
        
        if (motiontime >= delay_duration) {
            currentState = STANDING_UP;
            motiontime = 0;
            std::cout << "Change to STANDING_UP State." << std::endl;
        }
        return;
    }

    // 站起状态
    if (currentState == STANDING_UP) {
        float rate = (float)motiontime / standup_duration;
        
        if (motiontime <= standup_duration) {
            for(int i = 0; i < 12; i++) {
                float q_des = linearInterpolation(init_pos[i], stand_pos[i], rate);
                
                cmd.motorCmd[i].q = q_des;
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].Kp = kp;
                cmd.motorCmd[i].Kd = kd;
                
                if(i == 0 || i == 3) {
                    cmd.motorCmd[i].tau = -0.65f;
                }
            }
        } else {
            currentState = STANDING;
            std::cout << "Change to STANDING State." << std::endl;
        }
    }
    
    // 站立状态
    if (currentState == STANDING) {
        for(int i = 0; i < 12; i++) {
            cmd.motorCmd[i].q = stand_pos[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = kp;
            cmd.motorCmd[i].Kd = kd;
            
            if(i == 0 || i == 3) {
                cmd.motorCmd[i].tau = -0.65f;
            }
        }
    }

    // 趴下状态
    if (currentState == LYING_DOWN) {
        float rate = (float)motiontime / liedown_duration;
        
        if (motiontime <= liedown_duration) {
            for(int i = 0; i < 12; i++) {
                float q_des = linearInterpolation(stand_pos[i], init_pos[i], rate);
                
                cmd.motorCmd[i].q = q_des;
                cmd.motorCmd[i].dq = 0;
                cmd.motorCmd[i].Kp = kp;
                cmd.motorCmd[i].Kd = kd;
                
                if(i == 0 || i == 3) {
                    cmd.motorCmd[i].tau = -0.65f;
                }
            }
        }
    }

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