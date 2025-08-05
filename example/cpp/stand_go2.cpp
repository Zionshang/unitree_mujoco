#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "loop_timer.hpp"
#include <Eigen/Dense>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

double dt = 0.002;

class UnitreeInterface
{
public:
    UnitreeInterface() {};
    ~UnitreeInterface() {};
    void Init();

    unitree_go::msg::dds_::LowCmd_ &low_cmd() { return low_cmd_; }
    const unitree_go::msg::dds_::LowState_ &low_state() const { return low_state_; }

    // [x, y, z, w]
    Eigen::Vector4d quaternion()
    {
        return Eigen::Vector4d(low_state_.imu_state().quaternion()[1],
                               low_state_.imu_state().quaternion()[2],
                               low_state_.imu_state().quaternion()[3],
                               low_state_.imu_state().quaternion()[0]);
    }

    Eigen::Vector3d gyroscope()
    {
        return Eigen::Vector3d(low_state_.imu_state().gyroscope()[0],
                               low_state_.imu_state().gyroscope()[1],
                               low_state_.imu_state().gyroscope()[2]);
    }

    Eigen::Vector3d accelerometer()
    {
        return Eigen::Vector3d(low_state_.imu_state().accelerometer()[0],
                               low_state_.imu_state().accelerometer()[1],
                               low_state_.imu_state().accelerometer()[2]);
    }

    Eigen::VectorXd motor_position()
    {
        Eigen::VectorXd motor_pos(12);
        for (int i = 0; i < 12; i++)
        {
            motor_pos[i] = low_state_.motor_state()[i].q();
        }
        return motor_pos;
    }

    Eigen::VectorXd motor_velocity()
    {
        Eigen::VectorXd motor_vel(12);
        for (int i = 0; i < 12; i++)
        {
            motor_vel[i] = low_state_.motor_state()[i].dq();
        }
        return motor_vel;
    }

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    unitree_go::msg::dds_::LowCmd_ low_cmd_{};     // default init
    unitree_go::msg::dds_::LowState_ low_state_{}; // default init (no change here, just clarifying)

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

class StandController
{
public:
    void update(unitree_go::msg::dds_::LowCmd_ &low_cmd)
    {
        running_time += dt;
        if (running_time < 3.0)
        {
            // Stand up in first 3 second
            // Total time for standing up or standing down is about 1.2s
            phase = tanh(running_time / 1.2);
            for (int i = 0; i < 12; i++)
            {
                low_cmd.motor_cmd()[i].q() = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = phase * 50.0 + (1 - phase) * 20.0;
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
        }
        else
        {
            // Then stand down
            phase = tanh((running_time - 3.0) / 1.2);
            for (int i = 0; i < 12; i++)
            {
                low_cmd.motor_cmd()[i].q() = phase * stand_down_joint_pos[i] + (1 - phase) * stand_up_joint_pos[i];
                low_cmd.motor_cmd()[i].dq() = 0;
                low_cmd.motor_cmd()[i].kp() = 50;
                low_cmd.motor_cmd()[i].kd() = 3.5;
                low_cmd.motor_cmd()[i].tau() = 0;
            }
        }
    }

private:
    double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
                                     0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
    double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
                                       1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
    double running_time = 0.0;
    double phase = 0.0;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void UnitreeInterface::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&UnitreeInterface::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &UnitreeInterface::LowCmdWrite, this);
}

void UnitreeInterface::InitLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd_.motor_cmd()[i].q() = (PosStopF);
        low_cmd_.motor_cmd()[i].kp() = (0);
        low_cmd_.motor_cmd()[i].dq() = (VelStopF);
        low_cmd_.motor_cmd()[i].kd() = (0);
        low_cmd_.motor_cmd()[i].tau() = (0);
    }
}

void UnitreeInterface::LowStateMessageHandler(const void *message)
{
    low_state_ = *(unitree_go::msg::dds_::LowState_ *)message;
}

void UnitreeInterface::LowCmdWrite()
{
    low_cmd_.crc() = crc32_core((uint32_t *)&low_cmd_, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd_);
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    UnitreeInterface unitree_interface;
    unitree_interface.Init();
    StandController stand_controller;
    loop::LoopTimer main_timer(dt);
    while (1)
    {
        main_timer.start();
        stand_controller.update(unitree_interface.low_cmd());
        std::cout << "============================" << std::endl;
        std::cout << "quaternion: " << unitree_interface.quaternion().transpose() << std::endl;
        std::cout << "gyroscope: " << unitree_interface.gyroscope().transpose() << std::endl;
        std::cout << "accelerometer: " << unitree_interface.accelerometer().transpose() << std::endl;
        std::cout << "motor position: " << unitree_interface.motor_position().transpose() << std::endl;
        std::cout << "motor velocity: " << unitree_interface.motor_velocity().transpose() << std::endl;
        main_timer.sleep();
    }

    return 0;
}
