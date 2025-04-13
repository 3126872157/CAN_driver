#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

#include <cstdint>

#define message_data struct __attribute__((packed))

message_data aim_data_t {
    uint8_t fire_advice;
    float pitch;
    float yaw;
    float distance;
    uint8_t id;
};

message_data gimbal_data_t {
    uint8_t mode;
    float pitch;
    float roll;
    float yaw;
};

message_data gimbal_401_t{
    float roll;
    float pitch;
};

message_data gimbal_402_t{
    float yaw;
    uint8_t mode;
    uint8_t padding[3]={};
};

#endif

