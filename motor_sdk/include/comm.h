#ifndef COMM_H
#define COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**********************Define**************************/
#define cmd_size           (23U)    // sizeof(MotorCmd)
#define sta_size           (21U)    // sizeof(MotorState)=IMU_EQU_NUM*sizeof(ImuState)
#define IMU_EQU_NUM        (2U)     // IMU_EQU_NUM = sizeof(ImuState)/sizeof(MotorState), equivalent 
#define BAT_EQU_NUM        (1U)     // BAT_EQU_NUM = sizeof(BatState)/sizeof(MotorState)

/**********************Struct**************************/
/** 
  * @brief  serial port Status structures definition  
  */  
typedef enum{
  SP_Default        = 0x00U,
  SP_SendTimeout    = 0x01U,
  SP_SendLenError   = 0x02U,
  SP_SendIDError    = 0x03U,
  SP_RecvTimeout    = 0x04U,
  SP_RecvLenError   = 0x05U,
  SP_RecvCRCError   = 0x06U,
  SP_RecvMotorTimeout = 0x07U,
  SP_SendRecvOK     = 0x09U,
} SerialPortStatus;

/* imu status */
typedef enum{
  IMU_Default       = 0x00U,
  IMU_BaundError    = 0x01U,
  IMU_SendError     = 0x02U,
  IMU_RecvError     = 0x03U,
  IMU_RecvAccError  = 0x04U,
  IMU_RecvGyroError = 0x05U,
  IMU_RecvQuatError = 0x06U,
  IMU_SendRecvOK    = 0x09U,
} ImuStatus;

/* battery status */
typedef enum{
  Bat_Default       = 0x00U,
  Bat_RecvCRCError  = 0x01U,
  Bat_RecvTimeout   = 0x02U,
  Bat_SendRecvOK    = 0x09U,
} BatStatus;

/* communication type */
typedef enum{
    USB             = 0x00U,
    UDP             = 0x01U
}CommType;

/* motor type */
typedef enum{
  LZ17              = 0x03U,
  LZ120             = 0x04U,
  J60               = 0x05U,
  First             = LZ17,
  Last              = J60
}MotorType;

#pragma pack(1)

typedef struct{
    uint8_t cStatus;	// communication status. 0:recv OK 1:recv timeout 2:recv error 3??send error
    uint8_t id;       // motor id
    uint8_t type;     // motor type
    uint8_t mStatus;	// motor status. refer to MI_motor_dev.h/motor_state_e struct and message_motor/MOTOR_recv struct
    uint8_t mode;     // motor working mode, refer to MotorCmd:mode below
    float q;          // current angle (unit: radian)
    float dq;         // current velocity (unit: radian/second)
    float tauEst;     // current estimated output torque (unit: N.m)
    float temp;       // motor temprature
} MotorState;       // motor feedback
// int a = sizeof(MotorState);   // 21

typedef struct{
    uint8_t id;     // motor id
    uint8_t type;   // motor type
    uint8_t mode;   // desired working mode. 0:passive 10:MIT
    float q;        // desired angle (unit: radian)
    float dq;       // desired velocity (unit: radian/second)
    float tau;      // desired output torque (unit: N.m)
    float kp;       // desired position stiffness (unit: N.m/rad )
    float kd;       // desired velocity stiffness (unit: N.m/(rad/s) )
} MotorCmd;       // motor control
// int b = sizeof(MotorCmd);     // 23

typedef struct{
    uint8_t cStatus;
    uint8_t id;
    float acc[3];
    float gyro[3];
    float quat[4];
} ImuState;
// int c = sizeof(ImuState);   // 42

typedef struct{
    uint8_t cStatus;	// communication status. 0:recv OK 1:recv timeout 2:recv error 3??send error
    uint8_t bms_id;   // BMS id
    uint32_t failure; // error status
    float cum_vol;    // Cumulative voltage
    float current;    // current
    float soc;        // SOC
    uint8_t res[3];   // reserved
} BatState;
// int c = sizeof(BatState);   // 21

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif
