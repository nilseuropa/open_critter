#ifndef _OPEN_CRITTER_DATAGRAM_PROTOCOL_HEADER_
#define _OPEN_CRITTER_DATAGRAM_PROTOCOL_HEADER_

#if PRAGMA_PACK_AVAILABLE
#define __PACKED
#pragma pack(push, 1)
#else
#define __PACKED __attribute__((packed))
#endif

#define PID 0
//
#define PID_SET_SERVO      8
#define PID_SET_ALL_SERVOS 16

enum jointOrder {
  FL_Coxa,
  FL_Femur,
  FL_Tibia,
  FR_Coxa,
  FR_Femur,
  FR_Tibia,
  RR_Coxa,
  RR_Femur,
  RR_Tibia,
  RL_Coxa,
  RL_Femur,
  RL_Tibia
};

typedef struct {
  uint8_t  packetId;
	uint8_t  angles[16];
} __PACKED RobotStatePacket;

typedef struct {
  uint8_t  packetId;
  uint8_t  jointId;
	uint8_t  angle;
} __PACKED ServoStatePacket;

#endif
