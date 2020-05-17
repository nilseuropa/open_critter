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
} __PACKED Header;

typedef struct {
  Header   header;
	double   states[16];
} __PACKED RobotStatePacket;

typedef struct {
  Header   header;
  uint8_t  jointId;
  uint8_t  jointIdd;
  uint8_t  jointIdf;
	double   state;
} __PACKED ServoStatePacket;

#endif
