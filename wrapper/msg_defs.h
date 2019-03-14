#ifndef MSG_DEFS_H
#define MSG_DEFS_H

typedef unsigned char uchar;

#define PACKET_SIZE 24
#define PACKET_OVERHEAD 3
#define PACKET_DATA_SIZE (PACKET_SIZE - PACKET_OVERHEAD)

#define POS_SOP 0
#define POS_TYPE 1
#define POS_DATA 2
#define POS_CHECKSUM (PACKET_SIZE-1)

#define SOP 253

#define LRA_NO_SPIN 0
#define LRA_SPIN 1

// Packets with type NONE are considered invalid Packets
enum PacketType {
  NONE = 0,
  ACK = 1,
  STANDBY = 2,
  STATE_CHANGE = 3,
  IMU_DATA = 4,
  LRA_CONTROL = 5,
  NUM_TYPES
};

typedef uchar PacketData[PACKET_DATA_SIZE];

union Packet {

  struct {
    uchar sop;
    uchar type;
    uchar data[PACKET_DATA_SIZE];
    uchar checksum;
  } as_struct;

  uchar as_array[PACKET_SIZE];

};

#endif