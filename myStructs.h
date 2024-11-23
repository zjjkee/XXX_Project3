#ifndef __MY_STRUCTS__H
#define __MY_STRUCTS__H
enum AlarmType
{
  ALARM_PING,
  ALARM_UPDATESTATUS,
  ALARM_DV,
  ALARM_DV_TRIGGERED,
  ALARM_LS,
  ALARM_LS_TRIGGERED
};
struct Packet
{
  ePacketType PacketType;
  unsigned short Size;
  unsigned short srcID;
  unsigned short dstID;

  void *data;
};
struct portInfo
{
  unsigned short portNum;
  unsigned short connectedRouterID;
  unsigned int delay;
  unsigned int LastUpdate;
  portInfo(unsigned short portNum, unsigned short connectedRouterID, unsigned int delay, unsigned int lastUpdate)
  {
    this->portNum = portNum;
    this->connectedRouterID = connectedRouterID;
    this->delay = delay;
    this->LastUpdate = lastUpdate;
  }
};
struct myLink
{
  unsigned short from;
  unsigned short to;
  unsigned int cost;
  unsigned short portNumber;
  bool isAlive;
  myLink() {}
  myLink(unsigned short from, unsigned short to, unsigned int cost, unsigned short portNo, bool isAlive)
  {
    this->from = from;
    this->to = to;
    this->isAlive = isAlive;
    this->cost = cost;
    this->portNumber = portNo;
  }
};
#endif