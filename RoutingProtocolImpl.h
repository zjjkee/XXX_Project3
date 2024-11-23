#ifndef ROUTINGPROTOCOLIMPL_H
#define ROUTINGPROTOCOLIMPL_H

#include "RoutingProtocol.h"
#include "Node.h"
#include "myStructs.h"
#include <arpa/inet.h>
#include <map>
#include <set>
#include <utility>

class RoutingProtocolImpl : public RoutingProtocol
{
public:
  RoutingProtocolImpl(Node *n);
  ~RoutingProtocolImpl();

  void init(unsigned short num_ports, unsigned short router_id, eProtocolType protocol_type);
  // As discussed in the assignment document, your RoutingProtocolImpl is
  // first initialized with the total number of ports on the router,
  // the router's ID, and the protocol type (P_DV or P_LS) that
  // should be used. See global.h for definitions of constants P_DV
  // and P_LS.

  void handle_alarm(void *data);
  // As discussed in the assignment document, when an alarm scheduled by your
  // RoutingProtoclImpl fires, your RoutingProtocolImpl's
  // handle_alarm() function will be called, with the original piece
  // of "data" memory supplied to set_alarm() provided. After you
  // handle an alarm, the memory pointed to by "data" is under your
  // ownership and you should free it if appropriate.

  void recv(unsigned short port, void *packet, unsigned short size);
  // When a packet is received, your recv() function will be called
  // with the port number on which the packet arrives from, the
  // pointer to the packet memory, and the size of the packet in
  // bytes. When you receive a packet, the packet memory is under
  // your ownership and you should free it if appropriate. When a
  // DATA packet is created at a router by the simulator, your
  // recv() function will be called for such DATA packet, but with a
  // special port number of SPECIAL_PORT (see global.h) to indicate
  // that the packet is generated locally and not received from
  // a neighbor router.

private:
  Node *sys; // To store Node object; used to access GSR9999 interfaces
  unsigned short portNum;
  eProtocolType protocolType;
  vector<portInfo> ports;

  // =======================================  DV data

  // table to store all current min distances in DV algorithm - dvTable[{dest, via}] = cost
  map<pair<unsigned short, unsigned short>, int> dvTable;
  // store the last update time for each entry of dvTable
  map<pair<unsigned short, unsigned short>, unsigned int> dvTableLastUpdate;
  // map to store the next Hop to send data for a dest. Filled with DV and LS algorithms. nextHop[dest] = nodeID
  map<unsigned short, unsigned short> nextHop;

  // ========================================  LS data

  // set of all discovered nodes
  set<unsigned short> AllNodes;
  // set of all discovered links between all nodes - Links[{from, to}] = cost
  map<pair<unsigned short, unsigned short>, int> Links;
  // stores the last update time for current link
  map<pair<unsigned short, unsigned short>, unsigned int> LinksLastUpdate;
  // map to store last recieved sequence number - LastSequenceNumber[node] = seqNumber
  map<unsigned short, unsigned int> LastSequenceNumber;
  // store number to use as sequence number to send other nodes
  unsigned int updateCount = 0;

  // extracts common packet info from data.
  Packet extractPacketData(void *data);
  // creates a packet from given data and writes it to address given by pointer
  void createPacket(ePacketType packetType, unsigned short size, unsigned short src, unsigned short dst, void *packet);

  // create a pong response based on recieved ping and send it to source node
  void recvPing(unsigned short port, void *packet);
  // updates all data according to recieved pong
  void recvPong(unsigned short port, void *packet);

  // map from node id to myLink object to store related data - neighbors[node] = myLink object
  map<unsigned short, myLink> neighbors;
  // update neighbor link based on given data
  void updateLink(myLink link);

  // ================  alarms

  // sends Ping request on all ports
  void sendPing();

  // ================  ports

  // updates port info (link to neighbors) and the cost after recieving pong
  void updatePortInfo(unsigned short port, unsigned short ConnectedRouter, unsigned int delay);
  // checks last update time of the links and table entries etc
  void checkStatus();

  // ================  update linktable

  // updates LS links info , calls computeshortestpath and updates next hop for each destination
  void updateLS(void *packet);
  // updates DVtable from given data on packet
  void updateDV(void *packet);
  // sends info about neigbors to all nodes
  void sendLSTable();
  // sends DVtable info to all neighbor nodes
  void sendDVTable();

  // computes shortest path in LS algorithm
  void ComputeLSShortestPath();
};

#endif
