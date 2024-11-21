#ifndef LINK_STATE_PROTOCOL_H
#define LINK_STATE_PROTOCOL_H

#include "global.h"
#include <arpa/inet.h>
#include <unordered_map>
#include <vector>
#include <set>

// Structure representing a Link State Record
struct LinkStateRecord {
    unsigned int expirationTime;
    unsigned short neighborID;
    unsigned short linkCost;

    LinkStateRecord(unsigned int expiration, unsigned short neighbor, unsigned short cost) 
        : expirationTime(expiration), neighborID(neighbor), linkCost(cost) {}

    LinkStateRecord(LinkStateRecord* record) 
        : expirationTime(record->expirationTime), 
          neighborID(record->neighborID), 
          linkCost(record->linkCost) {}
};

class LinkStateProtocol {

public:

    // Constructor and Destructor
    LinkStateProtocol(unsigned short routerID);
    ~LinkStateProtocol();

    // Methods to handle topology and link state changes
    void updateLinkState(std::set<unsigned short>& modifiedNeighbors);
    void updateTopology(unsigned short nodeID);
    bool validateLinkState(unsigned int currentTime);
    LinkStateRecord* getLinkStateRecord(unsigned short sourceID);

    // Router and link state updates
    void setRouterID(unsigned short routerID);
    void processUpdatePacket(char* packet, unsigned int expiration, unsigned int currentTime, unsigned short packetSize);
    bool processPongResponse(unsigned short sourceID, unsigned int expiration, unsigned short linkCost, unsigned int currentTime);
    void computeShortestPaths(std::unordered_map<unsigned short, unsigned short>& routingTable);
    void createLinkStatePacket(char* packet, unsigned short packetSize);
    
    // Sequence number and state tracking
    void incrementSequenceNumber();
    bool checkSequenceNumber(char* packet);

    // Data structures for protocol state
    std::unordered_map<unsigned short, unsigned int> nodeToSequenceNumber;
    std::unordered_map<unsigned short, std::vector<LinkStateRecord*>*> recordTable;
    std::vector<LinkStateRecord*> currentLinkState;

private:
    unsigned short routerID;  // Unique ID for the router
    unsigned int sequenceNumber;  // Sequence number for link state packets
};

#endif
