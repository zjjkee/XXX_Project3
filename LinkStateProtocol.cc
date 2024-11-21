#include "LinkStateProtocol.h"

// Constructor: Initialize router ID and sequence number
LinkStateProtocol::LinkStateProtocol(unsigned short routerID) 
    : routerID(routerID), sequenceNumber(0) {}

// Initialize the router to start sending Link State Packets
void LinkStateProtocol::setRouterID(unsigned short routerID) {
    this->routerID = routerID;
    this->sequenceNumber = 0;
    this->currentLinkState.clear();
    this->nodeToSequenceNumber.clear();
}

// Modify link state according to changes or deletions
void LinkStateProtocol::updateLinkState(std::set<unsigned short>& modifiedIDs) {
    for (auto& id : modifiedIDs) {
        auto it = currentLinkState.begin();
        while (it != currentLinkState.end()) {
            LinkStateRecord* record = *it;
            if (record->neighborID == id) {
                updateTopology(record->neighborID);
                it = currentLinkState.erase(it);
                free(record);
                break;
            } else {
                ++it;
            }
        }
    }
}

// Destructor: Clean up dynamically allocated memory
LinkStateProtocol::~LinkStateProtocol() {
    for (auto& record : currentLinkState) {
        free(record);
    }
    currentLinkState.clear();

    for (auto& entry : recordTable) {
        for (auto& record : *(entry.second)) {
            free(record);
        }
        delete entry.second;
    }
    recordTable.clear();
}

// Update topology by removing unresponsive links
void LinkStateProtocol::updateTopology(unsigned short nodeID) {
    auto it = recordTable.find(nodeID);
    if (it == recordTable.end()) return;

    auto* recordVector = it->second;
    for (auto& record : *recordVector) {
        free(record);
    }
    delete recordVector;
    recordTable.erase(it);
}

// Check if any links are unresponsive
bool LinkStateProtocol::validateLinkState(unsigned int currentTime) {
    bool hasChanged = false;
    auto it = currentLinkState.begin();
    while (it != currentLinkState.end()) {
        if ((*it)->expirationTime < currentTime) {
            hasChanged = true;
            LinkStateRecord* record = *it;
            updateTopology(record->neighborID);
            it = currentLinkState.erase(it);
            free(record);
        } else {
            ++it;
        }
    }
    return hasChanged;
}

// Create a Link State Packet
void LinkStateProtocol::createLinkStatePacket(char* packet, unsigned short packetSize) {
    *(char*)packet = LS;
    *(unsigned short*)(packet + 2) = htons(packetSize);
    *(unsigned short*)(packet + 4) = htons(this->routerID);
    *(unsigned int*)(packet + 8) = htonl(this->sequenceNumber);

    int count = 0;
    for (auto& record : currentLinkState) {
        int offset = 12 + (count * 4);
        *(unsigned short*)(packet + offset) = htons(record->neighborID);
        *(unsigned short*)(packet + offset + 2) = htons(record->linkCost);
        ++count;
    }
}

// Return the Link State Record for a given source ID
LinkStateRecord* LinkStateProtocol::getLinkStateRecord(unsigned short sourceID) {
    for (auto& record : currentLinkState) {
        if (record->neighborID == sourceID)
            return record;
    }
    return nullptr;
}

// Calculate shortest paths and populate the routing table
void LinkStateProtocol::computeShortestPaths(std::unordered_map<unsigned short, unsigned short>& routingTable) {
    routingTable.clear();
    std::unordered_map<unsigned short, std::pair<unsigned short, unsigned short>> costMap;

    for (auto& record : currentLinkState) {
        costMap[record->neighborID] = { record->linkCost, record->neighborID };
    }

    while (!costMap.empty()) {
        unsigned short minCost = INFINITY_COST;
        unsigned short nextNode = 0;

        for (const auto& [node, costPair] : costMap) {
            if (costPair.first < minCost) {
                minCost = costPair.first;
                nextNode = node;
            }
        }

        routingTable[nextNode] = costMap[nextNode].second;
        costMap.erase(nextNode);

        auto it = recordTable.find(nextNode);
        if (it != recordTable.end()) {
            for (auto& record : *(it->second)) {
                unsigned short totalCost = minCost + record->linkCost;

                if (costMap.find(record->neighborID) != costMap.end() &&
                    totalCost < costMap[record->neighborID].first) {
                    costMap[record->neighborID] = { totalCost, nextNode };
                } else if (routingTable.find(record->neighborID) == routingTable.end() &&
                           record->neighborID != this->routerID) {
                    costMap[record->neighborID] = { totalCost, nextNode };
                }
            }
        }
    }
}

// Update PONG response
bool LinkStateProtocol::processPongResponse(unsigned short sourceID, unsigned int timeout, unsigned short linkCost, unsigned int currentTime) {
    bool hasChanged = false;
    LinkStateRecord* record = getLinkStateRecord(sourceID);

    if (record) {
        record->expirationTime = currentTime + timeout;
        if (linkCost != record->linkCost) {
            record->linkCost = linkCost;
            hasChanged = true;
        }
    } else {
        record = static_cast<LinkStateRecord*>(malloc(sizeof(LinkStateRecord)));
        record->neighborID = sourceID;
        record->linkCost = linkCost;
        record->expirationTime = currentTime + timeout;
        currentLinkState.push_back(record);
        hasChanged = true;
    }
    return hasChanged;
}

// Process and update Link State from received packet
void LinkStateProtocol::processUpdatePacket(char* packet, unsigned int timeout, unsigned int currentTime, unsigned short size) {
    unsigned short sourceID = ntohs(*(unsigned short*)(packet + 4));
    unsigned int recordCount = (size - 12) / 4;

    auto* recordVector = new std::vector<LinkStateRecord*>;
    for (unsigned int i = 0; i < recordCount; ++i) {
        unsigned int offset = 12 + (i * 4);
        unsigned short neighborID = ntohs(*(unsigned short*)(packet + offset));
        unsigned short cost = ntohs(*(unsigned short*)(packet + offset + 2));

        if (neighborID == this->routerID) continue;

        auto* record = static_cast<LinkStateRecord*>(malloc(sizeof(LinkStateRecord)));
        record->neighborID = neighborID;
        record->linkCost = cost;
        recordVector->push_back(record);
    }

    auto* existingRecord = getLinkStateRecord(sourceID);
    if (existingRecord) {
        existingRecord->expirationTime = currentTime + timeout;
    }

    auto it = recordTable.find(sourceID);
    if (it != recordTable.end()) {
        for (auto& record : *(it->second)) {
            free(record);
        }
        delete it->second;
    }
    recordTable[sourceID] = recordVector;
}

// Increment the sequence number
void LinkStateProtocol::incrementSequenceNumber() {
    this->sequenceNumber++;
}

// Verify sequence number for a received packet
bool LinkStateProtocol::checkSequenceNumber(char* packet) {
    unsigned short sourceID = ntohs(*(unsigned short*)(packet + 4));
    unsigned int seqNum = ntohl(*(unsigned int*)(packet + 8));

    if (sourceID == this->routerID) return false;

    if (nodeToSequenceNumber.find(sourceID) == nodeToSequenceNumber.end() ||
        nodeToSequenceNumber[sourceID] < seqNum) {
        nodeToSequenceNumber[sourceID] = seqNum;
        return true;
    }
    return false;
}
