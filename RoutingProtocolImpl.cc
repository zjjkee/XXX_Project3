#include "RoutingProtocolImpl.h"

RoutingProtocolImpl::RoutingProtocolImpl(Node *n) : RoutingProtocol(n)
{
    sys = n;
    // add your own code
    AllNodes.insert(sys->id);
}

RoutingProtocolImpl::~RoutingProtocolImpl()
{
    // add your own code (if needed)
}

void RoutingProtocolImpl::init(unsigned short num_ports, unsigned short router_id, eProtocolType protocol_type)
{
    // add your own code
    this->portNum = num_ports;
    sys->id = router_id;
    this->protocolType = protocol_type;

    updateCount = 1;
    this->dvTable = map<pair<unsigned short, unsigned short>, int>();
    this->nextHop = map<unsigned short, unsigned short>();

    // intialize ports
    for (int i = 0; i < num_ports; i++)
    {
        portInfo port(i, 0, 0, 0);
        this->ports.push_back(port);
    }
    // set ping alarms
    sendPing();

    // set status check alarm
    checkStatus();

    if (protocol_type == P_DV)
    {
        void *alarm = malloc(1);
        *((unsigned char *)alarm) = ALARM_DV;
        sys->set_alarm(this, 30 * 1000, alarm);
    }
    else
    {
        void *alarm = malloc(1);
        *((unsigned char *)alarm) = ALARM_LS;
        sys->set_alarm(this, 30 * 1000, alarm);
    }
}

void RoutingProtocolImpl::handle_alarm(void *data)
{
    // add your own code
    unsigned char alarmType = *((unsigned char *)data);
    if ((AlarmType)alarmType == ALARM_PING)
    {
        sendPing();
    }
    else if ((AlarmType)alarmType == ALARM_UPDATESTATUS)
    {
        checkStatus();
    }
    else if ((AlarmType)alarmType == ALARM_LS)
    {
        sendLSTable();
        // set alarm for next update
        void *alarm = malloc(1);
        *((unsigned char *)alarm) = ALARM_LS;
        sys->set_alarm(this, 30 * 1000, alarm);
    }
    else if ((AlarmType)alarmType == ALARM_LS_TRIGGERED)
    {
        sendLSTable();
    }
    else if ((AlarmType)alarmType == ALARM_DV)
    {
        sendDVTable();
        // set alarm for next update
        void *alarm = malloc(1);
        *((unsigned char *)alarm) = ALARM_DV;
        sys->set_alarm(this, 30 * 1000, alarm);
    }
    else if ((AlarmType)alarmType == ALARM_DV_TRIGGERED)
    {
        sendDVTable();
    }
    else
    {
        // unknown alarm type
        cerr << "Unknown type of alarm found!\n";
    }
    free(data);
}

void RoutingProtocolImpl::recv(unsigned short port, void *packet, unsigned short size)
{
    // add your own code

    // DATA,  PING,  PONG,  DV,  LS,
    Packet packetData = extractPacketData(packet);
    if (packetData.PacketType == DATA)
    {

        if (packetData.dstID == sys->id)
        {
            // process packet

            // free memory
            free(packet);
        }
        else
        {
            if (port == 0xffff)
            {
                // the packet is originated from this router
            }
            if (nextHop.find(packetData.dstID) != nextHop.end())
            {
                unsigned short next = nextHop[packetData.dstID];
                sys->send(neighbors[next].portNumber, packet, size);
            }
            else
            {
                cerr << "Error - can not forward packet. Destination unknown or unreachable.\n";
            }
        }
    }
    else if (packetData.PacketType == PING)
    {
        recvPing(port, packet);
    }
    else if (packetData.PacketType == PONG)
    {
        recvPong(port, packet);
    }
    else if (packetData.PacketType == DV)
    {
        // update DV table
        if (this->protocolType == P_DV)
            updateDV(packet);
    }
    else if (packetData.PacketType == LS)
    {
        // update LS table
        if (this->protocolType == P_LS)
            updateLS(packet);
    }
    else
    {
        // invalid packet
        cerr << "Error: unknown type of packet recieved!\n";
    }
}

// add more of your own code
void RoutingProtocolImpl::sendPing()
{
    // send ping to every port
    for (int i = 0; i < this->portNum; i++)
    {
        // create packet
        void *data = malloc(12);
        createPacket(PING, 12, sys->id, 0, data);

        // add payload
        *((unsigned int *)data + 2) = htonl(sys->time());

        sys->send(i, data, 12);
    }

    // set alarm for next ping
    void *packet = malloc(1);
    *((unsigned char *)packet) = ALARM_PING;
    sys->set_alarm(this, 10 * 1000, packet);
}
void RoutingProtocolImpl::recvPing(unsigned short port, void *packet)
{
    Packet packetData = extractPacketData(packet);
    createPacket(PONG, packetData.Size, sys->id, packetData.srcID, packet);

    // send pong response
    sys->send(port, packet, packetData.Size);
}
void RoutingProtocolImpl::recvPong(unsigned short port, void *packet)
{
    Packet packetData = extractPacketData(packet);

    // update link table based on recieved time
    unsigned int sendTime = ntohl(*((unsigned int *)(packetData.data)));
    unsigned int delay = sys->time() - sendTime;
    updatePortInfo(port, packetData.srcID, delay);
    free(packet);
}
Packet RoutingProtocolImpl::extractPacketData(void *data)
{
    Packet result;
    unsigned char packetType = *((unsigned char *)data);
    if (packetType >= 0 && packetType < 5)
    {
        result.PacketType = (ePacketType)packetType;
    }
    else
    {
        // unknown packet type
        cerr << "Error - unknown packet type!\n";
    }

    result.Size = ntohs(*((unsigned short *)(data) + 1));
    result.srcID = ntohs(*((unsigned short *)(data) + 2));
    result.dstID = ntohs(*((unsigned short *)(data) + 3));
    result.data = (unsigned char *)data + 8;
    return result;
}
void RoutingProtocolImpl::createPacket(ePacketType packetType, unsigned short size, unsigned short src, unsigned short dst, void *packet)
{
    *((unsigned char *)packet) = (unsigned char)packetType;
    *((unsigned short *)(packet) + 1) = htons(size);
    *((unsigned short *)(packet) + 2) = htons(src);
    *((unsigned short *)(packet) + 3) = htons(dst);
}
void RoutingProtocolImpl::updateLink(myLink link)
{
    neighbors[link.to] = link;
}
void RoutingProtocolImpl::updatePortInfo(unsigned short port, unsigned short ConnectedRouter, unsigned int delay)
{
    ports[port].delay = delay;
    ports[port].connectedRouterID = ConnectedRouter;
    ports[port].LastUpdate = sys->time();
    bool updateNeeded = false;

    if (neighbors.find(ConnectedRouter) == neighbors.end() || neighbors[ConnectedRouter].cost != delay)
    {
        updateNeeded = true;
    }
    neighbors[ConnectedRouter] = myLink(sys->id, ConnectedRouter, delay, port, delay != 0xffff);
    // Update the last known active time for the link from this node (sys->id) to the connected router
    LinksLastUpdate[{sys->id, ConnectedRouter}] = sys->time();
    // Update the last known active time for the reverse link from the connected router to this node
    LinksLastUpdate[{ConnectedRouter, sys->id}] = sys->time();

    if (protocolType == P_LS)
    {
        if (AllNodes.find(ConnectedRouter) == AllNodes.end())
        {
            AllNodes.insert(ConnectedRouter);
            updateNeeded = true;
        }
        if (Links.find({sys->id, ConnectedRouter}) == Links.end())
        {
            Links[{sys->id, ConnectedRouter}] = delay;
            Links[{ConnectedRouter, sys->id}] = delay;
            updateNeeded = true;
        }
    }
    if (updateNeeded)
    {
        if (protocolType == P_DV)
        {
            if (nextHop.find(ConnectedRouter) == nextHop.end() || (unsigned int)dvTable[{ConnectedRouter, nextHop[ConnectedRouter]}] > delay)
            {
                nextHop[ConnectedRouter] = ConnectedRouter;
                dvTable[{ConnectedRouter, ConnectedRouter}] = delay;
                dvTableLastUpdate[{ConnectedRouter, ConnectedRouter}] = sys->time();
                sendDVTable();
            }
        }
        else
        {
            ComputeLSShortestPath();
            sendLSTable();
        }
    }
}
void RoutingProtocolImpl::checkStatus()
{
    for (size_t i = 0; i < ports.size(); i++)
    {
        // ports that is not updated in last 15 seconds is dead
        if (ports[i].LastUpdate > 0 && sys->time() - ports[i].LastUpdate > 15 * 1000)
            updatePortInfo(i, ports[i].connectedRouterID, 0x9999);
    }

    if (protocolType == P_DV)
    {
        bool updated = false;
        for (const auto &entry : dvTableLastUpdate)
        {
            if (entry.first.first == entry.first.second)
                continue;
            // entries that are not updated in last 45 seconds should be considered unreachable
            if (entry.second > 0 && sys->time() - entry.second > 45 * 1000)
            {
                if (dvTable[entry.first] != 0xffff)
                    updated = true;
                dvTable[entry.first] = 0xffff;
            }
        }
        if (updated)
            sendDVTable();
    }
    else
    {
        bool updated = false;
        for (const auto &entry : LinksLastUpdate)
        {
            if (entry.second > 0 && sys->time() - entry.second > 45 * 1000)
            {
                if (Links[entry.first] != 0xffff)
                    updated = true;
                Links[entry.first] = 0xffff;
            }
        }
        if (updated)
        {
            ComputeLSShortestPath();
            sendLSTable();
        }
    }
    // set alarm for next check
    void *alarm = malloc(1);
    *((unsigned char *)alarm) = ALARM_UPDATESTATUS;
    sys->set_alarm(this, 1000, alarm);
}
void RoutingProtocolImpl::updateDV(void *packet)
{
    Packet packetData = extractPacketData(packet);
    bool updated = false;
    for (unsigned short i = 0; i < packetData.Size / 2 - 4; i += 2)
    {
        unsigned short dest = ntohs(*((unsigned short *)(packetData.data) + i));
        unsigned short cost = ntohs(*((unsigned short *)(packetData.data) + i + 1));
        unsigned short neighbor = packetData.srcID;
        unsigned short to_neighbor_cost = neighbors[neighbor].cost;
        unsigned short new_cost = to_neighbor_cost + cost;

        if (dest == sys->id)
            continue;

        // Update distance table and next hop
        if (new_cost < dvTable[{dest, neighbor}])
        {
            dvTable[{dest, neighbor}] = new_cost;
            nextHop[dest] = neighbor;
            updated = true;
        }
        dvTableLastUpdate[{dest, neighbor}] = sys->time();
        // Update routing table based on the minimum cost
        int min_cost = -1;
        for (const auto &entry : neighbors)
        {
            if (min_cost < 0 || dvTable[{dest, entry.second.to}] < min_cost)
            {
                min_cost = dvTable[{dest, entry.second.to}];
                nextHop[dest] = entry.second.to;
            }
        }
    }
    // send updated distances
    if (updated)
    {
        sendDVTable();
    }
    free(packet);
}
void RoutingProtocolImpl::updateLS(void *packet)
{
    bool updated = false;
    Packet packetData = extractPacketData(packet);
    unsigned int seqNumber = ntohl(*((unsigned int *)(packet) + 2));

    if (LastSequenceNumber.find(packetData.srcID) == LastSequenceNumber.end() || LastSequenceNumber[packetData.srcID] < seqNumber)
    {

        for (unsigned short i = 0; i < (packetData.Size - 12) / 2; i += 2)
        {
            unsigned short dest = ntohs(*((unsigned short *)(packet) + 6 + i));
            unsigned short cost = ntohs(*((unsigned short *)(packet) + 6 + i + 1));

            // update new discovered nodes
            if (AllNodes.find(dest) == AllNodes.end())
            {
                AllNodes.insert(dest);
                updated = true;
            }
            // update links
            if (Links.find({packetData.srcID, dest}) == Links.end() || Links[{packetData.srcID, dest}] != cost)
            {
                Links[{packetData.srcID, dest}] = cost;
                Links[{dest, packetData.srcID}] = cost;
                updated = true;
            }
            LinksLastUpdate[{packetData.srcID, dest}] = sys->time();
        }
        // if network status is changed, compute shortest path again
        if (updated)
        {
            ComputeLSShortestPath();
        }
        // update last sequence number
        LastSequenceNumber[packetData.srcID] = seqNumber;
    }
    free(packet);
}

void RoutingProtocolImpl::ComputeLSShortestPath()
{
    // Set of nodes for which the shortest path is known (starts with self)
    set<unsigned short> unknownNodes;

    // shortest distances found so far
    map<unsigned short, unsigned int> dist;

    for (const auto &node : AllNodes)
    {
        if (node != sys->id)
            unknownNodes.insert(node);
        dist[node] = 0xffff;
    }

    // parent in shortest path tree
    map<unsigned short, unsigned short> parent;

    dist[sys->id] = 0;

    // intialize to neighbor links
    for (const auto &neighbor : neighbors)
    {
        dist[neighbor.second.to] = neighbor.second.cost;
        parent[neighbor.second.to] = sys->id;
    }

    // Dijkstra algorithm for findung shortest distances
    while (unknownNodes.size() > 0)
    {
        // Find the node with min dist
        unsigned int minDist = 0;
        unsigned short minNodeID = 0;

        for (const auto &node : unknownNodes)
        {
            if (minDist == 0 || dist[node] < minDist)
            {
                minDist = dist[node];
                minNodeID = node;
            }
        }
        if (minDist == 0xffff)
            break;
        // add min to known nodes
        unknownNodes.erase(minNodeID);

        // update distances based on found min
        for (const auto &edge : Links)
        {
            if (edge.first.first == minNodeID)
            {
                if (dist[edge.first.second] > dist[minNodeID] + edge.second)
                {
                    dist[edge.first.second] = dist[minNodeID] + edge.second;
                    parent[edge.first.second] = minNodeID;
                }
            }
        }
    }

    // find next hop for each node using shortest map tree
    for (const auto &node : AllNodes)
    {
        if (node != sys->id && dist[node] != 0xffff)
        {
            unsigned short p = parent[node];
            if (p == sys->id)
                nextHop[node] = node;
            else
            {
                while (parent[p] != sys->id)
                    p = parent[p];
                nextHop[node] = p;
            }
        }
    }
}

void RoutingProtocolImpl::sendLSTable()
{
    // send the packet to all ports
    for (size_t p = 0; p < portNum; p++)
    {
        // create a packet to send LS table

        int packetSize = 12;
        // we should send just the live links
        for (size_t i = 0; i < portNum; i++)
        {
            if (ports[i].delay > 0 && ports[i].delay < 0xffff)
                packetSize += 4;
        }
        void *packet = malloc(packetSize);
        createPacket(LS, packetSize, sys->id, 0, packet);

        // set sequence number
        *((unsigned int *)packet + 2) = htonl(this->updateCount);
        this->updateCount++;

        // add neighbors link info
        unsigned short k = 0;
        for (size_t i = 0; i < portNum; i++)
        {
            if (ports[i].delay > 0 && ports[i].delay < 0xffff)
            {
                (*(((unsigned short *)packet) + 6 + k)) = htons(ports[i].connectedRouterID);
                (*(((unsigned short *)packet) + 6 + k + 1)) = htons((unsigned short)ports[i].delay);
                k += 2;
            }
        }

        sys->send(p, packet, packetSize);
    }
}
void RoutingProtocolImpl::sendDVTable()
{

    // for each neighbor create a packet to send DV table
    for (const auto &entry : neighbors)
    {
        int packetSize = 8 + dvTable.size() * 4;
        void *packet = malloc(packetSize);
        createPacket(DV, packetSize, sys->id, 0, packet);
        int i = 0;
        for (const auto &dest : nextHop)
        {
            // don't send a path starting from neighbor to himself -- poison reverse
            if (dest.second == entry.first)
            {
                *(((unsigned short *)packet) + 4 + i) = htons(dest.second);
                *(((unsigned short *)packet) + 4 + i + 1) = 0xffff;
            }
            else
            {
                *(((unsigned short *)packet) + 4 + i) = htons(dest.second);
                *(((unsigned short *)packet) + 4 + i + 1) = htons(dvTable[{dest.first, dest.second}]);
            }
            i += 2;
        }

        sys->send(entry.second.portNumber, packet, packetSize);
    }
}