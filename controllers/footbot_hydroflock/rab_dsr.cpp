#include "rab_dsr.h"

size_t CDynamicSourceRouting::m_nPacketSize = 0;
size_t CDynamicSourceRouting::m_nSequenceNumber = 0;

CDynamicSourceRouting::CDynamicSourceRouting(const std::string& robotID) : m_sRobotID(robotID) {}

CDynamicSourceRouting::~CDynamicSourceRouting() {}

void CDynamicSourceRouting::Init(CCI_RangeAndBearingSensor* pcRABSens, CCI_RangeAndBearingActuator* pcRABActuator) {
    m_pcRABSens = pcRABSens;
    m_pcRABActuator = pcRABActuator;
    m_nPacketSize = m_pcRABActuator->GetSize();
}

std::vector<CDynamicSourceRouting::Packet> CDynamicSourceRouting::ListenAndUpdate() {
    // TODO: Design ListenAndUpdate function

    /**
     * * Functions to include here:
     * 
     *  ✔️  1.      Receive() - Get and deserialize packets from the RAB sensor. This will filter and return packets that are 
     *                  for the controller (not used for background services).
     * 
     *  ✔️  1.1.    ProcessInboundPackets() - Process the received packets and update the routing table, neighbor table, and 
     *                  other data structures. (called by Receive())
     * 
     *  ❌  2.      NeighborRequest() - Send a neighbor request to all neighbors. This will trigger the neighbors to send a 
     *                  neighbor reply and update their neighbor table. (should have a custom frequency)
     * 
     *  ✔️  3.      Send() - Send packets using the RAB actuator. This will send the next packet in the queue to the neighbors.
     * 
     *  ✔️  3.1.    ProcessOutboundPackets() - Preprocess the packets in the prep queue and send them to the ready queue, urgent 
     *                  queue, or waiting for route discovery queue. (called by Send())
     * 
     */
    
    std::vector<Packet> controllerBoundPackets = Receive();

    Send();

    return controllerBoundPackets;
}

void CDynamicSourceRouting::Send() {

    ProcessOutboundPackets();

    while (!m_qOutPacketReadyQueue.empty()) {
        Packet packet = m_qOutPacketReadyQueue.front();
        if (packet.GetState() != READY) {
            LOGERR << "Dev Error: Packet state is not READY but is attempting to be sent." << std::endl;
            exit(1);
        }
        m_qOutPacketReadyQueue.pop();
        m_pcRABActuator->SetData(packet.ToByteArray());
    }

}

std::vector<CDynamicSourceRouting::Packet> CDynamicSourceRouting::Receive() {
    // Get and deserialize received packets
    std::vector<Packet> receivedPackets;
    const CCI_RangeAndBearingSensor::TReadings& tReadings = m_pcRABSens->GetReadings();
    if (tReadings.empty()) std::cout << "No packets received." << std::endl;
    for (const auto& tReading : tReadings) {
        receivedPackets.push_back(Packet(tReading.Data));
    }
    
    // Do background processing and return packets that are only for this robot that haven't already been processed.
    return ProcessInboundPackets(receivedPackets);
}

std::vector<CDynamicSourceRouting::Packet> CDynamicSourceRouting::ProcessInboundPackets(std::vector<Packet>& packetList) {

    std::vector<Packet> filteredPackets; // Packets that are for this Robot (e.g. broadcast or unicast to this robot's ID)

    for (CDynamicSourceRouting::Packet& packet : packetList) {

        bool alreadyProcessed = m_setReceivedPackets.find(packet.GetPacketID()) != m_setReceivedPackets.end();
        
        switch(packet.GetMode()){

            case BROADCAST:
                if (!alreadyProcessed) {
                    filteredPackets.push_back(packet);
                    AddToQueue(packet);
                }
                break;

            case UNICAST:
                if (!alreadyProcessed) {

                    if (packet.GetDestination() == m_sRobotID) { // If this robot is the destination (intended recipient)

                        filteredPackets.push_back(packet);

                    } else if (IsOnRoute(packet)) {     // If this robot is on the route but not the destination (intended recipient)

                        AddToQueue(packet);
                    }
                }
                break;

            case RREQ: // Route Request

                if (!alreadyProcessed){

                    if (packet.GetDestination() == m_sRobotID) {
                        
                        Path new_path = packet.GetPath();
                        new_path.push_back(m_sRobotID);
                        packet.SetPath(new_path);     // new path = old path + this robot's ID

                        // Change mode to route reply
                        packet.SetRREP();

                        // Reset hop count
                        packet.ResetHopCount(); 

                        // Add RREP packet to outbound preprocessing queue
                        AddToQueue(packet);
                        
                        
                    } else {
                        
                        Path new_path = packet.GetPath();
                        new_path.push_back(m_sRobotID);
                        packet.SetPath(new_path);     // new path = old path + this robot's ID

                        // Add RREQ packet to outbound preprocessing queue
                        AddToQueue(packet);
                    }
                }
                break;

            case RREP: // Route Reply

                if (packet.GetSource() == m_sRobotID) { // If this robot is the source of the RREQ/RREP packet

                    // Update the routing table
                    m_mapRoutingTable[packet.GetDestination()] = Path(packet.GetPath());

                    // Check if there are packets waiting for this route, if so set them to ready, add to ready queue, and remove from awaiting RD map
                    if (m_mapPacketsAwaitingRD.find(packet.GetDestination()) != m_mapPacketsAwaitingRD.end()) {
                        for (auto& waitingPacket : m_mapPacketsAwaitingRD[packet.GetDestination()].waitingPackets) {
                            waitingPacket.SetState(READY);
                            m_qOutPacketReadyQueue.push(waitingPacket);
                        }
                        m_mapPacketsAwaitingRD.erase(packet.GetDestination());
                    }

                } else if (IsOnRoute(packet)){ // If robot is on the route and not the source

                    // Add RREP packet to outbound preprocessing queue
                    AddToQueue(packet);
                }
                break;

            case RERR: // Route Error

                if (!alreadyProcessed){

                    // Parse the payload to get the disconnected edge
                    std::pair<std::string,std::string> edge = ParseRERRPayload(packet.GetPayload());

                    // Remove all entries in the routing table that contain the disconnected edge
                    for(auto it = m_mapRoutingTable.begin(); it != m_mapRoutingTable.end(); ++it) {
                        if (HasEdgeOnRoute(it->second, edge.first, edge.second)) {
                            m_mapRoutingTable.erase(it);
                        }
                    }

                    // Add RERR packet to outbound preprocessing queue
                    AddToQueue(packet);
                }

            case NREQ: // Neighbor Request

                //TODO: Must note that NREQ and NREP process can be done during route discovery without the need for a separate process.
                /**
                 * @brief This separate process adds more overhead to the system. It might be better to integrate the NREQ and NREP into the route discovery process.
                 * 
                 * However, as DSR is an on-demand protocol, a separate process might be required if we want to maintain a more accurate and up-to-date neighbor table.
                 * 
                 * Moreover, it might be required for the RERR packets. To initiate a RERR packet we need an event such as a neighbor going out of range. The only
                 * way we can send up-to-date neighbor information is by maintaining the neighbor table at more frequent intervals than what might occur during route discovery.
                 * 
                 * I am uncertain of this and will need to do more research on this.
                 * 
                 */
                 

                /**
                 * @brief We see the NREQ as a trigger to initiate the NREP broadcast which allows the receivers
                 * of the NREQ to let their neighbors know they exist. This is important for the RERR packets.
                 * 
                 */

                if (!alreadyProcessed) {

                    // Send a neighbor reply
                    packet.SetNREP();
                    packet.SetDestination(packet.GetSource());  // For NREP, the destination is the source of the NREQ packet (not necessary but maybe useful for debugging)
                    packet.SetSource(m_sRobotID);               // For NREP, the source is this robot's ID
                    AddToQueue(packet);
                }

            case NREP: // Neighbor Reply

                /**
                 * @brief All robots that receive the NREP packet will update their neighbor table with the source of the NREP packet.
                 * 
                 * Even if the robot is not the source of the NREQ packet, it will still update its neighbor table with the source of the NREP packet.
                 * 
                 */
            
                // Update the neighbor table
                m_setNeighborList.insert(packet.GetSource());
        
            default:
                if (packet.GetMode() == -1){
                    LOGERR << "Dev Error: Received packet with uninitialized mode." << std::endl;
                } else {
                    LOGERR <<   "User Error: Invalid packet mode received. Becareful changing the modes. Check doucmentation." << std::endl << 
                                "Mode: " << packet.GetMode() << "Is not a valid mode..." << std::endl;
                }

                break;
        } 

    }

    return filteredPackets;
}

bool CDynamicSourceRouting::IsOnRoute(const Packet& packet, const std::string& robotID) {

    std::string id = robotID;
    bool isOnRoute = false;
    if (robotID.empty()) id = m_sRobotID;
    
    switch (packet.GetMode()){
        case UNICAST:
            isOnRoute = packet.GetPath()[packet.GetHopCount()] == id ? true : false;
            break;
        case RREP:
            isOnRoute = packet.GetPath()[packet.GetPath().size() - packet.GetHopCount() - 1] == id ? true : false;
            break;
        default:
            isOnRoute = true;
            break;
    }

    return isOnRoute;
}

bool CDynamicSourceRouting::HasEdgeOnRoute(const Path& route, const std::string& nodeA, const std::string& nodeB) {
    bool hasEdgeOnRoute = false;
    for (auto it1 = route.begin(), it2 = route.begin()+1; it2 != route.end() - 1; it1++, it2++) {
        hasEdgeOnRoute = (*it1 == nodeA && *it2 == nodeB) || (*it1 == nodeB && *it2 == nodeA) ? true : false;
        if (hasEdgeOnRoute) break;
    }
    return hasEdgeOnRoute;
}

std::pair<std::string,std::string> CDynamicSourceRouting::ParseRERRPayload(const std::string& payload) {
    size_t pos = payload.find("-");
    if (pos == std::string::npos) {
        LOGERR << "Error parsing RERR payload: " << payload << std::endl;
        LOGERR << "Could not find delimiter '-' in payload." << std::endl;
        exit(1);
    }
    return std::make_pair(payload.substr(0, pos), payload.substr(pos + 1));
}

void CDynamicSourceRouting::NeighborRequest(const CVector2& myPosition) {

    Packet nreqPacket(NREQ, m_sRobotID, myPosition);
    AddToQueue(nreqPacket);
}

void CDynamicSourceRouting::StartRouteDiscovery(const std::string& destination) {
    // TODO: Implement route discovery process
    return;
}

void CDynamicSourceRouting::StartRouteMaintenance() {
    // TODO: Implement route maintenance process
    return;
}

void CDynamicSourceRouting::PacketForwarding() {
    // TODO: Implement packet forwarding process (Not sure we need this anymore)

    //! I think this will get replaced with ProcessOutboundPackets() function

    return;
}

void CDynamicSourceRouting::AddToQueue(Packet& packet){
    packet.SetState(QUEUED_FOR_PROCESSING);
    m_qOutPacketPrepQueue.push(packet);
}

void CDynamicSourceRouting::ProcessOutboundPackets(){
    // Preprocess packets in the queue
    while (!m_qOutPacketPrepQueue.empty()) {
        Packet packet = m_qOutPacketPrepQueue.front();
        m_qOutPacketPrepQueue.pop();
        
        switch(packet.GetMode()){

            case BROADCAST:
                // Increment hop count
                packet.IncrementHopCount();
                // Set packet state to ready
                packet.SetState(READY);
                // Add to ready queue
                m_qOutPacketReadyQueue.push(packet);
                break;

            case UNICAST:
                // check if route is in the routing table
                if (m_mapRoutingTable.find(packet.GetDestination()) != m_mapRoutingTable.end()) {
                    // Send to ready queue
                    packet.SetState(READY);
                    m_qOutPacketReadyQueue.push(packet);
                } else {
                    // Switch packet state to waiting for route discovery
                    packet.SetState(WAITING_FOR_ROUTE_DISCOVERY);
                    // Add to packets awaiting route discovery
                    m_mapPacketsAwaitingRD[packet.GetDestination()].waitingPackets.push_back(packet);
                    // if this is the first packet for this destination, start route discovery
                    if (m_mapPacketsAwaitingRD[packet.GetDestination()].waitingPackets.size() == 1) {
                        m_mapPacketsAwaitingRD[packet.GetDestination()].awaitingRREP = true;
                        //TODO: Need to implement route discovery to call StartRouteDiscovery(packet.GetDestination());
                        // StartRouteDiscovery(packet.GetDestination());
                    }                 
                }
                break;

            case RREQ:
                // Send to ready queue
                packet.SetState(READY);
                m_qOutPacketReadyQueue.push(packet);
                break;

            case RREP:
                // Send to ready queue
                packet.SetState(READY);
                m_qOutPacketReadyQueue.push(packet);
                break;

            case RERR:
                // Send to urgent queue
                packet.SetState(URGENT);
                m_qOutPacketUrgentQueue.push(packet);
                break;

            case NREQ:
                // Send to ready queue
                packet.SetState(READY);
                m_qOutPacketReadyQueue.push(packet);
                break;

            case NREP:
                // Send to ready queue
                packet.SetState(READY);
                m_qOutPacketReadyQueue.push(packet);
                break;

            default:
                LOGERR << "Invalid packet mode received: " << packet.GetMode() << std::endl;
                break;
        }
    }

}

uint32_t CDynamicSourceRouting::FNV1aHash(const std::string& key) {
    // FNV-1a hash algorithm
    uint32_t hash = 2166136261u;
    for (char c : key) {
        hash ^= static_cast<uint32_t>(c);
        hash *= 16777619u;
    }
    return hash;
}

uint32_t CDynamicSourceRouting::DJB2Hash(const std::string& str) {
    uint32_t hash = 5381;
    for (char c : str) {
        hash = ((hash << 5) + hash) + static_cast<uint32_t>(c);  // hash * 33 + c
    }
    return hash;
}


/********************************************************************************************************/

CDynamicSourceRouting::Packet::Packet() : mode(NONE), hop_count(0), path(), source(""), position(CVector2()), destination(""), payload(""), seq_num(-1), packet_id(0), state(UNINITIALIZED) {}

CDynamicSourceRouting::Packet::Packet(  const PacketMode& mode, const std::string& source, const CVector2& position, 
                                        const std::string& destination, const std::string& payload) {
    this->mode = mode;
    this->hop_count = 0; // Initialize hop count to 0
    this->path = Path(); // Initialize path to empty
    this->source = source;
    this->position = position;
    this->destination = destination;
    this->payload = payload;
    this->seq_num = m_nSequenceNumber++; // Generate a unique packet sequence number
    this->state = UNINITIALIZED; // Initialize state to waiting for preprocessing
    GeneratePacketID(); // Generate a unique packet ID using the default hash function
}

CDynamicSourceRouting::Packet::Packet(const CByteArray& c_array) {
    Deserialize(c_array);
    GeneratePacketID();
}

CByteArray CDynamicSourceRouting::Packet::Serialize() const {
    CByteArray c_array;

    // Serialize mode
    c_array << static_cast<UInt8>(mode) << "|";

    // Serialize hop count
    c_array << static_cast<UInt8>(hop_count) << "|";

    // Serialize path (if RREQ/RREP or unicast)
    if (mode == UNICAST || mode == RREQ || mode == RREP) {
        // Serailize path
        for (auto node : path) {
            for (char c : node) {
                c_array << static_cast<UInt8>(c);
            }
            c_array << ":"; // Delimiter between nodes (robot IDs)
        }
        c_array << "|"; // Delimiter between packet elements
    }

    // Serialize source ID
    for (char c : source) {
        c_array << static_cast<UInt8>(c) << "|";
    }

    // Serialize position
    c_array << static_cast<Real>(position.GetX()) << "," << static_cast<Real>(position.GetY()) << "|";


    // Serialize destination (if RREQ/RREP or unicast)
    if (mode == UNICAST || mode == RREQ || mode == RREP) {

        // Serialize destination
        for (char c : destination) {
            c_array << static_cast<UInt8>(c) << "|";
        }
    }

    // Serialize payload (if broadcast or unicast)
    if (mode == BROADCAST || mode == UNICAST || mode == RERR) {
        for (char c : payload) {
            c_array << static_cast<UInt8>(c) << "|";
        }
    }

    // Serialize the packet sequence number
    c_array << static_cast<UInt32>(seq_num) << "|";

    if (c_array.Size() > m_nPacketSize) {
        LOGERR << "Packet size exceeds maximum size of " << m_nPacketSize << " bytes specified for RAB Actuator in .xml/.argos file. [Adjust packet size]" << std::endl;
        exit(1);
    } else {
        // Fill the rest of the packet with empty space.
        for(size_t i = c_array.Size(); i < m_nPacketSize; ++i) {
            c_array << static_cast<UInt8>(0);
        }
        return c_array;
    } 
}

void CDynamicSourceRouting::Packet::Deserialize(const CByteArray& c_array) {
    size_t i = 0;
    std::string element = "";
    char c;

    try {
        // Deserialize mode
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Mode is empty");
        if (element == "-1") throw std::runtime_error("An uninitialized packet was sent.");     // TODO: Maybe add feature to generate log file to log messages sent and received.
        if (std::stoi(element) >= NONE && std::stoi(element) <= NREP){
            mode = static_cast<PacketMode>(std::stoi(element));
        } else {
            throw std::runtime_error("Invalid packet mode");
        }
        element = "";

        // Deserialize hop count
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Hop count is empty");
        hop_count = std::stoi(element);
        element = "";

        // Deserialize path (if RREQ/RREP or unicast)
        if (mode == RREQ || mode == RREP || mode == UNICAST) {
            // Deserialize path
            while (i < c_array.Size() && (c = c_array[i++]) != '|') {
                std::string node = "";
                while (i < c_array.Size() && (c = c_array[i++]) != ':') {
                    node += c;
                }
                if (node.empty()) throw std::runtime_error("Path node is empty");
                path.push_back(node);
            }
            element = "";
        }

        // Deserialize source ID
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Source ID is empty");
        source = element;
        element = "";

        // Deserialize position
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Position is empty");
        size_t pos = element.find(",");
        if (pos == std::string::npos) throw std::runtime_error("Invalid position format");
        position.SetX(std::stod(element.substr(0, pos)));
        position.SetY(std::stod(element.substr(pos + 1)));
        element = "";

        // Deserialize destination (if RREQ/RREP or unicast)
        if (mode == RREQ || mode == RREP || mode == UNICAST) {
            // Deserialize destination
            while (i < c_array.Size() && (c = c_array[i++]) != '|') {
                element += c;
            }
            if (element.empty()) throw std::runtime_error("Destination is empty");
            destination = element;
            element = "";
        }

        // Deserialize payload (if broadcast or unicast)
        if (mode == BROADCAST || mode == UNICAST || mode == RERR) {
            while (i < c_array.Size()) {
                while (i < c_array.Size() && (c = c_array[i++]) != '|') {
                    element += c;
                }
                if (element.empty()) throw std::runtime_error("Payload is empty");
                payload += element;
                element = "";
            }
        }

        // Deserialize the packet sequence number
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Sequence number is empty");
        seq_num = std::stoi(element);

    } catch (const std::exception& e) {
        LOGERR << "Error deserializing byte array: " << e.what() << std::endl;
        exit(1);
    }
}

void CDynamicSourceRouting::Packet::GeneratePacketID(const std::string hash_function) {

    CByteArray c_array = Serialize();
    std::string packetStr = std::string(reinterpret_cast<const char*>(c_array.ToCArray()), c_array.Size());
    std::string packetSubStr;
    const std::set<int> noPathModes = {BROADCAST, RERR, NREQ, NREP}; // Modes that do not have a path and are serialized differently
    size_t delimeterCount = (noPathModes.count(mode)) ? 2 : 3;
    size_t pos = 0;

    for (size_t i = 0; i < delimeterCount; i++){
        pos = packetStr.find("|", pos);
        if (pos == std::string::npos) {
            LOGERR << "Error generating packet ID for broadcast packet: "<< packetStr << std::endl;
            LOGERR << "Could not parse packet string." << std::endl;
            exit(1);
        }
        ++pos;
    }
    packetSubStr = packetStr.substr(pos);
    
    if (hash_function == "fnv1a") {
        packet_id = FNV1aHash(packetSubStr);
    } else if (hash_function == "djb2") {
        packet_id = DJB2Hash(packetSubStr);
    } else {
        LOGERR << "Invalid hash function specified for generating packet ID. [Use 'FNV1a' or 'DJB2']" << std::endl;
        exit(1);
    }
}