#include "rab_dsr.h"

CDynamicSourceRouting::CDynamicSourceRouting(const std::string& robotID) : m_nSequenceNumber(0), m_sRobotID(robotID) {}

CDynamicSourceRouting::~CDynamicSourceRouting() {}

void CDynamicSourceRouting::Init(CCI_RangeAndBearingSensor* pcRABSens, CCI_RangeAndBearingActuator* pcRABActuator) {
    m_pcRABSens = pcRABSens;
    m_pcRABActuator = pcRABActuator;
    m_nPacketSize = m_pcRABActuator->GetPacketSize();
}

void CDynamicSourceRouting::ListenAndUpdate() {
    // Handle periodic tasks such as route maintenance
    RouteMaintenance();
}

void CDynamicSourceRouting::Send(const Packet& packet) {

    if (packet.GetMode() == 0) {
        // Broadcast the message to all neighbors
        m_pcRABActuator->SetData(packet.Serialize());
    }
    // TODO: Need to clarify the difference between Send() and ForwardPacket()
    
    // TODO: Not sure if this is done here or in ForwardPacket()
    // Initiate route discovery if necessary and send the message
    if (m_mapRoutingTable.find(packet.GetDestination()) == m_mapRoutingTable.end()) {
        RouteDiscovery(packet.GetDestination());
    }
    if (m_mapRoutingTable.find(packet.GetDestination()) != m_mapRoutingTable.end()) {
        ForwardPacket(packet, m_mapRoutingTable[packet.GetDestination()].path);
    }
}

std::vector<Packet> CDynamicSourceRouting::Receive() {
    // Get and deserialize received packets
    std::vector<Packet> receivedPackets;
    const CCI_RangeAndBearingSensor::TReadings& tReadings = m_pcRABSens->GetReadings();
    for (const auto& tReading : tReadings) {
        receivedPackets.push_back(Packet(tReading.Data));
    }
    
    // Do background processing and return packets that are only for this robot that haven't already been processed.
    return ProcessPackets(receivedPackets);
}

std::vector<Packet> CDynamicSourceRouting::ProcessPackets(const std::vector<Packet>& packetList) {

    std::vector<Packet> filteredPackets; // Packets that are for this Robot (e.g. broadcast or unicast to this robot's ID)

    for (const auto& packet : packetList) {

        bool alreadyProcessed = m_setReceivedPackets.find(packet.GetPacketID()) != m_setReceivedPackets.end();
        
        switch(packet.GetMode()){

            case 0: // Broadcast
                if (!alreadyProcessed) filteredPackets.push_back(packet);
                break;

            case 1: // Unicast
                if (!alreadyProcessed) packet.GetDestination() == m_sRobotID ? filteredPackets.push_back(packet) : ForwardPacket(packet);
                break;

            case 2: // Route Request
                if (packet.GetDestination() == m_sRobotID) {
                    
                    packet.SetPath(packet.GetPath().push_back(m_sRobotID));     // new path = old path + this robot's ID
                    
                    
                } else {
                    packet.SetPath(packet.GetPath().push_back(m_sRobotID));     // new path = old path + this robot's ID
                    // Forward the RREQ packet
                    ForwardPacket(packet);
                }
                break;

            case 3: // Route Reply
                if (packet.GetSource() == m_sRobotID) {

                    // Update the routing table
                    m_mapRoutingTable[packet.GetDestination()] = RouteEntry(packet.GetPath());
                } else {
                    ForwardPacket(packet);
                }
                break;
            
            default:
                LOGERR << "Invalid packet mode received: " << packet.GetMode() << std::endl;
                break;
        } 

    }
}

void CDynamicSourceRouting::StartRouteDiscovery(const std::string& destination) {
    // TODO: Implement route discovery process
}

void CDynamicSourceRouting::StartRouteMaintenance() {
    // TODO: Implement route maintenance process
}

void CDynamicSourceRouting::ForwardPacket(const Packet& packet) {
    // TODO: Implement packet forwarding process
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

CDynamicSourceRouting::Packet::Packet() : mode(-1), hop_count(0), path(), source(""), position(CVector2()), destination(""), payload(""), seq_num(-1), packet_id(0) {}

CDynamicSourceRouting::Packet::Packet(  size_t& mode, const std::string& source, const CVector2& position, 
                                        const std::string& destination = "", const std::string& payload) {
    this->mode = mode;
    this->hop_count = 0; // Initialize hop count to 0
    this->path = std::vector<RouteEntry>(); // Initialize path to empty
    this->source = source;
    this->position = position;
    this->destination = destination;
    this->payload = payload;
    this->seq_num = m_nSequenceNumber++; // Generate a unique packet sequence number
    GeneratePacketID(); // Generate a unique packet ID using the default hash function
}

CDynamicSourceRouting::Packet::Packet(const CByteArray& c_array) {
    Deserialize(c_array);
    GeneratePacketID();
}

CByteArray CDynamicSourceRouting::Packet::Serialize(size_t packetSize = m_nPacketSize) const {
    CByteArray c_array;

    // Serialize mode
    c_array << static_cast<UInt8>(mode) << "|";

    // Serialize hop count
    c_array << static_cast<UInt8>(hop_count) << "|";

    // Serialize path (if RREQ/RREP or unicast)
    if (mode == 1 || mode == 2 || mode == 3) {
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
    if (mode == 1 || mode == 2 || mode == 3) {

        // Serialize destination
        for (char c : destination) {
            c_array << static_cast<UInt8>(c) << "|";
        }
    }

    // Serialize payload (if broadcast or unicast)
    if (mode == 0 || mode == 1) {
        for (char c : payload) {
            c_array << static_cast<UInt8>(c) << "|";
        }
    }

    // Serialize the packet sequence number
    c_array << static_cast<UInt32>(seq_num) << "|";

    if (c_array.Size() > packetSize) {
        LOGERR << "Packet size exceeds maximum size of " << packetSize << " bytes specified for RAB Actuator in .xml/.argos file. [Adjust packet size]" << std::endl;
        exit(1);
    } else {
        // Fill the rest of the packet with empty space.
        for(size_t i = c_array.Size(); i < packetSize; ++i) {
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
        mode = std::stoi(element);
        element = "";

        // Deserialize hop count
        while (i < c_array.Size() && (c = c_array[i++]) != '|') {
            element += c;
        }
        if (element.empty()) throw std::runtime_error("Hop count is empty");
        hop_count = std::stoi(element);
        element = "";

        // Deserialize path (if RREQ/RREP or unicast)
        if (mode == 1 || mode == 2 || mode == 3) {
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
        if (mode == 1 || mode == 2 || mode == 3) {
            // Deserialize destination
            while (i < c_array.Size() && (c = c_array[i++]) != '|') {
                element += c;
            }
            if (element.empty()) throw std::runtime_error("Destination is empty");
            destination = element;
            element = "";
        }

        // Deserialize payload (if broadcast or unicast)
        if (mode == 0 || mode == 1) {
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

    } catch (const std::exception& e) {
        LOGERR << "Error deserializing byte array: " << e.what() << std::endl;
        exit(1);
    }
}

void CDynamicSourceRouting::Packet::GeneratePacketID(const std::string hash_function = "fnv1a") {

    CByteArray c_array = Serialize();
    std::string packetStr = std::string(reinterpret_cast<const char*>(c_array.ToCArray()), c_array.Size());
    std::string packetSubStr;
    size_t delimeterCount = (mode == 0) ? 2 : 3;
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