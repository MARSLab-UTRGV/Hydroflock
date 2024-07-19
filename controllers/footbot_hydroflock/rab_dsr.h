#ifndef RAB_DSR_H
#define RAB_DSR_H

#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <map>
#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <set>
#include <iostream>

using namespace argos;

// TODO: Documentation for README
/**
 *      Must note that the terms "path" and "route" may be used interchangeably but in most cases,
 * "path" refers to an unfinalized route. However the finalized route is still stored in the `path`
 *  variable of the Packet struct but is treated as a route for unicast and broadcast.
 * 
 */

/**
 * Dynamic Source Routing implementation using the Range and Bearing system
 */
class CDynamicSourceRouting {
public:

    enum PacketState{
        UNINITIALIZED,
        QUEUED_FOR_PROCESSING,             
        WAITING_FOR_ROUTE_DISCOVERY,
        READY,
        URGENT,
        SENT
    };

    enum PacketMode{
        NONE,       // Uninitialized mode
        BROADCAST,
        UNICAST,
        RREQ,
        RREP,
        RERR,
        NREQ,
        NREP
    };

    typedef std::vector<std::string> Path;

    /**
    * @brief The Packet struct represents a packet used to create packets to send using the RAB or
    * parse packets for data.
    * 
    * This struct contains information about the packet, including its ID, hop count, and data.
    * It also provides methods to convert the packet to and from a CByteArray for the RAB Actuator.
    * 
    * By default it uses broadcast mode (mode = 0). Call `SetUnicast()` to change to unicast mode.
    */
    struct Packet {
        public:

            /**
             * @brief Default constructor for the Packet struct.
             */
            Packet();

            /**
             * @brief Constructor for the Packet struct. Creates and initializes a Packet object with the specified parameters.
             * 
             * @param source The source node (robot) ID.
             * @param pos The position of the original sender.
             * @param mode The send mode of the packet.
             * @param payload The payload of the packet.
             */
            Packet( const PacketMode& mode, const std::string& source, const CVector2& position, const std::string& destination = "", const std::string& payload = "");

            /**
             * @brief Constructs a Packet object from a CByteArray.
             * 
             * This constructor takes a CByteArray as input and parses its content to initialize the Packet object.
             * 
             * The intended use is to convert the byte array received from the RAB Sensor into a Packet object to organize
             * and use the received data.
             * 
             * If the content is in the expected format, the 
             * 
             * members of the Packet object are set accordingly.
             * 
             * If the content is not in the expected format, an error message is logged.
             * 
             * @param c_array The CByteArray to parse and construct the Packet object from.
             */

            Packet(const CByteArray& c_array);

            void SetSource(const std::string& source) { this->source = source; }
            void SetDestination (const std::string& destination) { this->destination = destination; }
            void SetPosition(const CVector2& position) { this->position = position; };
            void SetUnicast(std::string& destination) { mode = UNICAST; this->destination = destination;};
            void SetPayload(const std::string& payload) { this->payload = payload;};
            void SetRREQ() { mode = RREQ; };
            void SetRREP() { mode = RREP; };
            void SetRERR(std::string node1, std::string node2) { mode = RERR; this->payload = node1 + "-" + node2;};
            void SetNREQ() { mode = NREQ; };
            void SetNREP() { mode = NREP; };
            void SetPath(const Path& path) { this->path = path; };
            void SetSeqNum(size_t seq_num) { this->seq_num = seq_num; };
            void ResetHopCount() { hop_count = 0; };
            void IncrementHopCount() { hop_count++; };
            void SetState(PacketState state) { this->state = state; };

            std::string GetSource() const { return source; }
            std::string GetDestination() const { return destination; }
            CVector2 GetPosition() const { return position; }
            PacketMode GetMode() const { return mode; }
            std::string GetPayload() const { return payload; }
            std::uint32_t GetPacketID() const { return packet_id; }
            Path GetPath() const { return path; }
            size_t GetSeqNum() const { return seq_num; }
            size_t GetHopCount() const { return hop_count; }
            PacketState GetState() const { return state; }

            /**
             * @brief Converts the object to a byte array.
             * 
             * This function serializes the object and returns a byte array representation of it.
             * 
             * @return A byte array representation of the object.
             */
            CByteArray ToByteArray() const { return Serialize(); }

        private:

            PacketMode mode; /** The send mode of the packet (0 = broadcast, 1 = unicast, 2 = route request, 3 = route reply, 4 = route error, 5 = neighbor request, 6 = neighbor reply). */
            size_t hop_count; /** The hop count of the packet. Use for unicast and route reply as an index pointing to who is next in the route. */
            Path path; /** The path to take for unicast packets or to be updated and used by RREQ and RREP packets. */
            std::string source; /** The ID of the sender. */
            CVector2 position;  /** The position of the packet sender. */
            std::string destination; /** The destination for the packet if not broadcasted. */
            std::string payload; /** The payload of the packet. */
            size_t seq_num; /** The sequence number of the packet. A locally-unique identifier for the packet */

            uint32_t packet_id; /** A unique ID for the packet. */
            PacketState state; /** The state of the packet (e.g., waiting, ready, etc.) */
            
            /**
             * @brief Converts the payload of the object into a byte array of the specified packet size.
             * The `packet_size` should match the data size specified for the RAB Actuator.
             *
             * @param packet_size The size of the packet to be created. By default, it is already set to the RAB packet size
             * specified in the xml/argos file.
             * @return A `CByteArray` object containing the converted payload.
             */
            CByteArray Serialize() const;

            /**
             * @brief Deserializes the data from a byte array.
             *
             * This function is responsible for deserializing the byte array and updating the internal state of the object accordingly.
             *
             * @param c_array The byte array containing the serialized data.
             */
            void Deserialize(const CByteArray& c_array);

            /**
             * @brief Generates a packet ID using the specified hash function.
             * 
             * It is a hash of the source, position, payload, and sequence number for broadcast packets OR
             * the source, position, destination, payload, and sequence number for unicast, RREQ, and RREP packets.
             *
             * This function generates a packet ID using the specified hash function.
             * The default hash function used is FNV1a.
             * 
             * The `hash_function` parameter options are "fnv1a" and "djb2". 
             *
             * @param hash_function The hash function to use for generating the packet ID.
             *                      Default value is "fnv1a".
             */
            void GeneratePacketID(const std::string hash_function = "fnv1a");

        
   };

   struct RDInfo {
        bool awaitingRREP;
        std::vector<Packet> waitingPackets;

        RDInfo() : awaitingRREP(false) {}
   };

    /**
     * @brief Constructs a CDynamicSourceRouting object.
     */
    CDynamicSourceRouting(const std::string& robotID);
    ~CDynamicSourceRouting();

    // TODO: Document Init()
    void Init(CCI_RangeAndBearingSensor* pcRABSens, CCI_RangeAndBearingActuator* pcRABActuator);
    // TODO: Document ListenAndUpdate()
    std::vector<Packet> ListenAndUpdate();
    // TODO: Document AddToQueue()
    void AddToQueue(Packet& packet);
    // TODO: Document GetNeighborCount()
    size_t GetNeighborCount() const { return m_setNeighborList.size(); }


private:

    // TODO: Document member variables
    CCI_RangeAndBearingSensor* m_pcRABSens;
    CCI_RangeAndBearingActuator* m_pcRABActuator;
    std::map<std::string, Path> m_mapRoutingTable;
    static size_t m_nPacketSize;
    const std::string m_sRobotID;
    static size_t m_nSequenceNumber;
    std::unordered_set<uint32_t> m_setReceivedPackets;
    std::unordered_set<std::string> m_setNeighborList;  // List of neighbors
    std::queue<Packet> m_qOutPacketReadyQueue; // Outbound packets ready to be sent
    std::queue<Packet> m_qOutPacketPrepQueue; // Outbound packets waiting to be processed
    std::map<std::string, RDInfo> m_mapPacketsAwaitingRD; // Packets awaiting a route discovery reply
    std::queue<Packet> m_qOutPacketUrgentQueue; // Outbound packets that need to be sent immediately (e.g. RERR packets)

    // TODO: Document ProcessOutboundPackets()
    void ProcessOutboundPackets();

    // TODO: Document StartRouteDiscovery()
    void StartRouteDiscovery(const std::string& destination);

    // TODO: Document StartRouteMaintenance()
    void StartRouteMaintenance();

    // TODO: Document PacketForwarding()
    void PacketForwarding();

    //TODO: Document Send()
    void Send();

    // TODO: Document Receive()
    std::vector<Packet> Receive(); 

    // TODO: Document ProcessInboundPackets()
    std::vector<Packet> ProcessInboundPackets(std::vector<Packet>& packetList);

    // TODO: Document NeighborRequest()
    void NeighborRequest(const CVector2& myPosition);

    // TODO: Document IsOnRoute()
    bool IsOnRoute(const Packet& packet, const std::string& robotID = "");
    
    // TODO: Document HasEdgeOnRoute()
    bool HasEdgeOnRoute(const Path& route, const std::string& nodeA, const std::string& nodeB);

    // TODO: Document ParseRERRPayload()
    std::pair<std::string,std::string> ParseRERRPayload(const std::string& payload);


    /**
     * Calculates the hash value for a given key using the Fowler-Noll-Vo, variant 1a hash algorithm.
     * 
     * The FNV-1a hash algorithm is a simple and efficient hash function that produces a 32-bit hash value.
     *
     * @param key The key to calculate the hash value for.
     * @return The FNV-1a hash value for the key.
     */
    static uint32_t FNV1aHash(const std::string& key);

    /**
     * @brief Calculates the hash value for a given key using the Daniel J. Bernstein hash algorithm.
     *
     * The DJB2 hash algorithm is a simple and efficient hash function that produces a 32-bit hash value.
     *
     * @param key The string key for which the hash value is calculated.
     * @return The DJB2 hash value of the key.
     */
    static uint32_t DJB2Hash(const std::string& key);
};

#endif
