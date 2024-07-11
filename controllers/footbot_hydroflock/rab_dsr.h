#ifndef RAB_DSR_H
#define RAB_DSR_H

#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <map>
#include <vector>
#include <string>
#include <unordered_set>

using namespace argos;

/**
 * Dynamic Source Routing implementation using the Range and Bearing system
 */
class CDynamicSourceRouting {
public:
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
             * @param mode The send mode of the packet (0 = broadcast, 1 = unicast).
             * @param payload The payload of the packet.
             */
            Packet( size_t& mode, const std::string& source, const CVector2& position, std::string& destination = "", const std::string& payload);

            /**
             * @brief Constructs a Packet object from a CByteArray.
             * 
             * This constructor takes a CByteArray as input and parses its content to initialize the Packet object.
             * 
             * The intended use is to convert the byte array received from the RAB Sensor into a Packet object to organize
             * and use the received data.
             * 
             * If the content is in the expected format, the members of the Packet object are set accordingly.
             * 
             * If the content is not in the expected format, an error message is logged.
             * 
             * @param c_array The CByteArray to parse and construct the Packet object from.
             */

            Packet(const CByteArray& c_array);

            void SetSource(const std::string& source) { this->source = source; }
            void SetDestination (const std::string& destination) { this->destination = destination; }
            void SetPosition(const CVector2& position) { this->position = position; };
            void SetUnicast(std::string& destination) { mode = 1; this->destination = destination;};
            void SetPayload(const std::string& payload) { this->payload = payload;};
            void SetRREQ() { mode = 2; };
            void SetRREP() { mode = 3; };
            void SetPath(const std::vector<RouteEntry>& path) { this->path = path; }

            std::string GetSource() const { return source; }
            std::string GetDestination() const { return destination; }
            CVector2 GetPosition() const { return position; }
            size_t GetMode() const { return mode; }
            std::string GetPayload() const { return payload; }
            std::uint32_t GetPacketID() const { return packet_id; }
            std::vector<RouteEntry> GetPath() const { return path; }

        private:

            size_t mode; /** The send mode of the packet (0 = broadcast, 1 = unicast, 2 = route request, 3 = route reply). */
            size_t hop_count; /** The hop count of the packet. */
            std::vector<RouteEntry> path; /** The path to take for unicast packets or to be updated and used by RREQ and RREP packets. */
            std::string source; /** The ID of the sender. */
            CVector2 position;  /** The position of the packet sender. */
            std::string destination; /** The destination for the packet if not broadcasted. */
            std::string payload; /** The payload of the packet. */
            size_t seq_num; /** The sequence number of the packet. A locally-unique identifier for the packet */

            uint32_t packet_id; /** A unique ID for the packet. */
            
            /**
             * @brief Converts the payload of the object into a byte array of the specified packet size.
             * The `packet_size` should match the data size specified for the RAB Actuator.
             *
             * @param packet_size The size of the packet to be created. By default, it is already set to the RAB packet size
             * specified in the xml/argos file.
             * @return A `CByteArray` object containing the converted payload.
             */
            CByteArray Serialize(size_t packetSize) const;

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

    /**
     * @brief Constructs a CDynamicSourceRouting object.
     */
    CDynamicSourceRouting(const std::string& robotID);
    ~CDynamicSourceRouting();

    void Init(CCI_RangeAndBearingSensor* pcRABSens, CCI_RangeAndBearingActuator* pcRABActuator);
    void ListenAndUpdate();
    void Send(const Packet& packet);


private:

    typedef std::vector<std::string> RouteEntry;

    CCI_RangeAndBearingSensor* m_pcRABSens;
    CCI_RangeAndBearingActuator* m_pcRABActuator;
    std::map<std::string, RouteEntry> m_mapRoutingTable;
    size_t m_nSequenceNumber;
    const size_t m_nPacketSize;
    const std::string m_sRobotID;
    std::unordered_set<uint32_t> m_setReceivedPackets;

    void StartRouteDiscovery(const std::string& destination);
    void StartRouteMaintenance();
    void ForwardPacket(const Packet& payload);

    // TODO: Document
    // Must note that this filters out duplicate packets, already processed packets, 
    // and background packets (packets that are being forwarded and used for route discovery and maintenance)
    std::vector<Packet> Receive(); 

    std::vector<Packet> ProcessPackets(const std::vector<Packet>& packetList);


    /**
     * Calculates the hash value for a given key using the Fowler-Noll-Vo, variant 1a hash algorithm.
     * 
     * The FNV-1a hash algorithm is a simple and efficient hash function that produces a 32-bit hash value.
     *
     * @param key The key to calculate the hash value for.
     * @return The FNV-1a hash value for the key.
     */
    uint32_t FNV1aHash(const std::string& key);

    /**
     * @brief Calculates the hash value for a given key using the Daniel J. Bernstein hash algorithm.
     *
     * The DJB2 hash algorithm is a simple and efficient hash function that produces a 32-bit hash value.
     *
     * @param key The string key for which the hash value is calculated.
     * @return The DJB2 hash value of the key.
     */
    uint32_t DJB2Hash(const std::string& key);
};

#endif
