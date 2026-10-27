#ifndef AERIAL_RELAY_APPLICATION_H
#define AERIAL_RELAY_APPLICATION_H

#include "ns3/application.h"
#include "ns3/address.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

/**
 * \ingroup futurenet-aerial
 * \brief Application installed on aerial nodes to receive and forward packets
 *
 * AerialRelayApplication listens on a specified UDP port. When packets are received,
 * it forwards them to the configured destination address if the node is currently the
 * active relay. If the relay is inactive, packets are dropped and tracked.
 */
class AerialRelayApplication : public Application
{
public:
  static TypeId GetTypeId(void);

  AerialRelayApplication();
  ~AerialRelayApplication() override;

  /**
   * \brief Configure the listen port and forwarding destination address
   * \param listenPort Port to listen on for incoming datagrams
   * \param destAddress Destination address for forwarded packets
   */
  void Setup(uint16_t listenPort, Address destAddress);

  /**
   * \brief Set whether this relay is currently active
   * \param active True if relay is active and should forward packets
   */
  void SetActiveRelay(bool active);

  /**
   * \brief Check if this relay is currently active
   * \return True if active
   */
  bool IsActiveRelay() const;

  /**
   * \brief Get total count of forwarded packets
   * \return Count of forwarded packets
   */
  uint32_t GetPacketsForwarded() const;

  /**
   * \brief Get total count of dropped packets (received while inactive)
   * \return Count of dropped packets
   */
  uint32_t GetPacketsDropped() const;

  /**
   * \brief Update the destination address
   * \param destAddress New destination address
   */
  void SetDestinationAddress(Address destAddress);

  /**
   * \brief Get the current destination address
   * \return Destination address
   */
  Address GetDestinationAddress() const;

  /**
   * \brief Set the listening port
   * \param port Listening port number
   */
  void SetPort(uint16_t port);

  /**
   * \brief Get the listening port
   * \return Listening port number
   */
  uint16_t GetPort() const;

protected:
  void StartApplication() override;
  void StopApplication() override;
  void DoDispose() override;

private:
  /**
   * \brief Callback invoked when a packet is received on the listening socket
   * \param socket Socket receiving the packet
   */
  void HandleRead(Ptr<Socket> socket);

  uint16_t m_port;
  Address m_destAddress;
  bool m_isActiveRelay;
  uint32_t m_packetsForwarded;
  uint32_t m_packetsDropped;
  Ptr<Socket> m_recvSocket;
  Ptr<Socket> m_sendSocket;
};

#endif // AERIAL_RELAY_APPLICATION_H
