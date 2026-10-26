// Extracted from m03-aerial-relay.cc.

// 4. AerialRelayApplication (Packet forwarding on Relays) (M03-O4)
// ===========================================================================

class AerialRelayApplication : public Application
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::AerialRelayApplication")
      .SetParent<Application>()
      .SetGroupName("FutureNetAerial")
      .AddConstructor<AerialRelayApplication>();
    return tid;
  }

  AerialRelayApplication()
    : m_port(9000),
      m_isActiveRelay(false),
      m_packetsForwarded(0),
      m_packetsDropped(0)
  {
  }

  void Setup(uint16_t listenPort, Address destAddress)
  {
    m_port = listenPort;
    m_destAddress = destAddress;
  }

  void SetActiveRelay(bool active)
  {
    m_isActiveRelay = active;
  }

  bool IsActiveRelay() const
  {
    return m_isActiveRelay;
  }

  uint32_t GetPacketsForwarded() const { return m_packetsForwarded; }
  uint32_t GetPacketsDropped() const { return m_packetsDropped; }

private:
  void StartApplication() override
  {
    m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_recvSocket->Bind(local);
    m_recvSocket->SetRecvCallback(MakeCallback(&AerialRelayApplication::HandleRead, this));

    m_sendSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  }

  void StopApplication() override
  {
    if (m_recvSocket)
    {
      m_recvSocket->Close();
      m_recvSocket = nullptr;
    }
    if (m_sendSocket)
    {
      m_sendSocket->Close();
      m_sendSocket = nullptr;
    }
  }

  void HandleRead(Ptr<Socket> socket)
  {
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from)))
    {
      if (m_isActiveRelay)
      {
        m_packetsForwarded++;
        std::cout << "  [Time " << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                  << "s] Relay (Node " << GetNode()->GetId()
                  << ") FORWARDED packet (" << packet->GetSize() << " bytes) towards destination." << std::endl;
        m_sendSocket->SendTo(packet, 0, m_destAddress);
      }
      else
      {
        m_packetsDropped++;
        std::cout << "  [Time " << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                  << "s] Relay (Node " << GetNode()->GetId()
                  << ") DROPPED packet (inactive relay)." << std::endl;
      }
    }
  }

  uint16_t m_port;
  Address m_destAddress;
  bool m_isActiveRelay;
  uint32_t m_packetsForwarded;
  uint32_t m_packetsDropped;
  Ptr<Socket> m_recvSocket;
  Ptr<Socket> m_sendSocket;
};

// ===========================================================================
