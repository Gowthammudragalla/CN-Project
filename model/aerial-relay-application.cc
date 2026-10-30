#ifndef AERIAL_RELAY_APPLICATION_CC
#define AERIAL_RELAY_APPLICATION_CC

#include "aerial-relay-application.h"

#include <iostream>
#include <iomanip>

using namespace ns3;

TypeId
AerialRelayApplication::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::CNProjectAerialRelayApplication")
    .SetParent<Application>()
    .SetGroupName("FutureNetAerial")
    .AddConstructor<AerialRelayApplication>();
  return tid;
}

AerialRelayApplication::AerialRelayApplication()
  : m_port(9000),
    m_isActiveRelay(false),
    m_packetsForwarded(0),
    m_packetsDropped(0),
    m_recvSocket(nullptr),
    m_sendSocket(nullptr)
{
}

AerialRelayApplication::~AerialRelayApplication()
{
}

void
AerialRelayApplication::Setup(uint16_t listenPort, Address destAddress)
{
  m_port = listenPort;
  m_destAddress = destAddress;
}

void
AerialRelayApplication::SetActiveRelay(bool active)
{
  m_isActiveRelay = active;
}

bool
AerialRelayApplication::IsActiveRelay() const
{
  return m_isActiveRelay;
}

uint32_t
AerialRelayApplication::GetPacketsForwarded() const
{
  return m_packetsForwarded;
}

uint32_t
AerialRelayApplication::GetPacketsDropped() const
{
  return m_packetsDropped;
}

void
AerialRelayApplication::SetDestinationAddress(Address destAddress)
{
  m_destAddress = destAddress;
}

Address
AerialRelayApplication::GetDestinationAddress() const
{
  return m_destAddress;
}

void
AerialRelayApplication::SetPort(uint16_t port)
{
  m_port = port;
}

uint16_t
AerialRelayApplication::GetPort() const
{
  return m_port;
}

void
AerialRelayApplication::StartApplication()
{
  NS_ASSERT_MSG(GetNode() != nullptr, "AerialRelayApplication requires an aggregated Node");

  if (!m_recvSocket)
  {
    m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
    m_recvSocket->Bind(local);
    m_recvSocket->SetRecvCallback(MakeCallback(&AerialRelayApplication::HandleRead, this));
  }

  if (!m_sendSocket)
  {
    m_sendSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  }
}

void
AerialRelayApplication::StopApplication()
{
  if (m_recvSocket)
  {
    m_recvSocket->SetRecvCallback(MakeNullCallback<void, Ptr<Socket>>());
    m_recvSocket->Close();
    m_recvSocket = nullptr;
  }
  if (m_sendSocket)
  {
    m_sendSocket->Close();
    m_sendSocket = nullptr;
  }
}

void
AerialRelayApplication::DoDispose()
{
  StopApplication();
  Application::DoDispose();
}

void
AerialRelayApplication::HandleRead(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom(from)))
  {
    if (m_isActiveRelay)
    {
      if (m_sendSocket)
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
      }
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

#endif // AERIAL_RELAY_APPLICATION_CC
