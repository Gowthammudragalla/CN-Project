#ifndef AERIAL_RELAY_MANAGER_CC
#define AERIAL_RELAY_MANAGER_CC

#include "aerial-relay-manager.h"
#include "aerial-relay-application.h"

#include <iostream>
#include <iomanip>
#include <cmath>

using namespace ns3;

TypeId
AerialRelayManager::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::AerialRelayManager")
    .SetParent<Object>()
    .SetGroupName("FutureNetAerial")
    .AddConstructor<AerialRelayManager>()
    .AddTraceSource("AerialPositionUpdate",
                    "Trace called when an aerial node position is updated",
                    MakeTraceSourceAccessor(&AerialRelayManager::m_aerialPositionUpdateTrace),
                    "ns3::AerialRelayManager::AerialPositionUpdateTracedCallback")
    .AddTraceSource("RelayCandidateChange",
                    "Trace called when candidate eligibility changes",
                    MakeTraceSourceAccessor(&AerialRelayManager::m_relayCandidateChangeTrace),
                    "ns3::AerialRelayManager::RelayCandidateChangeTracedCallback")
    .AddTraceSource("RelaySelectionChange",
                    "Trace called when selected relay changes",
                    MakeTraceSourceAccessor(&AerialRelayManager::m_relaySelectionChangeTrace),
                    "ns3::AerialRelayManager::RelaySelectionChangeTracedCallback")
    .AddTraceSource("RelayUnavailable",
                    "Trace called when selected relay becomes unavailable",
                    MakeTraceSourceAccessor(&AerialRelayManager::m_relayUnavailableTrace),
                    "ns3::AerialRelayManager::RelayUnavailableTracedCallback");
  return tid;
}

AerialRelayManager::AerialRelayManager()
  : m_sourceNode(nullptr),
    m_destNode(nullptr),
    m_selectedRelayId(-1),
    m_decisionInterval(Seconds(1.0)),
    m_minRelayDwellTime(Seconds(2.0)),
    m_lastSelectionTime(Seconds(0.0)),
    m_csvFileStream(nullptr)
{
  m_linkModel = CreateObject<AerialLinkModel>();
  m_policy = CreateObject<NearestRelayPolicy>();
}

AerialRelayManager::~AerialRelayManager()
{
}

void
AerialRelayManager::SetLinkModel(Ptr<AerialLinkModel> linkModel)
{
  m_linkModel = linkModel;
}

Ptr<AerialLinkModel>
AerialRelayManager::GetLinkModel() const
{
  return m_linkModel;
}

void
AerialRelayManager::SetPolicy(Ptr<RelaySelectionPolicy> policy)
{
  m_policy = policy;
}

Ptr<RelaySelectionPolicy>
AerialRelayManager::GetPolicy() const
{
  return m_policy;
}

void
AerialRelayManager::SetDecisionInterval(Time interval)
{
  m_decisionInterval = interval;
}

Time
AerialRelayManager::GetDecisionInterval() const
{
  return m_decisionInterval;
}

void
AerialRelayManager::SetMinRelayDwellTime(Time dwellTime)
{
  m_minRelayDwellTime = dwellTime;
}

Time
AerialRelayManager::GetMinRelayDwellTime() const
{
  return m_minRelayDwellTime;
}

void
AerialRelayManager::RegisterRelay(Ptr<Node> relayNode)
{
  NS_ASSERT_MSG(relayNode != nullptr, "Cannot register null relay node");
  m_relays.push_back(relayNode);
  m_candidateEligibility[relayNode->GetId()] = false;
}

void
AerialRelayManager::RegisterEndpoint(Ptr<Node> endpointNode)
{
  NS_ASSERT_MSG(endpointNode != nullptr, "Cannot register null endpoint node");
  m_endpoints.push_back(endpointNode);
}

void
AerialRelayManager::SetEndpoints(Ptr<Node> source, Ptr<Node> dest)
{
  NS_ASSERT_MSG(source != nullptr && dest != nullptr, "Source and destination must be non-null");
  m_sourceNode = source;
  m_destNode = dest;
  RegisterEndpoint(source);
  RegisterEndpoint(dest);
}

int32_t
AerialRelayManager::GetSelectedRelay(uint32_t endpointId) const
{
  return m_selectedRelayId;
}

int32_t
AerialRelayManager::GetSelectedRelayId() const
{
  return m_selectedRelayId;
}

void
AerialRelayManager::SetCsvStream(std::shared_ptr<std::ofstream> csvStream)
{
  m_csvFileStream = csvStream;
  if (m_csvFileStream && m_csvFileStream->is_open())
  {
    *m_csvFileStream << "time_s,node_id,role,x_m,y_m,z_m,speed_mps,selected_relay,available\n";
    m_csvFileStream->flush();
  }
}

void
AerialRelayManager::Start()
{
  EvaluateRelays();
  m_evalEvent = Simulator::Schedule(m_decisionInterval, &AerialRelayManager::PeriodicEvaluate, this);
}

void
AerialRelayManager::Stop()
{
  if (m_evalEvent.IsPending())
  {
    m_evalEvent.Cancel();
  }
}

void
AerialRelayManager::DoDispose()
{
  Stop();
  m_sourceNode = nullptr;
  m_destNode = nullptr;
  m_relays.clear();
  m_endpoints.clear();
  m_candidateEligibility.clear();
  m_linkModel = nullptr;
  m_policy = nullptr;
  m_csvFileStream = nullptr;
  Object::DoDispose();
}

void
AerialRelayManager::EvaluateRelays()
{
  if (!m_sourceNode || !m_destNode)
  {
    return;
  }

  uint32_t srcId = m_sourceNode->GetId();
  uint32_t dstId = m_destNode->GetId();

  std::vector<RelayState> candidates;

  // 1. Evaluate candidate eligibility and fire traces
  for (const auto& relayNode : m_relays)
  {
    uint32_t rId = relayNode->GetId();
    bool currentlyEligible = m_linkModel->IsEligible(srcId, rId, dstId);

    // Track eligibility change
    if (m_candidateEligibility.find(rId) != m_candidateEligibility.end())
    {
      if (m_candidateEligibility[rId] != currentlyEligible)
      {
        m_candidateEligibility[rId] = currentlyEligible;
        m_relayCandidateChangeTrace(srcId, rId, currentlyEligible);
      }
    }
    else
    {
      m_candidateEligibility[rId] = currentlyEligible;
    }

    // Fire position update trace
    Ptr<MobilityModel> mob = relayNode->GetObject<MobilityModel>();
    if (mob)
    {
      m_aerialPositionUpdateTrace(rId, mob->GetPosition());
    }

    if (currentlyEligible)
    {
      RelayState state;
      state.relayId = rId;
      state.node = relayNode;
      state.position = mob ? mob->GetPosition() : Vector(0, 0, 0);
      state.distanceToSource = m_linkModel->GetDistance(m_sourceNode, relayNode);
      state.distanceToDest = m_linkModel->GetDistance(relayNode, m_destNode);
      state.totalDistance = state.distanceToSource + state.distanceToDest;
      state.estimatedDelay = m_linkModel->GetEstimatedPathDelay(m_sourceNode, relayNode, m_destNode);

      Ptr<AerialNodeModel> nodeModel = relayNode->GetObject<AerialNodeModel>();
      state.residualEnergy = nodeModel ? nodeModel->GetResidualEnergyFraction() : 1.0;

      candidates.push_back(state);
    }
  }

  // 2. Check if the currently selected relay has become unavailable
  bool currentStillEligible = false;
  if (m_selectedRelayId != -1)
  {
    for (const auto& cand : candidates)
    {
      if (static_cast<int32_t>(cand.relayId) == m_selectedRelayId)
      {
        currentStillEligible = true;
        break;
      }
    }

    if (!currentStillEligible)
    {
      std::cout << "[Time " << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                << "s] [UNAVAILABLE] Selected Relay (Node " << m_selectedRelayId
                << ") is OUT OF RANGE!" << std::endl;
      m_relayUnavailableTrace(srcId, static_cast<uint32_t>(m_selectedRelayId));
    }
  }

  // 3. Execute RelaySelectionPolicy
  RelayRequest request{srcId, dstId};
  RelayDecision decision = m_policy->Select(candidates, request);
  int32_t preferredRelayId = decision.hasRelay ? decision.selectedRelayId : -1;

  Time now = Simulator::Now();

  // 4. Enforce minimum dwell time:
  // Switch immediately if current relay is UNAVAILABLE (!currentStillEligible).
  // Otherwise, suppress switch until dwell time has elapsed.
  int32_t finalRelayId = preferredRelayId;
  if (currentStillEligible && preferredRelayId != m_selectedRelayId)
  {
    if (now - m_lastSelectionTime < m_minRelayDwellTime)
    {
      finalRelayId = m_selectedRelayId; // Stay on current relay
    }
  }

  // 5. Perform Handover if changed
  if (finalRelayId != m_selectedRelayId)
  {
    int32_t oldRelay = m_selectedRelayId;
    m_selectedRelayId = finalRelayId;
    m_lastSelectionTime = now;

    UpdateRelayAppStates();

    std::cout << ">>> [Time " << std::fixed << std::setprecision(2) << now.GetSeconds()
              << "s] RELAY HANDOVER: Old Relay = " << oldRelay
              << " -> New Relay = " << m_selectedRelayId
              << " (Metric: " << decision.metricValue << ") <<<" << std::endl;

    m_relaySelectionChangeTrace(srcId, oldRelay, m_selectedRelayId);
  }

  // 6. Record to CSV if configured
  LogCsvRecord();
}

void
AerialRelayManager::UpdateRelayAppStates()
{
  for (auto& relay : m_relays)
  {
    for (uint32_t a = 0; a < relay->GetNApplications(); ++a)
    {
      Ptr<AerialRelayApplication> app = DynamicCast<AerialRelayApplication>(relay->GetApplication(a));
      if (app)
      {
        app->SetActiveRelay(static_cast<int32_t>(relay->GetId()) == m_selectedRelayId);
      }
    }
  }
}

void
AerialRelayManager::LogCsvRecord()
{
  if (!m_csvFileStream || !m_csvFileStream->is_open())
  {
    return;
  }

  double nowSec = Simulator::Now().GetSeconds();

  for (const auto& relayNode : m_relays)
  {
    uint32_t nId = relayNode->GetId();
    Ptr<AerialNodeModel> am = relayNode->GetObject<AerialNodeModel>();
    std::string roleStr = am ? RoleToString(am->GetRole()) : "UAV";

    Ptr<MobilityModel> mob = relayNode->GetObject<MobilityModel>();
    Vector pos = mob ? mob->GetPosition() : Vector(0, 0, 0);
    Vector vel = mob ? mob->GetVelocity() : Vector(0, 0, 0);
    double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);

    bool isAvailable = m_candidateEligibility[nId];

    *m_csvFileStream << std::fixed << std::setprecision(2)
                     << nowSec << ","
                     << nId << ","
                     << roleStr << ","
                     << std::setprecision(1)
                     << pos.x << ","
                     << pos.y << ","
                     << pos.z << ","
                     << speed << ","
                     << m_selectedRelayId << ","
                     << (isAvailable ? 1 : 0) << "\n";
  }
  m_csvFileStream->flush();
}

void
AerialRelayManager::PeriodicEvaluate()
{
  EvaluateRelays();
  m_evalEvent = Simulator::Schedule(m_decisionInterval, &AerialRelayManager::PeriodicEvaluate, this);
}

#endif // AERIAL_RELAY_MANAGER_CC
