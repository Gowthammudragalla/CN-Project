// Extracted from m03-aerial-relay.cc.

// 5. AerialRelayManager (M03-FR-06, M03-FR-07, Section 6 & 7)
// ===========================================================================

class AerialRelayManager : public Object
{
public:
  static TypeId GetTypeId(void)
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

  // SRS Trace Callbacks
  typedef void (*AerialPositionUpdateTracedCallback)(uint32_t nodeId, Vector position);
  typedef void (*RelayCandidateChangeTracedCallback)(uint32_t endpointId, uint32_t relayId, bool eligible);
  typedef void (*RelaySelectionChangeTracedCallback)(uint32_t endpointId, int32_t oldRelayId, int32_t newRelayId);
  typedef void (*RelayUnavailableTracedCallback)(uint32_t endpointId, uint32_t relayId);

  AerialRelayManager()
    : m_selectedRelayId(-1),
      m_decisionInterval(Seconds(1.0)),
      m_minRelayDwellTime(Seconds(2.0)),
      m_lastSelectionTime(Seconds(0.0)),
      m_csvFileStream(nullptr)
  {
    m_linkModel = CreateObject<AerialLinkModel>();
    m_policy = CreateObject<NearestRelayPolicy>();
  }

  void SetLinkModel(Ptr<AerialLinkModel> linkModel) { m_linkModel = linkModel; }
  Ptr<AerialLinkModel> GetLinkModel() const { return m_linkModel; }

  void SetPolicy(Ptr<RelaySelectionPolicy> policy) { m_policy = policy; }
  Ptr<RelaySelectionPolicy> GetPolicy() const { return m_policy; }

  void SetDecisionInterval(Time interval) { m_decisionInterval = interval; }
  void SetMinRelayDwellTime(Time dwellTime) { m_minRelayDwellTime = dwellTime; }

  void RegisterRelay(Ptr<Node> relayNode)
  {
    m_relays.push_back(relayNode);
    m_candidateEligibility[relayNode->GetId()] = false;
  }

  void RegisterEndpoint(Ptr<Node> endpointNode)
  {
    m_endpoints.push_back(endpointNode);
  }

  void SetEndpoints(Ptr<Node> source, Ptr<Node> dest)
  {
    m_sourceNode = source;
    m_destNode = dest;
    RegisterEndpoint(source);
    RegisterEndpoint(dest);
  }

  int32_t GetSelectedRelay(uint32_t endpointId) const
  {
    return m_selectedRelayId;
  }

  int32_t GetSelectedRelayId() const
  {
    return m_selectedRelayId;
  }

  void SetCsvStream(std::shared_ptr<std::ofstream> csvStream)
  {
    m_csvFileStream = csvStream;
    if (m_csvFileStream && m_csvFileStream->is_open())
    {
      // SRS Section 7 CSV Header:
      *m_csvFileStream << "time_s,node_id,role,x_m,y_m,z_m,speed_mps,selected_relay,available\n";
      m_csvFileStream->flush();
    }
  }

  void Start()
  {
    EvaluateRelays();
    m_evalEvent = Simulator::Schedule(m_decisionInterval, &AerialRelayManager::PeriodicEvaluate, this);
  }

  void Stop()
  {
    if (m_evalEvent.IsPending())
    {
      m_evalEvent.Cancel();
    }
  }

  void EvaluateRelays()
  {
    if (!m_sourceNode || !m_destNode)
    {
      return;
    }

    uint32_t srcId = m_sourceNode->GetId();
    uint32_t dstId = m_destNode->GetId();

    std::vector<RelayState> candidates;

    // Check candidate eligibility and fire RelayCandidateChange traces
    for (const auto& relayNode : m_relays)
    {
      uint32_t rId = relayNode->GetId();
      bool currentlyEligible = m_linkModel->IsEligible(srcId, rId, dstId);

      // Check for eligibility transition trace
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

      // Check position update trace
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

    // Check if the current relay became unavailable
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

    // Run RelaySelectionPolicy
    RelayRequest request{srcId, dstId};
    RelayDecision decision = m_policy->Select(candidates, request);
    int32_t preferredRelayId = decision.hasRelay ? decision.selectedRelayId : -1;

    Time now = Simulator::Now();

    // Dwell Time: do not switch away from an eligible relay if dwell time hasn't passed
    int32_t finalRelayId = preferredRelayId;
    if (currentStillEligible && preferredRelayId != m_selectedRelayId)
    {
      if (now - m_lastSelectionTime < m_minRelayDwellTime)
      {
        finalRelayId = m_selectedRelayId; // Stay on current relay
      }
    }

    // Perform Handover if changed
    if (finalRelayId != m_selectedRelayId)
    {
      int32_t oldRelay = m_selectedRelayId;
      m_selectedRelayId = finalRelayId;
      m_lastSelectionTime = now;

      // Update application states on relay nodes
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

      std::cout << ">>> [Time " << std::fixed << std::setprecision(2) << now.GetSeconds()
                << "s] RELAY HANDOVER: Old Relay = " << oldRelay
                << " -> New Relay = " << m_selectedRelayId
                << " (Metric: " << decision.metricValue << ") <<<" << std::endl;

      m_relaySelectionChangeTrace(srcId, oldRelay, m_selectedRelayId);
    }

    // Write CSV Row
    LogCsvRecord();
  }

  void LogCsvRecord()
  {
    if (!m_csvFileStream || !m_csvFileStream->is_open())
    {
      return;
    }

    double nowSec = Simulator::Now().GetSeconds();

    // Log all relays
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

private:
  void PeriodicEvaluate()
  {
    EvaluateRelays();
    m_evalEvent = Simulator::Schedule(m_decisionInterval, &AerialRelayManager::PeriodicEvaluate, this);
  }

  Ptr<Node> m_sourceNode;
  Ptr<Node> m_destNode;
  std::vector<Ptr<Node>> m_relays;
  std::vector<Ptr<Node>> m_endpoints;
  std::map<uint32_t, bool> m_candidateEligibility;

  Ptr<AerialLinkModel> m_linkModel;
  Ptr<RelaySelectionPolicy> m_policy;
  int32_t m_selectedRelayId;

  Time m_decisionInterval;
  Time m_minRelayDwellTime;
  Time m_lastSelectionTime;
  EventId m_evalEvent;

  std::shared_ptr<std::ofstream> m_csvFileStream;

  TracedCallback<uint32_t, Vector> m_aerialPositionUpdateTrace;
  TracedCallback<uint32_t, uint32_t, bool> m_relayCandidateChangeTrace;
  TracedCallback<uint32_t, int32_t, int32_t> m_relaySelectionChangeTrace;
  TracedCallback<uint32_t, uint32_t> m_relayUnavailableTrace;
};

// ===========================================================================
