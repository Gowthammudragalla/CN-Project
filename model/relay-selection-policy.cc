// Extracted from m03-aerial-relay.cc.

// 3. RelaySelectionPolicy Interface & Implementations (M03-FR-04, 05, 08)
// ===========================================================================

struct RelayState
{
  uint32_t relayId;
  Ptr<Node> node;
  Vector position;
  double distanceToSource;
  double distanceToDest;
  double totalDistance;
  Time estimatedDelay;
  double residualEnergy;
};

struct RelayRequest
{
  uint32_t sourceId;
  uint32_t destinationId;
};

struct RelayDecision
{
  int32_t selectedRelayId; // -1 if no eligible candidate
  bool hasRelay;
  double metricValue;      // distance in meters or delay in ms
};

class RelaySelectionPolicy : public Object
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::RelaySelectionPolicy")
      .SetParent<Object>()
      .SetGroupName("FutureNetAerial");
    return tid;
  }

  virtual RelayDecision Select(const std::vector<RelayState>& candidates,
                               const RelayRequest& request) const = 0;
};

// M03-FR-05: NearestRelayPolicy selects nearest eligible aerial relay
class NearestRelayPolicy : public RelaySelectionPolicy
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::NearestRelayPolicy")
      .SetParent<RelaySelectionPolicy>()
      .SetGroupName("FutureNetAerial")
      .AddConstructor<NearestRelayPolicy>();
    return tid;
  }

  RelayDecision Select(const std::vector<RelayState>& candidates,
                       const RelayRequest& request) const override
  {
    RelayDecision decision;
    decision.selectedRelayId = -1;
    decision.hasRelay = false;
    decision.metricValue = 1e12; // infinity

    for (const auto& cand : candidates)
    {
      if (cand.totalDistance < decision.metricValue)
      {
        decision.metricValue = cand.totalDistance;
        decision.selectedRelayId = cand.relayId;
        decision.hasRelay = true;
      }
    }
    return decision;
  }
};

// M03-FR-08: LowestDelayRelayPolicy selects eligible relay with lowest delay
class LowestDelayRelayPolicy : public RelaySelectionPolicy
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::LowestDelayRelayPolicy")
      .SetParent<RelaySelectionPolicy>()
      .SetGroupName("FutureNetAerial")
      .AddConstructor<LowestDelayRelayPolicy>();
    return tid;
  }

  RelayDecision Select(const std::vector<RelayState>& candidates,
                       const RelayRequest& request) const override
  {
    RelayDecision decision;
    decision.selectedRelayId = -1;
    decision.hasRelay = false;
    decision.metricValue = 1e12;

    for (const auto& cand : candidates)
    {
      double delayMs = cand.estimatedDelay.GetMilliSeconds();
      if (delayMs < decision.metricValue)
      {
        decision.metricValue = delayMs;
        decision.selectedRelayId = cand.relayId;
        decision.hasRelay = true;
      }
    }
    return decision;
  }
};

// ===========================================================================
