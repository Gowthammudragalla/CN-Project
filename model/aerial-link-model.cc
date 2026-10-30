// Extracted from m03-aerial-relay.cc.

// 2. AerialLinkModel (M03-FR-03, Section 5 & 6)
// ===========================================================================

struct LinkState
{
  bool isConnected;
  double distanceMeters;
  Time propagationDelay;
  DataRate nominalRate;
};

class AerialLinkModel : public Object
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::CNProjectAerialLinkModel")
      .SetParent<Object>()
      .SetGroupName("FutureNetAerial")
      .AddConstructor<AerialLinkModel>();
    return tid;
  }

  AerialLinkModel()
    : m_maxGroundAerialRange(2000.0),    // Default: 2000 m
      m_maxAerialAerialRange(5000.0),    // Default: 5000 m
      m_nominalLinkRate("50Mbps"),       // Default: 50 Mbit/s
      m_baseProcessingDelay(MilliSeconds(1)) // Default: 1 ms
  {
  }

  void SetMaxGroundAerialRange(double rangeMeters) { m_maxGroundAerialRange = rangeMeters; }
  double GetMaxGroundAerialRange() const { return m_maxGroundAerialRange; }

  void SetMaxAerialAerialRange(double rangeMeters) { m_maxAerialAerialRange = rangeMeters; }
  double GetMaxAerialAerialRange() const { return m_maxAerialAerialRange; }

  void SetNominalLinkRate(DataRate rate) { m_nominalLinkRate = rate; }
  DataRate GetNominalLinkRate() const { return m_nominalLinkRate; }

  void SetBaseProcessingDelay(Time delay) { m_baseProcessingDelay = delay; }
  Time GetBaseProcessingDelay() const { return m_baseProcessingDelay; }

  double GetDistance(Ptr<Node> a, Ptr<Node> b) const
  {
    Ptr<MobilityModel> mobA = a->GetObject<MobilityModel>();
    Ptr<MobilityModel> mobB = b->GetObject<MobilityModel>();
    NS_ASSERT_MSG(mobA && mobB, "Nodes must have MobilityModel installed");
    return mobA->GetDistanceFrom(mobB);
  }

  double GetDistance(uint32_t aId, uint32_t bId) const
  {
    Ptr<Node> nodeA = NodeList::GetNode(aId);
    Ptr<Node> nodeB = NodeList::GetNode(bId);
    return GetDistance(nodeA, nodeB);
  }

  // Check connectivity between two nodes
  bool IsLinkValid(Ptr<Node> nodeA, Ptr<Node> nodeB) const
  {
    Ptr<AerialNodeModel> modelA = nodeA->GetObject<AerialNodeModel>();
    Ptr<AerialNodeModel> modelB = nodeB->GetObject<AerialNodeModel>();

    AerialNodeRole roleA = modelA ? modelA->GetRole() : AerialNodeRole::GROUND;
    AerialNodeRole roleB = modelB ? modelB->GetRole() : AerialNodeRole::GROUND;

    double dist = GetDistance(nodeA, nodeB);

    // Ground <-> Aerial (UAV or HAPS)
    if ((roleA == AerialNodeRole::GROUND && roleB != AerialNodeRole::GROUND) ||
        (roleB == AerialNodeRole::GROUND && roleA != AerialNodeRole::GROUND))
    {
      return dist <= m_maxGroundAerialRange;
    }

    // Aerial <-> Aerial
    if (roleA != AerialNodeRole::GROUND && roleB != AerialNodeRole::GROUND)
    {
      return dist <= m_maxAerialAerialRange;
    }

    // Ground to Ground requires relay
    return false;
  }

  // SRS Section 6: IsEligible(uint32_t sourceId, uint32_t relayId, uint32_t destinationId)
  bool IsEligible(uint32_t sourceId, uint32_t relayId, uint32_t destinationId) const
  {
    Ptr<Node> src = NodeList::GetNode(sourceId);
    Ptr<Node> rly = NodeList::GetNode(relayId);
    Ptr<Node> dst = NodeList::GetNode(destinationId);
    return IsLinkValid(src, rly) && IsLinkValid(rly, dst);
  }

  LinkState GetLinkState(Ptr<Node> a, Ptr<Node> b) const
  {
    LinkState st;
    st.distanceMeters = GetDistance(a, b);
    st.isConnected = IsLinkValid(a, b);
    st.propagationDelay = Seconds(st.distanceMeters / SPEED_OF_LIGHT);
    st.nominalRate = m_nominalLinkRate;
    return st;
  }

  // Total estimated delay: (d1 / c) + processingDelay + (d2 / c)
  Time GetEstimatedPathDelay(Ptr<Node> src, Ptr<Node> relay, Ptr<Node> dst) const
  {
    double d1 = GetDistance(src, relay);
    double d2 = GetDistance(relay, dst);
    double propSeconds = (d1 + d2) / SPEED_OF_LIGHT;
    return Seconds(propSeconds) + m_baseProcessingDelay;
  }

private:
  double m_maxGroundAerialRange;
  double m_maxAerialAerialRange;
  DataRate m_nominalLinkRate;
  Time m_baseProcessingDelay;
};

// ===========================================================================
