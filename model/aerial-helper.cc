// Extracted from m03-aerial-relay.cc.

// 6. AerialHelper (SRS Section 6)
// ===========================================================================

class AerialHelper
{
public:
  AerialHelper()
    : m_uavAltitude(100.0),
      m_hapsAltitude(20000.0)
  {
    m_linkModel = CreateObject<AerialLinkModel>();
    m_manager = CreateObject<AerialRelayManager>();
    m_manager->SetLinkModel(m_linkModel);
  }

  void SetUavAltitude(double alt) { m_uavAltitude = alt; }
  void SetHapsAltitude(double alt) { m_hapsAltitude = alt; }

  void InstallGround(NodeContainer groundNodes)
  {
    for (uint32_t i = 0; i < groundNodes.GetN(); ++i)
    {
      Ptr<AerialNodeModel> model = CreateObject<AerialNodeModel>();
      model->SetRole(AerialNodeRole::GROUND);
      groundNodes.Get(i)->AggregateObject(model);
    }
  }

  void InstallUav(NodeContainer uavNodes)
  {
    for (uint32_t i = 0; i < uavNodes.GetN(); ++i)
    {
      Ptr<AerialNodeModel> model = CreateObject<AerialNodeModel>();
      model->SetRole(AerialNodeRole::UAV);
      uavNodes.Get(i)->AggregateObject(model);
      m_manager->RegisterRelay(uavNodes.Get(i));
    }
  }

  void InstallHaps(NodeContainer hapsNodes)
  {
    for (uint32_t i = 0; i < hapsNodes.GetN(); ++i)
    {
      Ptr<AerialNodeModel> model = CreateObject<AerialNodeModel>();
      model->SetRole(AerialNodeRole::HAPS);
      hapsNodes.Get(i)->AggregateObject(model);
      m_manager->RegisterRelay(hapsNodes.Get(i));
    }
  }

  void SetRelayPolicy(Ptr<RelaySelectionPolicy> policy)
  {
    m_manager->SetPolicy(policy);
  }

  Ptr<AerialRelayManager> GetManager() const { return m_manager; }
  Ptr<AerialLinkModel> GetLinkModel() const { return m_linkModel; }

private:
  double m_uavAltitude;
  double m_hapsAltitude;
  Ptr<AerialLinkModel> m_linkModel;
  Ptr<AerialRelayManager> m_manager;
};

// ===========================================================================
