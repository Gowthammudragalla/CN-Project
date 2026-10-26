// Extracted from m03-aerial-relay.cc.

// 1. NODE ROLES & AerialNodeModel (M03-FR-01, M03-FR-09)
// ===========================================================================

enum class AerialNodeRole
{
  GROUND = 0,
  UAV,
  HAPS
};

std::string RoleToString(AerialNodeRole role)
{
  switch (role)
  {
    case AerialNodeRole::GROUND: return "GROUND";
    case AerialNodeRole::UAV:    return "UAV";
    case AerialNodeRole::HAPS:   return "HAPS";
    default:                     return "UNKNOWN";
  }
}

class AerialNodeModel : public Object
{
public:
  static TypeId GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::AerialNodeModel")
      .SetParent<Object>()
      .SetGroupName("FutureNetAerial")
      .AddConstructor<AerialNodeModel>();
    return tid;
  }

  AerialNodeModel()
    : m_role(AerialNodeRole::UAV),
      m_residualEnergyFraction(1.0)
  {
  }

  void SetRole(AerialNodeRole role)
  {
    m_role = role;
  }

  AerialNodeRole GetRole() const
  {
    return m_role;
  }

  // Residual energy fraction [0.0 = 0%, 1.0 = 100%]
  void SetResidualEnergyFraction(double fraction)
  {
    m_residualEnergyFraction = std::max(0.0, std::min(1.0, fraction));
  }

  double GetResidualEnergyFraction() const
  {
    return m_residualEnergyFraction;
  }

private:
  AerialNodeRole m_role;
  double m_residualEnergyFraction;
};

// ===========================================================================
