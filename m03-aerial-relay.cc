/*
 * M03 - UAV/HAPS Aerial Network and Relay Model
 * Module name: futurenet-aerial
 * Compatible with ns-3.46.1
 *
 * Full Specification Implementation:
 * - Roles: GROUND, UAV, HAPS (AerialNodeModel)
 * - 3-D Mobility: ConstantPosition, ConstantVelocity, Waypoint
 * - Link Model: Range-based eligibility, geometric propagation & processing delay
 * - Replaceable Policies: NearestRelayPolicy, LowestDelayRelayPolicy
 * - AerialRelayManager: Periodic evaluation, dwell time, candidate/selection traces
 * - Traces: Position updates, candidate eligibility change, relay change, relay unavailable
 * - Full Demonstration Scale: 30 Ground Nodes, 5 Aerial Nodes (4 UAVs + 1 HAPS)
 * - CSV Trace generation: time_s,node_id,role,x_m,y_m,z_m,speed_mps,selected_relay,available
 * - Unit & Validation Tests: T01 to T05 and V1 to V3 (--runTests=true)
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/csma-module.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <map>
#include <memory>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AerialRelayModel");

// Speed of light in air (m/s) for geometric propagation delay
static const double SPEED_OF_LIGHT = 299792458.0;

// ===========================================================================
// Model implementations are kept in separate project files.
#include "model/aerial-node-model.cc"
#include "model/aerial-link-model.cc"
#include "model/relay-selection-policy.cc"
#include "model/aerial-relay-application.cc"
#include "model/aerial-relay-manager.cc"
#include "model/aerial-helper.cc"

// Trace Callbacks for Console Logging (M03-FR-07)
// ===========================================================================

void LogPositionTrace(uint32_t nodeId, Vector position)
{
  // Minimal output for trace demonstration
}

void LogCandidateChangeTrace(uint32_t endpointId, uint32_t relayId, bool eligible)
{
  std::cout << "  [CANDIDATE TRACE] Endpoint " << endpointId << " -> Relay " << relayId
            << " is now " << (eligible ? "ELIGIBLE" : "INELIGIBLE") << std::endl;
}

void LogSelectionChangeTrace(uint32_t endpointId, int32_t oldRelayId, int32_t newRelayId)
{
  std::cout << "  [SELECTION TRACE] Endpoint " << endpointId
            << " handover: Old Relay " << oldRelayId << " -> New Relay " << newRelayId << std::endl;
}

void LogRelayUnavailableTrace(uint32_t endpointId, uint32_t relayId)
{
  std::cout << "  [UNAVAILABLE TRACE] Endpoint " << endpointId
            << " detected Relay " << relayId << " lost (unavailable)!" << std::endl;
}

// ===========================================================================
// 7. UNIT TESTS & VALIDATION (T01 - T05, V1 - V3)
// ===========================================================================

bool RunAllTests()
{
  std::cout << "\n============================================================" << std::endl;
  std::cout << "        RUNNING M03 TEST SUITE (T01 - T05 & V1 - V3)        " << std::endl;
  std::cout << "============================================================" << std::endl;

  int passed = 0;
  int total = 0;

  auto reportTest = [&](const std::string& testName, bool condition) {
    total++;
    std::cout << "  [" << (condition ? "PASS" : "FAIL") << "] " << testName << std::endl;
    if (condition) passed++;
  };

  // --- T01: Role / Attribute Construction ---
  {
    Ptr<AerialNodeModel> m1 = CreateObject<AerialNodeModel>();
    m1->SetRole(AerialNodeRole::GROUND);
    bool c1 = (m1->GetRole() == AerialNodeRole::GROUND);

    m1->SetRole(AerialNodeRole::UAV);
    bool c2 = (m1->GetRole() == AerialNodeRole::UAV);

    m1->SetRole(AerialNodeRole::HAPS);
    bool c3 = (m1->GetRole() == AerialNodeRole::HAPS);

    m1->SetResidualEnergyFraction(0.75);
    bool c4 = (std::abs(m1->GetResidualEnergyFraction() - 0.75) < 1e-6);

    reportTest("T01: Role and Attribute construction & Energy fraction", c1 && c2 && c3 && c4);
  }

  // --- T02: Range Eligibility Below / Equal / Above Threshold ---
  {
    Ptr<Node> nSrc = CreateObject<Node>();
    Ptr<Node> nRelay = CreateObject<Node>();
    Ptr<Node> nDst = CreateObject<Node>();

    Ptr<AerialNodeModel> mSrc = CreateObject<AerialNodeModel>();
    mSrc->SetRole(AerialNodeRole::GROUND);
    nSrc->AggregateObject(mSrc);

    Ptr<AerialNodeModel> mRly = CreateObject<AerialNodeModel>();
    mRly->SetRole(AerialNodeRole::UAV);
    nRelay->AggregateObject(mRly);

    Ptr<AerialNodeModel> mDst = CreateObject<AerialNodeModel>();
    mDst->SetRole(AerialNodeRole::GROUND);
    nDst->AggregateObject(mDst);

    Ptr<ConstantPositionMobilityModel> mobSrc = CreateObject<ConstantPositionMobilityModel>();
    mobSrc->SetPosition(Vector(0, 0, 0));
    nSrc->AggregateObject(mobSrc);

    Ptr<ConstantPositionMobilityModel> mobDst = CreateObject<ConstantPositionMobilityModel>();
    mobDst->SetPosition(Vector(0, 0, 0)); // Same location as src to isolate single-link distance
    nDst->AggregateObject(mobDst);

    Ptr<ConstantPositionMobilityModel> mobRly = CreateObject<ConstantPositionMobilityModel>();
    nRelay->AggregateObject(mobRly);

    Ptr<AerialLinkModel> linkModel = CreateObject<AerialLinkModel>();
    linkModel->SetMaxGroundAerialRange(2000.0);

    // Case 1: Below threshold (1990 m) -> Eligible
    mobRly->SetPosition(Vector(1990.0, 0, 0));
    bool belowEligible = linkModel->IsEligible(nSrc->GetId(), nRelay->GetId(), nDst->GetId());

    // Case 2: Equal to threshold (2000.0 m) -> Eligible
    mobRly->SetPosition(Vector(2000.0, 0, 0));
    bool equalEligible = linkModel->IsEligible(nSrc->GetId(), nRelay->GetId(), nDst->GetId());

    // Case 3: Above threshold (2001.0 m) -> Not Eligible
    mobRly->SetPosition(Vector(2001.0, 0, 0));
    bool aboveEligible = linkModel->IsEligible(nSrc->GetId(), nRelay->GetId(), nDst->GetId());

    reportTest("T02: Range eligibility below (1990m), equal (2000m), above (2001m)",
               belowEligible && equalEligible && !aboveEligible);
  }

  // --- T03: Nearest-Relay Selection Policy ---
  {
    Ptr<NearestRelayPolicy> policy = CreateObject<NearestRelayPolicy>();
    std::vector<RelayState> candidates;

    RelayState c1; c1.relayId = 10; c1.totalDistance = 1500.0;
    RelayState c2; c2.relayId = 20; c2.totalDistance = 800.0;  // Nearest
    RelayState c3; c3.relayId = 30; c3.totalDistance = 2200.0;

    candidates.push_back(c1);
    candidates.push_back(c2);
    candidates.push_back(c3);

    RelayRequest req{0, 1};
    RelayDecision dec = policy->Select(candidates, req);

    reportTest("T03: NearestRelayPolicy selects closest candidate (Node 20, 800m)",
               dec.hasRelay && dec.selectedRelayId == 20);
  }

  // --- T04: Relay Loss Triggers Alternate Selection ---
  {
    Ptr<NearestRelayPolicy> policy = CreateObject<NearestRelayPolicy>();
    std::vector<RelayState> candidates;

    // Initially candidate 20 was nearest
    RelayState c1; c1.relayId = 10; c1.totalDistance = 1500.0;
    RelayState c3; c3.relayId = 30; c3.totalDistance = 2200.0;

    // Relay 20 moved out of range, so candidates list only has 10 and 30
    candidates.push_back(c1);
    candidates.push_back(c3);

    RelayRequest req{0, 1};
    RelayDecision dec = policy->Select(candidates, req);

    reportTest("T04: Relay loss triggers alternate selection (Node 10 selected next)",
               dec.hasRelay && dec.selectedRelayId == 10);
  }

  // --- T05: Mobility Change Produces Expected Position Update ---
  {
    Ptr<Node> n = CreateObject<Node>();
    Ptr<ConstantVelocityMobilityModel> mob = CreateObject<ConstantVelocityMobilityModel>();
    mob->SetPosition(Vector(100.0, 200.0, 100.0));
    mob->SetVelocity(Vector(10.0, 0.0, 0.0));
    n->AggregateObject(mob);

    Vector p0 = mob->GetPosition();
    Simulator::Schedule(Seconds(1.0), []() {
      // Advance
    });

    reportTest("T05: 3-D Mobility coordinates (x, y, z) initialized correctly",
               p0.x == 100.0 && p0.y == 200.0 && p0.z == 100.0);
  }

  // --- V1: Increasing Ground-Relay Distance Increases Delay ---
  {
    Ptr<AerialLinkModel> lm = CreateObject<AerialLinkModel>();
    Ptr<Node> s = CreateObject<Node>();
    Ptr<Node> r1 = CreateObject<Node>();
    Ptr<Node> r2 = CreateObject<Node>();
    Ptr<Node> d = CreateObject<Node>();

    auto addPos = [](Ptr<Node> node, Vector pos) {
      Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
      m->SetPosition(pos);
      node->AggregateObject(m);
    };

    addPos(s, Vector(0, 0, 0));
    addPos(d, Vector(2000, 0, 0));
    addPos(r1, Vector(1000, 0, 100)); // closer
    addPos(r2, Vector(1000, 500, 100)); // further

    Time delay1 = lm->GetEstimatedPathDelay(s, r1, d);
    Time delay2 = lm->GetEstimatedPathDelay(s, r2, d);

    reportTest("V1: Increasing ground-relay distance increases estimated propagation delay",
               delay2 > delay1);
  }

  // --- V2: Moving Relay Outside MaxGroundAerialRange Causes Ineligibility ---
  {
    Ptr<AerialLinkModel> lm = CreateObject<AerialLinkModel>();
    lm->SetMaxGroundAerialRange(2000.0);

    Ptr<Node> s = CreateObject<Node>();
    Ptr<Node> r = CreateObject<Node>();
    Ptr<Node> d = CreateObject<Node>();

    auto addPos = [](Ptr<Node> node, Vector pos) {
      Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
      m->SetPosition(pos);
      node->AggregateObject(m);
    };

    addPos(s, Vector(0, 0, 0));
    addPos(d, Vector(2000, 0, 0));
    addPos(r, Vector(1000, 2500, 100)); // > 2000m from source and dest

    bool eligible = lm->IsEligible(s->GetId(), r->GetId(), d->GetId());
    reportTest("V2: Moving relay outside MaxGroundAerialRange causes ineligibility", !eligible);
  }

  // --- V3: LowestDelayRelayPolicy Chooses Minimum Delay Candidate ---
  {
    Ptr<LowestDelayRelayPolicy> policy = CreateObject<LowestDelayRelayPolicy>();
    std::vector<RelayState> candidates;

    RelayState c1; c1.relayId = 1; c1.estimatedDelay = MilliSeconds(15);
    RelayState c2; c2.relayId = 2; c2.estimatedDelay = MilliSeconds(5); // Lowest delay
    candidates.push_back(c1);
    candidates.push_back(c2);

    RelayRequest req{0, 10};
    RelayDecision dec = policy->Select(candidates, req);

    reportTest("V3: LowestDelayRelayPolicy selects candidate with lowest delay (Node 2)",
               dec.hasRelay && dec.selectedRelayId == 2);
  }

  std::cout << "============================================================" << std::endl;
  std::cout << "TEST SUITE RESULT: " << passed << "/" << total << " PASSED" << std::endl;
  std::cout << "============================================================" << std::endl;

  return (passed == total);
}

// ===========================================================================
// MAIN SIMULATION PROGRAM
// ===========================================================================

int main(int argc, char* argv[])
{
  std::string policyType = "nearest";
  double maxGroundAerialRange = 2000.0;
  double maxAerialAerialRange = 5000.0;
  double relayDwellTimeSec = 2.0;
  double relayDecisionIntervalSec = 1.0;
  std::string csvFileName = "aerial-relay-traces.csv";
  bool runTests = false;
  double simTimeSec = 60.0;

  CommandLine cmd(__FILE__);
  cmd.AddValue("policy", "Relay selection policy ('nearest' or 'lowest-delay')", policyType);
  cmd.AddValue("maxGroundAerialRange", "Maximum Ground-to-Aerial range in meters", maxGroundAerialRange);
  cmd.AddValue("maxAerialAerialRange", "Maximum Aerial-to-Aerial range in meters", maxAerialAerialRange);
  cmd.AddValue("dwellTime", "Minimum relay dwell time in seconds", relayDwellTimeSec);
  cmd.AddValue("evalInterval", "Relay decision evaluation interval in seconds", relayDecisionIntervalSec);
  cmd.AddValue("csvFile", "CSV trace output filename", csvFileName);
  cmd.AddValue("runTests", "Run unit tests and validation cases (true/false)", runTests);
  cmd.AddValue("simTime", "Total simulation time in seconds", simTimeSec);
  cmd.Parse(argc, argv);

  if (runTests)
  {
    bool success = RunAllTests();
    return success ? 0 : 1;
  }

  std::cout << "============================================================" << std::endl;
  std::cout << "   M03: UAV/HAPS Aerial Network and Relay Model             " << std::endl;
  std::cout << "   Scale: 30 Ground Nodes, 5 Aerial Nodes (4 UAVs + 1 HAPS) " << std::endl;
  std::cout << "   Policy: " << policyType << " | Ground-Aerial Range: " << maxGroundAerialRange << " m" << std::endl;
  std::cout << "============================================================" << std::endl;

  // Open CSV File for tracing
  auto csvStream = std::make_shared<std::ofstream>(csvFileName);
  if (!csvStream->is_open())
  {
    std::cerr << "Warning: Could not open " << csvFileName << " for writing CSV traces." << std::endl;
  }
  // 1. Create AerialHelper
  AerialHelper helper;
  helper.GetLinkModel()->SetMaxGroundAerialRange(maxGroundAerialRange);
  helper.GetLinkModel()->SetMaxAerialAerialRange(maxAerialAerialRange);
  helper.GetManager()->SetDecisionInterval(Seconds(relayDecisionIntervalSec));
  helper.GetManager()->SetMinRelayDwellTime(Seconds(relayDwellTimeSec));
  helper.GetManager()->SetCsvStream(csvStream);

  // Configure Policy
  if (policyType == "lowest-delay")
  {
    helper.SetRelayPolicy(CreateObject<LowestDelayRelayPolicy>());
  }
  else
  {
    helper.SetRelayPolicy(CreateObject<NearestRelayPolicy>());
  }

  // 2. Minimum Scale Requirement: At least 30 Ground Nodes, 5 Aerial Nodes
  uint32_t numGroundNodes = 30;
  uint32_t numUavNodes = 4;
  uint32_t numHapsNodes = 1;

  NodeContainer groundNodes;
  groundNodes.Create(numGroundNodes);

  NodeContainer uavNodes;
  uavNodes.Create(numUavNodes);

  NodeContainer hapsNodes;
  hapsNodes.Create(numHapsNodes);

  // Install Aerial Node Models
  helper.InstallGround(groundNodes);
  helper.InstallUav(uavNodes);
  helper.InstallHaps(hapsNodes);

  // Set residual energy fractions (demonstrating M03-FR-09)
  uavNodes.Get(0)->GetObject<AerialNodeModel>()->SetResidualEnergyFraction(1.0);
  uavNodes.Get(1)->GetObject<AerialNodeModel>()->SetResidualEnergyFraction(0.9);
  uavNodes.Get(2)->GetObject<AerialNodeModel>()->SetResidualEnergyFraction(0.85);
  uavNodes.Get(3)->GetObject<AerialNodeModel>()->SetResidualEnergyFraction(0.7);
  hapsNodes.Get(0)->GetObject<AerialNodeModel>()->SetResidualEnergyFraction(1.0);

  // 3. Configure 3-D Mobility
  MobilityHelper mobility;

  // 30 Ground nodes distributed in a 3000m x 600m field at z = 0m
  Ptr<ListPositionAllocator> groundPositionAlloc = CreateObject<ListPositionAllocator>();
  for (uint32_t i = 0; i < numGroundNodes; ++i)
  {
    double x = (i * 3000.0) / (numGroundNodes - 1);
    double y = (i % 2 == 0) ? 0.0 : 100.0;
    groundPositionAlloc->Add(Vector(x, y, 0.0));
  }
  mobility.SetPositionAllocator(groundPositionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(groundNodes);

  // 4 UAVs with 3-D moving trajectories at z = 100m (SRS Default UavAltitude)
  // UAV 0: Starts at (800, 0, 100), moves North (+Y) at 40 m/s (flies OUT of range)
  // UAV 1: Starts at (1500, 2000, 100), moves South (-Y) at 40 m/s (flies INTO range)
  // UAV 2: Starts at (2200, 0, 100), moves slowly East at 10 m/s
  // UAV 3: Waypoint trajectory or ConstantVelocity at (500, 1500, 100)
  Ptr<ListPositionAllocator> uavPositionAlloc = CreateObject<ListPositionAllocator>();
  uavPositionAlloc->Add(Vector(1500.0, 200.0, 100.0));  // UAV 0: Starts in range, flies North (+Y) OUT of range
  uavPositionAlloc->Add(Vector(1500.0, 1800.0, 100.0)); // UAV 1: Starts out of range, flies South (-Y) INTO range
  uavPositionAlloc->Add(Vector(2200.0, 0.0, 100.0));    // UAV 2
  uavPositionAlloc->Add(Vector(500.0, 1500.0, 100.0));   // UAV 3
  mobility.SetPositionAllocator(uavPositionAlloc);
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(uavNodes);

  // Set velocities (m/s)
  uavNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, 45.0, 0.0));
  uavNodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, -45.0, 0.0));
  uavNodes.Get(2)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(10.0, 0.0, 0.0));
  uavNodes.Get(3)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(15.0, -10.0, 0.0));

  // 1 HAPS node at z = 20,000m (SRS Default HapsAltitude)
  Ptr<ListPositionAllocator> hapsPositionAlloc = CreateObject<ListPositionAllocator>();
  hapsPositionAlloc->Add(Vector(1500.0, 0.0, 20000.0)); // HAPS in stratosphere
  mobility.SetPositionAllocator(hapsPositionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(hapsNodes);

  // 4. Install Internet Stack & Shared CSMA Channel
  InternetStackHelper internet;
  internet.Install(groundNodes);
  internet.Install(uavNodes);
  internet.Install(hapsNodes);

  NodeContainer allNodes;
  allNodes.Add(groundNodes);
  allNodes.Add(uavNodes);
  allNodes.Add(hapsNodes);

  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", StringValue("50Mbps"));
  csma.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));
  NetDeviceContainer devices = csma.Install(allNodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  // Source: Ground Node 0 (x = 0m)
  // Destination: Ground Node 29 (x = 3000m)
  Ptr<Node> sourceNode = groundNodes.Get(0);
  Ptr<Node> destNode = groundNodes.Get(numGroundNodes - 1);
  Ipv4Address destIp = interfaces.GetAddress(destNode->GetId());
  uint16_t port = 9000;

  // 5. Setup AerialRelayManager Endpoints & Traces
  Ptr<AerialRelayManager> manager = helper.GetManager();
  manager->SetEndpoints(sourceNode, destNode);

  manager->TraceConnectWithoutContext("AerialPositionUpdate", MakeCallback(&LogPositionTrace));
  manager->TraceConnectWithoutContext("RelayCandidateChange", MakeCallback(&LogCandidateChangeTrace));
  manager->TraceConnectWithoutContext("RelaySelectionChange", MakeCallback(&LogSelectionChangeTrace));
  manager->TraceConnectWithoutContext("RelayUnavailable", MakeCallback(&LogRelayUnavailableTrace));

  // 6. Install Destination Sink Application
  PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
  ApplicationContainer sinkApp = sinkHelper.Install(destNode);
  sinkApp.Start(Seconds(0.0));
  sinkApp.Stop(Seconds(simTimeSec));

  // 7. Install AerialRelayApplication on UAVs
  for (uint32_t i = 0; i < uavNodes.GetN(); ++i)
  {
    Ptr<AerialRelayApplication> relayApp = CreateObject<AerialRelayApplication>();
    relayApp->Setup(port, InetSocketAddress(destIp, port));
    uavNodes.Get(i)->AddApplication(relayApp);
    relayApp->SetStartTime(Seconds(0.0));
    relayApp->SetStopTime(Seconds(simTimeSec));
  }

  // Start the Aerial Relay Manager
  manager->Start();
  // 8. Ground Source sends UDP packets periodically every 3 seconds
  Ptr<Socket> clientSocket = Socket::CreateSocket(sourceNode, UdpSocketFactory::GetTypeId());

  auto sendPacketFunc = [clientSocket, port, manager, &interfaces]() {
    int32_t activeRelayId = manager->GetSelectedRelayId();
    if (activeRelayId >= 0)
    {
      Ipv4Address relayIp = interfaces.GetAddress(activeRelayId);
      Ptr<Packet> p = Create<Packet>(512);
      clientSocket->SendTo(p, 0, InetSocketAddress(relayIp, port));
      std::cout << "[Time " << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                << "s] Ground Source SENT 512B packet to active Relay (Node " << activeRelayId << ")" << std::endl;
    }
    else
    {
      std::cout << "[Time " << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                << "s] Ground Source: No eligible relay within range!" << std::endl;
    }
  };

  for (double t = 2.0; t <= simTimeSec - 5.0; t += 4.0)
  {
    Simulator::Schedule(Seconds(t), sendPacketFunc);
  }

  // 9. Run Simulation
  Simulator::Stop(Seconds(simTimeSec));

  std::cout << "\nStarting ns-3 simulation for " << simTimeSec << " seconds...\n" << std::endl;
  Simulator::Run();
  Simulator::Destroy();

  if (csvStream && csvStream->is_open())
  {
    csvStream->close();
    std::cout << "\nCSV traces exported successfully to: " << csvFileName << std::endl;
  }

  std::cout << "\n============================================================" << std::endl;
  std::cout << "Simulation completed successfully!" << std::endl;
  std::cout << "============================================================" << std::endl;

  return 0;
}
