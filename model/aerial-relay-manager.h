#ifndef AERIAL_RELAY_MANAGER_H
#define AERIAL_RELAY_MANAGER_H

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

#include <vector>
#include <map>
#include <memory>
#include <fstream>
#include <string>

using namespace ns3;

class AerialLinkModel;
class RelaySelectionPolicy;

/**
 * \ingroup futurenet-aerial
 * \brief Central manager coordinating aerial relay selection, handovers, and tracing
 *
 * AerialRelayManager registers aerial relays and endpoints, evaluates connectivity using
 * AerialLinkModel, executes a replaceable RelaySelectionPolicy (NearestRelayPolicy or
 * LowestDelayRelayPolicy), detects relay unavailability, enforces dwell times, and updates
 * the active state of AerialRelayApplication on each relay node.
 */
class AerialRelayManager : public Object
{
public:
  static TypeId GetTypeId(void);

  // Trace callback signatures matching SRS Section 7
  typedef void (*AerialPositionUpdateTracedCallback)(uint32_t nodeId, Vector position);
  typedef void (*RelayCandidateChangeTracedCallback)(uint32_t endpointId, uint32_t relayId, bool eligible);
  typedef void (*RelaySelectionChangeTracedCallback)(uint32_t endpointId, int32_t oldRelayId, int32_t newRelayId);
  typedef void (*RelayUnavailableTracedCallback)(uint32_t endpointId, uint32_t relayId);

  AerialRelayManager();
  ~AerialRelayManager() override;

  /**
   * \brief Set the link model used for range eligibility and path delay calculation
   * \param linkModel Pointer to AerialLinkModel
   */
  void SetLinkModel(Ptr<AerialLinkModel> linkModel);

  /**
   * \brief Get the current link model
   * \return Pointer to AerialLinkModel
   */
  Ptr<AerialLinkModel> GetLinkModel() const;

  /**
   * \brief Set the relay selection policy (NearestRelayPolicy or LowestDelayRelayPolicy)
   * \param policy Pointer to RelaySelectionPolicy implementation
   */
  void SetPolicy(Ptr<RelaySelectionPolicy> policy);

  /**
   * \brief Get the active relay selection policy
   * \return Pointer to RelaySelectionPolicy
   */
  Ptr<RelaySelectionPolicy> GetPolicy() const;

  /**
   * \brief Set the interval between periodic relay evaluations
   * \param interval Periodic evaluation interval (e.g. 1 second)
   */
  void SetDecisionInterval(Time interval);

  /**
   * \brief Get the periodic decision interval
   * \return Decision interval Time
   */
  Time GetDecisionInterval() const;

  /**
   * \brief Set the minimum dwell time before switching away from an eligible relay
   * \param dwellTime Minimum dwell time (e.g. 2 seconds)
   */
  void SetMinRelayDwellTime(Time dwellTime);

  /**
   * \brief Get the minimum dwell time
   * \return Minimum dwell time Time
   */
  Time GetMinRelayDwellTime() const;

  /**
   * \brief Register an aerial node as a candidate relay
   * \param relayNode Node pointer of the UAV/HAPS relay
   */
  void RegisterRelay(Ptr<Node> relayNode);

  /**
   * \brief Register an endpoint node
   * \param endpointNode Node pointer of endpoint
   */
  void RegisterEndpoint(Ptr<Node> endpointNode);

  /**
   * \brief Configure the communication source and destination endpoints
   * \param source Source ground node
   * \param dest Destination ground node
   */
  void SetEndpoints(Ptr<Node> source, Ptr<Node> dest);

  /**
   * \brief Get the currently selected relay ID for an endpoint (SRS Interface)
   * \param endpointId ID of the endpoint node
   * \return Selected relay node ID or -1 if none
   */
  int32_t GetSelectedRelay(uint32_t endpointId) const;

  /**
   * \brief Get the currently selected relay ID
   * \return Selected relay node ID or -1 if none
   */
  int32_t GetSelectedRelayId() const;

  /**
   * \brief Set the output file stream for CSV traces
   * \param csvStream Shared pointer to ofstream
   */
  void SetCsvStream(std::shared_ptr<std::ofstream> csvStream);

  /**
   * \brief Start periodic relay evaluation
   */
  void Start();

  /**
   * \brief Stop periodic relay evaluation and cancel scheduled events
   */
  void Stop();

  /**
   * \brief Perform an immediate evaluation of all relays
   */
  void EvaluateRelays();

protected:
  void DoDispose() override;

private:
  void PeriodicEvaluate();
  void LogCsvRecord();
  void UpdateRelayAppStates();

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

#endif // AERIAL_RELAY_MANAGER_H
