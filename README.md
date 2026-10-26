# M03 - UAV/HAPS Aerial Network and Relay Model

**Module Name:** `futurenet-aerial`  
**Version:** 1.0  
**Framework:** ns-3.46.1  
**Course:** Computer Networks (CN)

---

## 1. Project Overview

This project implements a reusable UAV/HAPS aerial network and relay model using **ns-3.46.1** according to the SRS specification for **M03 - UAV/HAPS Aerial Network and Relay Model**. The simulation focuses on 3D mobility, connectivity, relay selection, and dynamic service availability.

```text
Ground Source (Node 0)
          |
          v
    UAV / HAPS Relay
          |
          v
Ground Destination (Node 29)
```

---

## 2. Objectives & Key Features

- **3-D Mobility**: Configure Ground, UAV, and HAPS nodes with 3D mobility (`ConstantPosition`, `ConstantVelocity`, and moving trajectories).
- **Logical Roles**: Distinguish `GROUND`, `UAV`, and `HAPS` node roles.
- **Connectivity Model**: Range-based link eligibility (`MaxGroundAerialRange = 2000 m`, `MaxAerialAerialRange = 5000 m`).
- **Replaceable Selection Policies**:
  - `NearestRelayPolicy`: Selects the eligible relay with minimum path distance.
  - `LowestDelayRelayPolicy`: Selects the eligible relay with minimum end-to-end propagation and processing delay.
- **Dynamic Relay Handover**: Detects when an active relay flies out of range and executes automated handover to an eligible alternative.
- **Traces & CSV Logging**: Records position updates, eligibility changes, selection events, and real-time state metrics.

---

## 3. Repository Structure

All project files are organized as follows:

| File Name | Description |
| :--- | :--- |
| `m03-aerial-relay.cc` | Main C++ simulation source code implementing all SRS classes and demonstration scale |
| `plot_aerial_relay.py` | Python visualization script to plot 2D/3D UAV trajectories and relay handover timeline |
| `aerial-relay-traces.csv` | Output CSV trace file containing real-time simulation metrics, coordinates, and relay status |
| `aerial_relay_analysis.png` | Generated analysis plot showing UAV coverage areas, trajectories, and dynamic handovers |
| `model/` | Implementations included by the main scratch driver |
| `README.md` | Comprehensive documentation, execution commands, and project details |

---

## 4. How to Build and Run

From the ns-3 root directory (`ns-3.46.1/`):

### 1. Build the Project
```bash
./ns3 build
```

### 2. Run the Automated SRS Test Suite (T01 - T05 & V1 - V3)
```bash
./ns3 run "scratch/m03-aerial-relay --runTests=true"
```

### 3. Run the Demonstration Simulation (30 Ground Nodes, 5 Aerial Nodes)
```bash
./ns3 run scratch/m03-aerial-relay
```

### 4. Run with Lowest-Delay Relay Policy
```bash
./ns3 run "scratch/m03-aerial-relay --policy=lowest-delay"
```

### 5. Run with Custom Parameters
```bash
./ns3 run "scratch/m03-aerial-relay --maxGroundAerialRange=2500 --dwellTime=3.0 --simTime=90"
```

### 6. Generate / Update Visual Analysis Graphs
```bash
python3 scratch/CN-Project-main/plot_aerial_relay.py scratch/CN-Project-main/aerial-relay-traces.csv scratch/CN-Project-main/aerial_relay_analysis.png
```

---

## 5. Simulation Scale & Demonstration

- **30 Ground Nodes** distributed across a $3000\text{ m} \times 100\text{ m}$ coordinate plane.
- **5 Aerial Nodes**: 4 moving UAVs at $z = 100\text{ m}$ altitude and 1 stratospheric HAPS at $z = 20,000\text{ m}$.
- Demonstrates active packet forwarding and dynamic handovers (`Relay 30 -> Relay 31 -> Relay 33`) as UAVs move in and out of range.

---

## 6. References

- [ns-3.46 Mobility Model Documentation](https://www.nsnam.org/docs/release/3.46/models/html/mobility.html)
- [ns-3.46 MobilityModel API](https://www.nsnam.org/docs/release/3.46/doxygen/db/ddb/classns3_1_1_mobility_model.html)
