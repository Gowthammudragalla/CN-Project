# M03 - UAV/HAPS Aerial Network and Relay Model
**Module Name:** futurenet-aerial  
**Target Simulator:** ns-3.46.1  

---

## 📁 Project Directory Structure (`cn project/`)

All files related to this project are organized here:

| File Name | Description |
| :--- | :--- |
| `m03-aerial-relay.cc` | Main C++ simulation source code implementing all SRS classes and demonstration scale |
| `plot_aerial_relay.py` | Python visualization script to plot 2D/3D UAV trajectories and relay handover timeline |
| `aerial-relay-traces.csv` | Output CSV trace file containing real-time simulation metrics, coordinates, and relay status |
| `aerial_relay_analysis.png` | Generated analysis plot showing UAV coverage areas, trajectories, and dynamic handovers |
| `README.md` | Comprehensive documentation, execution commands, and project details |

---

## 🚀 How to Build and Run

From the root directory (`ns-3.46.1/`):

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

### 6. Generate / Update Visual Graphs
```bash
python3 "cn project/plot_aerial_relay.py" "cn project/aerial-relay-traces.csv" "cn project/aerial_relay_analysis.png"
```

---

## 🧠 Key Components Implemented

1. **`AerialNodeModel`**:
   - Manages logical node roles: `GROUND`, `UAV`, and `HAPS`.
   - Tracks `ResidualEnergyFraction` ($0.0$ to $1.0$).

2. **`AerialLinkModel`**:
   - Calculates 3D Euclidean distances $\sqrt{\Delta x^2 + \Delta y^2 + \Delta z^2}$.
   - Enforces configurable communication range limits (`MaxGroundAerialRange = 2000m`, `MaxAerialAerialRange = 5000m`).
   - Computes path propagation and base processing delay ($1\text{ ms}$).

3. **`RelaySelectionPolicy` (Interface)**:
   - **`NearestRelayPolicy`**: Selects the eligible relay minimizing total Euclidean path distance.
   - **`LowestDelayRelayPolicy`**: Selects the eligible relay with minimum end-to-end delay.

4. **`AerialRelayManager`**:
   - Periodically evaluates candidate eligibility.
   - Enforces `MinRelayDwellTime` ($2\text{ s}$) to prevent route flapping/hysteresis.
   - Triggers traces: `AerialPositionUpdate`, `RelayCandidateChange`, `RelaySelectionChange`, `RelayUnavailable`.
   - Produces the specified CSV trace format.

5. **`AerialRelayApplication`**:
   - Receives UDP packets on port $9000$ and forwards them to the destination when active.
