# M03 - UAV/HAPS Aerial Network and Relay Model

**Module Name:** `futurenet-aerial`  
**Version:** 1.0  
**Framework:** NS-3.46.1  
**Course:** Computer Networks (CN)

## 1. Project Overview

This project aims to develop a reusable UAV/HAPS aerial network and relay model using NS-3. The simulation focuses on 3D mobility, connectivity, relay selection, and service availability.

## 2. Objectives

- Configure UAV and HAPS nodes with 3D mobility.
- Model connectivity between ground and aerial nodes.
- Implement replaceable relay selection policies.
- Support ground-to-aerial-to-ground communication.
- Record mobility, relay changes, and service availability.

## 3. Key Features

- Ground, UAV, and HAPS node roles.
- Constant-position and waypoint mobility support.
- Configurable range-based connectivity.
- Nearest-relay selection policy.
- Automatic alternative relay selection when a relay becomes unavailable.
- Position and relay-state tracing.
- CSV output for simulation analysis.

## 4.  Simulation Requirements

- At least 30 ground nodes.
- At least 5 aerial nodes.
- Moving UAVs.
- At least one relay selection change during simulation.
- Tests for range eligibility, nearest-relay selection, relay loss, and mobility tracing.

## 5. Planned Examples

1. **Basic Relay:** Ground source → moving UAV relay → ground destination.
2. **Multi-Relay:** Multiple UAV relays with relay selection changing as they move.

## 6. Reference

- [NS-3.46 Mobility Model Documentation](https://www.nsnam.org/docs/release/3.46/models/html/mobility.html)
- [NS-3.46 MobilityModel API](https://www.nsnam.org/docs/release/3.46/doxygen/db/ddb/classns3_1_1_mobility_model.html)
