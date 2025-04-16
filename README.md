BACKEND : https://github.com/OussamaGhz/warehouse-robot-system /n
ROBOT : https://github.com/WRH-05/RoboWare /n
FRONTEND-dashboard : https://github.com/Beccaworsnop/nest-hackathon /n
FRONTEND-portfolio : https://github.com/Beccaworsnop/nest-front
# SmartWare Warehouse Manager

SmartWare is an autonomous warehouse management robot designed to streamline logistics operations. Built as a line follower robot, SmartWare leverages an optimized Dijkstra algorithm to navigate complex environments—particularly excelling in sharp angle turns. The robot uses integrated RFID scanning to identify locations and packages, controls a stepper motor-driven crank shift for elevation adjustments, and communicates via MQTT for robust real-time messaging.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Setup & Installation](#setup--installation)
- [Usage](#usage)
- [Future Enhancements](#future-enhancements)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Overview

SmartWare was developed to efficiently manage warehouse operations by:
- Following a dedicated line map using advanced path planning with a Dijkstra algorithm.
- Reacting to embedded RFID tags distributed around the workspace.
- Automating the retrieval and placement of boxes on shelves using a crank shift mechanism.
- Maintaining communication over MQTT, suited for IoT devices in bandwidth-constrained environments.

By combining traditional navigation methods with plans for advanced machine learning-based routing, SmartWare aims to gradually evolve into a fully autonomous system free from reliance on physical line paths.

## Features

- **Line Following Navigation:**  
  Uses a Dijkstra algorithm optimized for sharp angle turns to determine the best route along pre-determined paths.

- **RFID Integration:**  
  - **On-Board Scanner:** Detects RFID tags placed around the warehouse, prompting the robot to stop and initiate handling routines.
  - **Crank Shift Platform Scanner:** At the top of the elevator mechanism, an RFID scanner confirms a box's ID and selects the appropriate shelf level.

- **Crank Shift Elevation Control:**  
  The `elevation.ino` code controls a stepper motor that adjusts a crank shift mechanism, raising or lowering the handling platform to match the target shelf level.

- **MQTT Communication:**  
  Utilizes the MQTT protocol for lightweight, reliable, machine-to-machine messaging—ideal for managing a network of resource-constrained devices.

## Hardware Components

- **Chassis & Locomotion:**  
  Designed as a line follower with sensors for path detection, optimized for precision on sharp turns.

- **RFID Scanners:**  
  - Primary scanner for reading tags on the floor.
  - Secondary scanner mounted on the elevation platform to determine box IDs.

- **Stepper Motor & Crank Shift Mechanism:**  
  Ensures precise vertical movements based on shelf level requirements.

- **Computing Unit:**  
  Runs the navigation algorithms and coordinates sensor inputs, motor outputs, and MQTT communications.

## Software Architecture

- **Navigation & Path Planning:**  
  The core navigation algorithm is built around Dijkstra’s shortest path algorithm adapted to handle the challenges of sharp angle turns in a warehouse environment.

- **Sensor Integration & Event Handling:**  
  RFID events are used to trigger state transitions (e.g., pausing navigation, activating the elevation system).

- **Elevation Control:**  
  The `elevation.ino` sketch handles stepper motor operations to move the crank shift mechanism up and down. It adjusts the platform height based on the RFID confirmation from the box scanning on the elevator.

- **Communication Layer:**  
  The MQTT client connects to a central broker, enabling remote control and data collection. This facilitates an IoT approach that could be expanded to include data logging and real-time system monitoring.

## Setup & Installation

### Prerequisites

- **Hardware:**  
  - Fully assembled SmartWare robot with sensors, stepper motors, RFID scanners, and computing unit.
- **Software & Libraries:**  
  - Arduino IDE (or relevant embedded system development environment)
  - MQTT library for Arduino (e.g., [PubSubClient](https://pubsubclient.knolleary.net/))
  - Stepper motor control libraries
  - RFID reader libraries

### Installation Steps

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/WRH-05/RoboWare
   cd smartware
