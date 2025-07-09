# SHOPILOT - Autonomous Shopping Assistant

**A Smart Shopping Cart that Opens New Paths for Shopping**
<img width="445" height="640" alt="Image" src="https://github.com/user-attachments/assets/68914d0c-6189-46b3-b56b-2f39da7f7563" />

https://youtu.be/pP-fz-G6JYQ

You can view the demonstration video at the link above (Korean language).


## Overview

SHOPILOT is the result of a capstone project focused on developing an autonomous shopping assistant robot. This repository contains the complete ROS2 source packages for the entire vehicle system(except web pages and chat bot), implementing an intelligent shopping cart that can navigate retail environments, assist customers with shopping tasks, and provide interactive AI-powered assistance.


### Key Features

- **LLM-Based Smart Shopping List**: Natural language processing for intuitive shopping assistance(not included in this rep.)
- **Autonomous Navigation**: Advanced path planning and obstacle avoidance in retail environments  
- **Real-time Map Visualization**: Interactive web-based store navigation with live updates
- **Multi-Modal Interaction**: Voice commands, touchscreen interface, and visual feedback
- **Product Recognition & Management**: AI-powered inventory tracking and product recommendations
- **Customer Leading**: Autonomous customer tracking and assistance
- **Safety Systems**: Comprehensive collision avoidance and emergency protocols

## System Architecture

<img width="1144" height="560" alt="Image" src="https://github.com/user-attachments/assets/e03324af-21c0-4f2a-af2c-e552d320b67c" />

## Hardware Configuration

### Core Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Mobile Platform** | Scout-Mini | Autonomous navigation base |
| **Main Computer** | Intel NUC | Control system and AI processing |
| **Sub Computer** | Jetson Orin Nano | Real-time camera processing |
| **LiDAR** | 2D-RPLiDAR | SLAM and obstacle detection |
| **Front Camera** | RealSense D457 | Human detection and interaction |
| **Rear Camera** | RealSense D435i | Customer tracking and following |

### SHOPILOT Physical Design
- Custom shopping cart frame with integrated technology
- Touchscreen display for customer interaction
- Storage basket for collected items
- Emergency stop button and safety systems

## Software Architecture

### Core Software Stack

| Layer | Technologies | Purpose |
|-------|-------------|---------|
| **AI & Database** | FAISS, Claude 3.7, Whisper, Vector DB | Smart assistance and NLP |
| **Web Interface** | React, Next.js, TypeScript, WebSocket | User interaction and visualization |
| **Robot Control** | ROS2 Humble, Nav2, SLAM Toolbox | Navigation and control |
| **Communication** | ROS Bridge, DDS | Real-time data exchange |

### Key Software Modules

#### 1. Navigation System (Nav2)
- **SLAM**: Real-time mapping using slam_toolbox with RPLiDAR A2M8
- **Path Planning**: Global planning with NavFnPlanner (Dijkstra algorithm)  
- **Local Control**: DWB Controller for dynamic obstacle avoidance
- **Localization**: AMCL particle filter for precise positioning

#### 2. Vision Systems
- **Human Detection**: YOLO v8 segmentation with DEEP-SORT tracking
- **Obstacle Detection**: 3D point cloud processing for front path clearing
- **Customer Leading**: Real-time person tracking and distance maintenance

#### 3. AI Assistant (RAG Architecture)
- **Knowledge Base**: 30 categories × 50 products = 1,500 product database
- **Natural Language**: Voice input processing with shopping intent recognition
- **Smart Recommendations**: Context-aware product suggestions
- **Multi-modal Input**: Text, voice, and visual query processing

#### 4. Web Interface (not included in this rep.)
- **Real-time Store Map**: Interactive visualization with live robot position
- **Shopping List Management**: Digital list with AI-powered optimization
- **Voice Integration**: Hands-free operation with speech recognition
- **Product Information**: Real-time inventory and pricing data

### Hardware Requirements

Based on the system configuration shown in the presentation:

| Component | Model | Purpose |
|-----------|-------|---------|
| **Mobile Platform** | Scout-Mini | Autonomous navigation base |
| **Main Computer** | Intel NUC | Control system and Overall data processing |
| **Sub Computer** | Jetson Orin Nano | Real-time camera processing |
| **LiDAR** | 2D-RPLiDAR | SLAM and obstacle detection |
| **Front Camera** | RealSense D457 | Human detection and interaction |
| **Rear Camera** | RealSense D435i | Customer tracking and following |

### Software Dependencies

Core technologies used in the system:

| Layer | Technologies |
|-------|-------------|
| **AI & Database** | FAISS, Claude 3.7, Whisper, Vector DB |
| **Web Interface** | React, Next.js, TypeScript, WebSocket |
| **Robot Control** | ROS2 Humble, Nav2, SLAM Toolbox |
| **Communication** | ROS Bridge, DDS |

### Repository Setup

```bash
# Clone the repository
git clone https://github.com/kjin3386/Autonomous-Shopping-Assistant.git
cd Autonomous-Shopping-Assistant

# Follow the installation instructions in the repository's documentation
```


### Navigation Capabilities
- **SLAM-based mapping and localization** using slam_toolbox
- **Path planning and obstacle avoidance** with Nav2 framework
- **Customer detection and tracking** using computer vision
- **Emergency stop and safety protocols**

### AI Assistant Features  
- **Natural language processing** for shopping queries
- **Product database management** with 1,500+ items across 30 categories
- **Voice interaction** using Whisper speech recognition
- **RAG (Retrieval Augmented Generation)** architecture for intelligent responses

### Web Interface Features
- **Real-time store map visualization** showing robot position
- **Interactive shopping list management** 
- **Voice control integration**
- **Product information display**

## Technical Implementation

Based on the system architecture shown in the presentation:

### Core Navigation System
- **SLAM**: slam_toolbox implementation for mapping and localization
- **Navigation**: Nav2 framework with behavior trees
- **Sensors**: RPLiDAR for laser scanning, RealSense cameras for vision
- **Platform**: Scout-Mini mobile robot base

### Vision Processing
- **Human Detection**: YOLO v8 segmentation for person tracking
- **Depth Processing**: RGB-D camera data for obstacle detection  
- **Tracking**: DEEP-SORT algorithm for multi-object tracking

### AI Backend
- **LLM Integration**: Claude 3.7 for natural language understanding
- **Speech Processing**: Whisper for voice recognition
- **Knowledge Base**: FAISS vector database with product information
- **RAG Architecture**: Retrieval-augmented generation for shopping assistance

### Communication Architecture
- **ROS2 Bridge**: WebSocket communication between web interface and robot
- **DDS**: Real-time data distribution for ROS2 nodes
- **Web Stack**: React frontend with TypeScript and Next.js



## Acknowledgments

- **SCOUT-MINI Platform**: AgileX Robotics for the mobile base
- **RPLiDAR**: Reliable laser scanning solutions
- **Open Source Community**: ROS2, OpenCV, and web development frameworks
