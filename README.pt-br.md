[![en](https://img.shields.io/badge/lang-en-red.svg)](/README.md)
[![pt-br](https://img.shields.io/badge/lang-pt--br-green.svg)](/README.pt-br.md)

# Juca

Juca is an embedded mobile robotics board designed specifically for education. Juca combines affordability, expandability, and robust functionality, making it suitable for classroom and competition environments. Key features include modular hardware akin to Arduino shields, support for differential drive robots, and seamless integration with educational programming platforms.

This repository contains all the resources related to the development of the rover, 
including **firmware**, **software**, and **hardware** (electronics and mechanics). 

For information on the motivation, design objectives, and development criteria of the robot,  
please refer to the article *“Juca: an embedded mobile robotics board for education”*,  
published in the *16th Workshop on Robotics in Education (WRE 2025)*.

## 📂 Repository Structure

my-robot-project/
│
├── docs/ # Project documentation
│
├── firmware/ # Microcontroller source code
│
├── software/ # PC applications, scripts, and simulations
│
├── hardware/
│ ├── electronics/
│ │ ├── kicad/ # Schematics and PCB layouts (KiCad project files)
│ │ └── bom/ # Bill of Materials
│ │
│ └── mechanics/ # Mechanical models (CAD, STL, STEP, drawings)
│
├── tests/ # Integration tests (firmware + hardware + software)
│
├── tools/ # Utility scripts and helper tools
│
├── LICENSE
└── README.md