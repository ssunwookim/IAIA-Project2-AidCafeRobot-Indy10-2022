# IAIA-Project2-AidCafeRobot-Indy10-2024



# Ade Cafe Robot

**Date:** 2024.12.20

**Author:**

22000090 | Sunwoo Kim

21800360 | Sion Seo

22100645 | Hojin Jang

**Github :** [IAIA-Project2-AidCafeRobot-Indy10-2022](https://github.com/ssunwookim/IAIA-Project2-AidCafeRobot-Indy10-2022)

**Demo Video :** [24-2 IAIA Project Ade Cafe Robot ](https://www.youtube.com/watch?v=TiJDjN4B75o)

**Short Video :** [24-2 IAIA Project Ade Cafe Robot ](https://www.instagram.com/reel/DD337D3oMmM/?utm_source=ig_web_copy_link&igsh=MzRlODBiNWFlZA==)



## Introduction

 This repository contains a tutorial on building a cafe robot system that automatically produces and provides four types of aids as a final project for the Industrial AI and Automation class conducted in the second semester of 2024 at Handong Global University.

- **Background**

   The global café market is experiencing steady growth and is projected to reach a valuation of 47.2 billion dollars within the next five years. One significant issue in the industry is the variability in drink quality and quantity, which directly depends on the skill and consistency of the workforce. This variability often results in inconsistent customer experiences and affects overall business efficiency.

- **Problem**

   The accuracy and consistency of drink preparation vary significantly depending on the individual worker. This inconsistency poses challenges in maintaining a high standard of service. Additionally, training new workers is both time-consuming and costly, typically requiring up to three months to become fully proficient, which significantly impacts work efficiency and operational throughput.

- **Goal** 

   There is a critical need for the automation of drink preparation processes to ensure precise and consistent output within a specified sampling time, ideally making each drink in under two minutes using an Indy 10 robot. Moreover, the robot system must be equipped to monitor material levels, identify when stocks are low, and facilitate timely replenishments to maintain uninterrupted service. This capability will enhance operational efficiency, reduce dependency on human labor, and ensure a high-quality customer experience consistently.



**This project was conducted under the guidance of Professor Young-Geun Kim of the Department of Mechanical and Control Engineering at Handong University.**



## Requirements

### Hardware

**Co-Robot**

- indy- 10

**Gripper**

- onrobot-rg2 grippers
- Dedicated LAN cable

**Camera**

- Webcam x2 (Microsoft LifeCam Studio)

**PC** 

- Before the Webcam connection, you **should** check if your PC recognizes two cameras at the same time via the hub. If it is not recognized via the hub, when connecting the camera,  the program is executed by connecting to the **two ports**, and a lower index  is assigned in the order of connection



### Software

**Python**

- 3.8.20

**QR Detection**

- pyzbar 0.1.9

**Image Processing**

- opencv-python 4.10.0.84

**Gripper**

- pymodbus 2.5.3 (Guide: [Link](https://github.com/takuya-ki/onrobot-rg))



## Contents

- #### ROS Environment-Indy10  & OnRobot Gripper RG2 Setting
[Reference](https://github.com/ssunwookim/IAIA-Project2-AidCafeRobot-Indy10-2022/blob/main/report/0.%20ROS%20environment-Indy10%20%26%20onRobot%20Gripper%20rg2%20setting.md)

- #### Hardware setting

[Reference](https://github.com/ssunwookim/IAIA-Project2-AidCafeRobot-Indy10-2022/blob/main/report/1.%20Hardware%20setting.md)

- #### Ade Robot Algorithm

[Reference](https://github.com/ssunwookim/IAIA-Project2-AidCafeRobot-Indy10-2022/blob/main/report/2.%20Ade%20Robot%20Algorithm.md)

## Reference

[HGU_IAIA/Tutorial/TU_ROS at main · ykkimhgu/HGU_IAIA](https://github.com/ykkimhgu/HGU_IAIA/tree/main/Tutorial/TU_ROS)

[로봇 에이드 카페 - 협동로봇 적용 사례](https://www.youtube.com/watch?v=ab3BU_RHz2c)
