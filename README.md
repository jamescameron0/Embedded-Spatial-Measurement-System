# Embedded-Spatial-Measurement-System

This repository is my Spatial Measurement System project, an affordable setup for scanning indoor spaces and generating 3D models. 

I combined a VL53L1X time-of-flight sensor on a 28BYJ-48 stepper motor with an MSP432E401Y microcontroller to perform 360° scans. The microcontroller sends distance data to a PC, where a Python script converts it into 3D point clouds using Open3D. This project demonstrates real-time spatial measurement and visualization.
