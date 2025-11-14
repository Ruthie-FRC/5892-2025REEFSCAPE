# Pipeline Folder

This folder contains the geofencing and input-processing pipeline for the robot. 

- Physical walls are defined in PhysicalWall.java
- Geofencing fences control max speed toward walls
- Each filter processes inputs in sequence through GeofencePipeline.java
- FilterContext.java carries all input/output data
