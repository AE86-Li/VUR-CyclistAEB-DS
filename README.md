# VUR-CyclistAEB-DS
# A multi-scenario AEB test dataset for the interaction between vehicles and cyclists

# Original_data_read.py

Description: 
  Parses CAN bus data in .trc format, extracts key parameters (e.g., position, velocity, acceleration, time-to-collision) 
  from vehicle-cyclist interaction scenarios, exports parsed results to CSV files.
  
Primary Data Sources:
  CAN bus messages (including vehicle status, position, and motion parameters with IDs 0x600-0x60F)
  
For example, the message ID is 0x601:

```python
if msg.messageId == 0x601 and len(msg.payload) >= 2:
    PosLon = msg.GetFloats(offset=4, factor=1e-7, fmt=4)
    PosLat = msg.GetFloats(offset=0, factor=1e-7, fmt=4)
else:
    PosLon = "N/A"
    PosLat = "N/A"
```
  
Output: 
  CSV files containing message number, timestamp, message ID, payload, and dynamic parameters (see CSV headers).
# 
# Original_data_split.py

Description: 
  Splits a large CSV file containing vehicle-cyclist interaction data into multiple 
  smaller CSVs based on parameter groups, preserving essential metadata columns.

Primary Input: 
  CSV file generated by CANMessage parser.
  
Output: 
  Multiple CSV files named by parameter group and physical quantity (e.g., "_01_Longitude_Latitude.csv").
# 
Grouping information such as:
```python
{
    "Longitude_Latitude",
    "Altitude",
    "2D_Speed",
    "3Axis_Angular_Acceleration",
    "Longitudinal_Lateral_Velocity",
    "3Axis_Linear_Acceleration",
    "Longitudinal_Lateral_Slip_Acceleration",
    "Heading_Pitch_Roll_Angle",
    "3Axis_Angular_Rate",
    "Longitudinal_Lateral_Angular_Rate",
    "Distance",
    "Local_XY_Position",
    "Local_XY_Velocity",
    "Local_Yaw_Track_Angle",
    "Longitudinal_Lateral_Angular_Acceleration"
}
  ```
