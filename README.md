# VUR-CyclistAEB-DS
A multi-scenario AEB test dataset for the interaction between vehicles and cyclists

Original_data_read.py
Description: 
  Parses CAN bus data in .trc format, extracts key parameters (e.g., position, velocity, acceleration, time-to-collision) 
  from vehicle-cyclist interaction scenarios, exports parsed results to CSV files.
Primary Data Sources:
  CAN bus messages (including vehicle status, position, and motion parameters with IDs 0x600-0x60F)
Output: 
  CSV files containing message number, timestamp, message ID, payload, and dynamic parameters (see CSV headers).
