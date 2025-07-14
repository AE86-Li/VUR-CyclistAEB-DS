# Description: 
#   Parses CAN bus data in .trc format, extracts key parameters (e.g., position, velocity, acceleration, time-to-collision) 
#   from vehicle-cyclist interaction scenarios, exports parsed results to CSV files.
# Author: [Your Name/Team Name]
# Application Scenarios: 
#   Multi-scenario cyclist AEB test data analysis.
# Primary Data Sources:
#   - CAN bus messages (including vehicle status, position, and motion parameters with IDs 0x600-0x60F)
# Output: 
#   CSV files containing message number, timestamp, message ID, payload, and dynamic parameters (see CSV headers).

import struct
import csv
import os

class CANMessageInfo:
    def __init__(self, lineString, timeMs=None):
        """
        Parses a line of text to construct a CANMessageInfo object.
        Original C# code logic:
          - tokens[0]: Message number (remove trailing symbol, e.g., "2)" becomes "2")
          - tokens[1]: Time offset
          - tokens[4]: Message ID (hexadecimal)
          - tokens[6]: Data length
          - tokens[7..]: Payload (hexadecimal, each token represents a byte)
        """
        tokens = lineString.split()
        # Remove the trailing right parenthesis from tokens[0] (e.g., "2)" -> "2")
        message_number_str = tokens[0].rstrip(')')
        self.messageNumber = int(message_number_str)
        self.timeOffset = float(tokens[1])
        # tokens[2] and tokens[3] (bus number, direction) are not processed
        self.messageId = int(tokens[4], 16)
        self.length = int(tokens[6])
        # Parse payload: take 'length' bytes starting from tokens[7]
        payload_tokens = tokens[7:7+self.length]
        self.payload = bytearray(int(tok, 16) for tok in payload_tokens)
        
        # Default timeMs equals timeOffset, use the provided timeMs if available
        if timeMs is None:
            self._timeMs = self.timeOffset
        else:
            self._timeMs = timeMs

        self._update_timeString()

    def _update_timeString(self):
        """
        Calculates the time string (format: hour:minute:second:ms) based on _timeMs.
        Note: To maintain consistency with C#, milliseconds are truncated to an integer.
        """
        total = int(self._timeMs)
        hour = total // 3600000
        rem = total % 3600000
        minute = rem // 60000
        rem = rem % 60000
        second = rem // 1000
        ms = rem % 1000
        self.timeString = f"{hour}:{minute}:{second}:{ms}"

    @property
    def timeMs(self):
        return self._timeMs

    @timeMs.setter
    def timeMs(self, value):
        self._timeMs = value
        self._update_timeString()

    def GetTimeMs(self):
        """
        Calculates the time (in milliseconds) from bytes 5-8 of the payload using the formula:
          payload[7] * 3600000 + payload[6] * 60000 + payload[5] * 1000 + payload[4] * 10
        Returns 0 if the payload length is less than 8 bytes.
        """
        if len(self.payload) >= 8:
            return (self.payload[7] * 3600000 +
                    self.payload[6] * 60000 +
                    self.payload[5] * 1000 +
                    self.payload[4] * 10)
        return 0

    def GetFloats(self, offset, factor, fmt):
        """
        Parses data from the payload according to fmt and multiplies by factor.
          - fmt == 1: Parse as unsigned 2 bytes
          - fmt == 2: Parse as signed 16 bits
          - fmt == 4: Parse as signed 32 bits
          - fmt == 8: Parse as signed 64 bits
          - Default (if fmt is None): Parse as Int16
        """

        if fmt is None:
            return self._get_int16(offset) * factor
        else:
            if fmt == 1:
                return self._get_char(offset) * factor
            elif fmt == 2:
                return self._get_int16(offset) * factor
            elif fmt == 4:
                return self._get_int32(offset) * factor
            elif fmt == 8:
                return self._get_int64(offset) * factor
            else:
                return self._get_int16(offset) * factor

    def _get_char(self, offset):
        return struct.unpack_from('<H', self.payload, offset)[0]

    def _get_int16(self, offset):
        if len(self.payload) < offset + 2:
            # Handle insufficient data according to requirements (e.g., return default value or skip)
            return 0  # Return 0 as default
        return struct.unpack_from('<h', self.payload, offset)[0]

    def _get_int32(self, offset):
        return struct.unpack_from('<i', self.payload, offset)[0]

    def _get_int64(self, offset):
        return struct.unpack_from('<q', self.payload, offset)[0]

class CANMessage:
    def __init__(self, fileName):
        """
        Parses CAN messages from a .trc file and generates a CSV file.
        Logic explanation (referencing C# code):
          - Skip the first 21 lines (and any empty or comment lines)
          - Process only lines longer than 40 characters
          - For messages with messageId==0x600, use GetTimeMs() to get absolute time
            and adjust timestamps of all previous messages
          - Timestamps of other messages are calculated by accumulating offsets from the last 0x600 message
          - Results are written to a CSV file with the same name as the source file but with .csv extension
        """
        self.messageList = []
        lastTimeMessage = None
        ifTimeMessage = False
        PosLat = None
        PosLon = None
        AccelX = None
        AccelY = None
        AccelZ =None
        AngRateX =None
        AngRateY=None
        AngRateZ=None
        Altitude = None
        AngAccelX = None
        AngAccelY = None
        AngAccelZ = None
        VelForward = None
        VelLateral= None
        Speed2D = None
        AccelForward  = None
        AccelLateral  = None
        AccelSlip = None
        AngleHeading = None
        AnglePitch = None
        AngleRoll = None
        AngRateForward = None
        AngRateLateral = None
        DistanceWithHold = None
        Distance = None
        AngAccelForward = None
        AngAccelLateral = None
        VelLocalX = None
        VelLocalY= None
        AngleLocalYaw = None
        AngleLocalTrack= None


        # Generate the CSV file name in the same directory with the same base name
        base, _ = os.path.splitext(fileName)
        csv_filename = base + ".csv"

        with open(fileName, 'r', encoding='utf-8') as f_in:
            lines = f_in.readlines()

        with open(csv_filename, 'w', newline='', encoding='utf-8') as f_out:
            writer = csv.writer(f_out)
            # Write CSV headers
            writer.writerow([
                "MessageNumber",
                "TimeOffset",
                "MessageID(hex)",
                "Length",
                "Payload(hex)",
                "TimeMs",
                "TimeString",
                "PosLon","PosLat",
                "Altitude",
                "Speed2D",
                "AngAccelX","AngAccelY","AngAccelZ",
                "VelForward","VelLateral",
                "AccelX","AccelY","AccelZ",
                "AccelForward", "AccelLateral","AccelSlip", 
                "AngleHeading","AnglePitch","AngleRoll",
                "AngRateX","AngRateY","AngRateZ",
                "AngRateForward ","AngRateLateral",
                "DistanceWithHold","Distance ",
                "PosLocalX","PosLocalY",
                "VelLocalX","VelLocalY","AngleLocalYaw ","AngleLocalTrack",
                "AngAccelForward °/s²","AngAccelLateral °/s²"
            ])

            lineNumber = 0
            for s in lines:
                lineNumber += 1
                s = s.strip()
                # Skip empty lines or comment lines starting with ';'
                if not s or s.startswith(';'):
                    continue
                # As per C# code logic: Process only lines after the 21st and longer than 40 characters
                if lineNumber < 21 or len(s) <= 40:
                    continue

                try:
                    msg = CANMessageInfo(s)
                except Exception as e:
                    print(f"Error parsing line {lineNumber}: {s}\nError: {e}")
                    continue

                if msg.messageId == 0x600:
                    msg.timeMs = msg.GetTimeMs()
                    if not ifTimeMessage:
                        ifTimeMessage = True
                        # Adjust timestamps of all previous messages
                        for m in self.messageList:
                            m.timeMs = msg.timeMs - (msg.timeOffset - m.timeOffset)
                    lastTimeMessage = msg
                elif ifTimeMessage and lastTimeMessage is not None:
                    msg.timeMs = (lastTimeMessage.timeMs +
                                  (msg.timeOffset - lastTimeMessage.timeOffset))

                self.messageList.append(msg)

                # Update position data if message ID is 0x601
                if msg.messageId == 0x601 and len(msg.payload) >= 2:
                    PosLon  = msg.GetFloats(offset=4,factor=1e-7,fmt=4)
                    PosLat  = msg.GetFloats(offset=0,factor=1e-7,fmt=4)
                else:
                    PosLon  = "N/A"
                    PosLat  = "N/A"

                # Update altitude data if message ID is 0x602
                if msg.messageId == 0x602 and len(msg.payload) >= 2:
                    Altitude  = msg.GetFloats(offset=0,factor=0.001,fmt=4)
                else:
                    Altitude  = "N/A"

                # Update speed data if message ID is 0x603
                if msg.messageId == 0x603 and len(msg.payload) >= 2:
                    Speed2D = msg.GetFloats(offset=6,factor=0.01,fmt=2)
                else:
                    Speed2D  = "N/A"

                # Update angular acceleration data if message ID is 0x60E
                if msg.messageId == 0x60E and len(msg.payload) >= 2:
                    AngAccelX = msg.GetFloats(offset=0,factor=0.1,fmt=2)
                    AngAccelY = msg.GetFloats(offset=2,factor=0.1,fmt=2)
                    AngAccelZ = msg.GetFloats(offset=4,factor=0.1,fmt=2)
                else:
                    AngAccelX  = "N/A"
                    AngAccelY  = "N/A"
                    AngAccelZ  = "N/A"

                # Update velocity data if message ID is 0x604
                if msg.messageId == 0x604 and len(msg.payload) >= 2:
                    VelForward = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    VelLateral  = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                else:
                    VelForward  = "N/A"
                    VelLateral = "N/A"

                # Update acceleration data if message ID is 0x605
                if msg.messageId == 0x605 and len(msg.payload) >= 2:
                    AccelX  = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    AccelY  = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                    AccelZ  = msg.GetFloats(offset=4,factor=0.01,fmt=2)
                else:
                    AccelX  = "N/A"
                    AccelY  = "N/A"
                    AccelZ  = "N/A"

                # Update forward/lateral acceleration data if message ID is 0x606
                if msg.messageId == 0x606 and len(msg.payload) >= 2:
                    AccelForward  = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    AccelLateral  = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                    AccelSlip   = msg.GetFloats(offset=6,factor=0.01,fmt=2)
                else:
                    AccelForward  = "N/A"
                    AccelLateral  = "N/A"
                    AccelSlip   = "N/A"

                # Update angle data if message ID is 0x607
                if msg.messageId == 0x607 and len(msg.payload) >= 2:
                    AngleHeading  = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    AnglePitch  = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                    AngleRoll    = msg.GetFloats(offset=4,factor=0.01,fmt=2)
                else:
                    AngleHeading  = "N/A"
                    AnglePitch  = "N/A"
                    AngleRoll    = "N/A"

                # Update angular rate data if message ID is 0x608
                if msg.messageId == 0x608 and len(msg.payload) >= 2:
                    AngRateX  = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    AngRateY  = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                    AngRateZ  = msg.GetFloats(offset=4,factor=0.01,fmt=2)
                else:
                    AngRateX  = "N/A"
                    AngRateY  = "N/A"
                    AngRateZ  = "N/A"

                # Update forward/lateral angular rate data if message ID is 0x609
                if msg.messageId == 0x609 and len(msg.payload) >= 2:
                    AngRateForward   = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    AngRateLateral   = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                else:
                    AngRateForward  = "N/A"
                    AngRateLateral   = "N/A"

                # Update distance data if message ID is 0x60B
                if msg.messageId == 0x60B and len(msg.payload) >= 2:
                    DistanceWithHold = msg.GetFloats(offset=0,factor=0.001,fmt=2)
                    Distance= msg.GetFloats(offset=4,factor=0.001,fmt=2)
                else:
                    DistanceWithHold  = "N/A"
                    Distance  = "N/A"

                # Update local position data if message ID is 0x60C
                if msg.messageId == 0x60C and len(msg.payload) >= 2:
                    PosLocalX = msg.GetFloats(offset=0,factor=0.0001,fmt=4)
                    PosLocalY = msg.GetFloats(offset=4,factor=0.0001,fmt=4)
                else:
                    PosLocalX  = "N/A"
                    PosLocalY  = "N/A"

                # Update local velocity and angle data if message ID is 0x60D
                if msg.messageId == 0x60D and len(msg.payload) >= 2:
                    VelLocalX  = msg.GetFloats(offset=0,factor=0.01,fmt=2)
                    VelLocalY = msg.GetFloats(offset=2,factor=0.01,fmt=2)
                    AngleLocalYaw  = msg.GetFloats(offset=4,factor=0.01,fmt=2)
                    AngleLocalTrack = msg.GetFloats(offset=6,factor=0.01,fmt=2)
                else:
                    VelLocalX  = "N/A"
                    VelLocalY  = "N/A"
                    AngleLocalYaw  = "N/A"
                    AngleLocalTrack  = "N/A"

                # Update angular acceleration data if message ID is 0x60F
                if msg.messageId == 0x60F and len(msg.payload) >= 2:
                    AngAccelForward  = msg.GetFloats(offset=0,factor=0.1,fmt=2)
                    AngAccelLateral  = msg.GetFloats(offset=2,factor=0.1,fmt=2)
                else:
                    AngAccelForward   = "N/A"
                    AngAccelLateral   = "N/A"

                # Write a row to CSV
                writer.writerow([
                    msg.messageNumber,
                    msg.timeOffset,
                    hex(msg.messageId),
                    msg.length,
                    ' '.join(f"{b:02X}" for b in msg.payload),
                    msg.timeMs,
                    msg.timeString,
                    PosLon,PosLat,
                    Altitude,
                    Speed2D,
                    AngAccelX,AngAccelY,AngAccelZ,
                    VelForward,VelLateral, 
                    AccelX,AccelY,AccelZ,
                    AccelForward ,AccelLateral,AccelSlip, 
                    AngleHeading,AnglePitch,AngleRoll, 
                    AngRateX,AngRateY,AngRateZ,
                    AngRateForward,AngRateLateral,
                    DistanceWithHold,Distance,
                    PosLocalX,PosLocalY,
                    VelLocalX,VelLocalY,AngleLocalYaw,AngleLocalTrack,
                    AngAccelForward,AngAccelLateral  
                ])

        print(f"Parsing completed. Results saved to: {csv_filename}")

    @property
    def messageCount(self):
        return len(self.messageList)

    def GetMessageList(self, id=None):
        """
        Returns all messages if id is None;
        Returns messages with messageId equal to id if specified, or None if no matches found.
        """
        if id is None:
            return self.messageList
        filtered = [m for m in self.messageList if m.messageId == id]
        return filtered if filtered else None

    def GetBreakLight(self):
        """
        For messages with ID 0x570, recalculate timeMs:
          payload[0]*3600000 + payload[1]*60000 + (payload[2,3] parsed as UInt16)
        """
        breakLightMessageList = self.GetMessageList(0x570)
        if breakLightMessageList:
            for m in breakLightMessageList:
                if len(m.payload) >= 4:
                    val = struct.unpack_from('<H', m.payload, 2)[0]
                    m.timeMs = (m.payload[0] * 3600000 +
                                m.payload[1] * 60000 +
                                val)
        return None


# --------------------- Python implementation of the "missing C# part" ---------------------

class PointCarrier:
    """
    Equivalent to C#'s PointCarrier, used to record time, position, speed, integrated position, etc.
    """
    def __init__(self, time_val, position, speed, position_integration=0.0):
        """
        :param time_val:   Time (milliseconds)
        :param position:   Current position
        :param speed:      Current speed
        :param position_integration: Integrated position
        """
        self.time = time_val
        self.position = position
        self.speed = speed
        self.positionIntegration = position_integration
        self.timeString = self._time2string()

    def _time2string(self):
        """
        Converts self.time (milliseconds) to a "hour:minute:second:ms" string,
        consistent with C# code logic.
        """
        hour = self.time // 3600000
        remainder = self.time % 3600000
        minute = remainder // 60000
        remainder = remainder % 60000
        second = remainder // 1000
        ms = remainder % 1000
        return f"{hour}:{minute}:{second}:{ms}"


class LocalMessage:
    """
    Equivalent to C#'s LocalMessage, used to read local files (time, position, speed)
    and perform integration calculations, saving results as a series of PointCarrier objects.
    """
    def __init__(self, fileName):
        """
        C# logic:
         1) Read the entire file into a byte array, replace colons ':' with periods '.', and write back
         2) Read line by line, split into time, position, speed
         3) For the first line, directly store position into positionIntegration; 
            subsequent lines use trapezoidal integration
         4) time = hour*3600000 + minute*60000 + second*1000 + 17000 (consistent with C#)
        """
        # 1) Replace ':' with '.'
        if os.path.isfile(fileName):
            with open(fileName, 'rb') as f:
                dataArray = bytearray(f.read())

            for i in range(len(dataArray)):
                if dataArray[i] == ord(':'):
                    dataArray[i] = ord('.')

            with open(fileName, 'wb') as f:
                f.write(dataArray)

        self.points = []
        speedPre = 0.0
        positionSum = 0.0
        timePre = 0
        lineNumber = 0

        # 2) Read line by line, parse time, position, speed
        with open(fileName, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                # Expected format: "HH.MM.SS, position, speed"
                # Split by comma as in C#
                parts = line.split(',')
                if len(parts) < 3:
                    continue

                time_str = parts[0].strip()
                pos_str = parts[1].strip()
                speed_str = parts[2].strip()

                # Split time_str by '.' into [hour, minute, second]
                time_tokens = time_str.split('.')
                if len(time_tokens) < 3:
                    continue

                hour = int(time_tokens[0])
                minute = int(time_tokens[1])
                second = int(time_tokens[2])
                # Add 17000 milliseconds as in C# code
                time_val = hour * 3600000 + minute * 60000 + second * 1000 + 17000

                position = float(pos_str)
                speed = float(speed_str)

                # 3) Integration calculation
                if lineNumber == 0:
                    # First line: positionIntegration = current position
                    self.points.append(PointCarrier(time_val, position, speed, position))
                else:
                    # Trapezoidal integration
                    dt = (time_val - timePre)  # milliseconds
                    positionSum += ((speed + speedPre) * dt / 2000.0)  # ( /1000 * /2)
                    self.points.append(PointCarrier(time_val, position, speed, positionSum))

                speedPre = speed
                timePre = time_val
                lineNumber += 1


class MessageDecode:
    """
    Equivalent to C#'s MessageDecode, used to combine CAN data (CANMessage) with local data (LocalMessage).
    """
    def __init__(self, fileCan, fileLocal):
        """
        C# logic:
          canData = new CANMessage(file1)
          carrierData = new LocalMessage(file2)
          canData.messageList.Sort(...) sort by timeMs
        """
        self.canData = CANMessage(fileCan)
        self.carrierData = LocalMessage(fileLocal)
        # Sort CAN messages by time
        self.canData.messageList.sort(key=lambda m: m.timeMs)


if __name__ == '__main__':
    # Replace this path with the actual path to your .trc file
    trc_file = r"Your_Path\Filename.trc"
    
    # Parse the .trc file to generate the CAN message list and create a CSV file
    can_data = CANMessage(trc_file)
    
    # Output the number of parsed CAN messages
    print(f"Read {can_data.messageCount} CAN messages.")
    
    # Optional: Iterate and print information about the first few messages
    for msg in can_data.messageList[:5]:
        print(f"Message Number: {msg.messageNumber}, ID: {hex(msg.messageId)}, Time: {msg.timeString}, Payload: {' '.join(f'{b:02X}' for b in msg.payload)}, TimeMs: {msg.timeMs}, Time Offset: {msg.timeOffset}")