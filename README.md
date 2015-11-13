# RooWifi: WiFi adapter for Roomba API
The latest version of RooWifi API is based on the gateway mode and it’s programmed using Qt Creator cross-platform IDE, which allows developers to use Windows, MAC, GNU/Linux,… Next revisions will have support and the required modifications to be able to develop with Visual Studio, gcc,g++ and Python

In roowifi.h comment #define ROOWIFI_DEBUG to disable debug output messages, this define is enabled by default.

If AutoCapture mode is used, when the RooWifi::Sensors is updated, the class generates an AutoCaptureUpdated() SIGNAL. If not the generated SIGNAL is DataReady() when a data byte is received from the RooWifi (eg, manual sensor reading using ExecuteCommand functions).

## TCP Socket Connection / Disconnection functions
### void SetIP( QString NewIP )
Sets the IP address of RooWifi. You need to call this function to select the IP address of your device. By default the IP address of RooWifi is 10.0.0.1 (the default IP address of RooWifi in Ad-Hoc mode).
In Windows Systems you could use the NBNSBios name to connect to your RooWifi.
Examples:
Windows: SetIP ( “roomba”);
Others: SetIP (“10.0.0.1”);
### QString GetIP()
Returns a QString with the NBNSBios name of your RooWifi or you RooWifi’s IP address.
Example:
QString AddressOfMyRooWifi = GetIP();
### void Connect()
Try to connect to RooWifi using a standard TCP/IP socket getting the IP address set in SetIP(QString NewIP) function. If it causes an exception or any error occurs while connecting process it will generate a trigger signal.
### void Disconnect()
Disconnects from RooWifi and close the TCP/IP socket connection.
### bool IsConnected()
Returns true if the connection is established or false if not.
## Sensors / Roomba Telemetry
### void RequestAllSensors()
Requests all sensor values.
## Working modes of Roomba
### void SafeMode()
Puts Roomba in Safe Mode. You can drive your Roomba but when it detects an obstacle or something that could damage the robot it will stop automatically and will set itself to Passive Mode (default).
### void FullMode()
Puts Roomba in Full Mode. Enables unrestricted control of Roomba and turns off the safety features. Be careful using this mode while driving or controlling Roomba.
## Actions / Emulation of buttons
### void Clean()
Starts a normal cleaning cycle, the same as a normal “clean” button press. This command puts in Safe Mode.
### void Spot()
Starts a spot cleaning cycle, the same as a normal “spot” button press. This command puts in Safe Mode.
### void GoDock()
Turns on force-seeking-dock mode, which causes the robot to immediately attempt to dock during its cleaning cycle if it encounters the docking beams from the Home Base. (Note, however, that if the robot was not active in a clean, spot or max cycle it will not attempt to execute the docking.) Normally the robot attempts to dock only if the cleaning cycle has completed or the battery is nearing depletion. This command can be sent anytime, but the mode will be cancelled if the robot turns off, begins charging, or is commanded into safe or full modes.
## Moving Roomba as you would
Play with Roomba like a RC Car :)
### void Drive( int Velocity, int Radius )
Parameters:
Velocity: specify the average velocity of the drive wheels in millimeters per second (mm/s). Max forward = 500mm/s. Max backward = -500mm/s
Radius: The radius, in millimeters, at which Roomba should turn. The longer radii make Roomba drive straighter; shorter radii make it turn more. Max turn = 1 or -1. Margins= -2000mm , 2000mm
A Drive command with a positive velocity and a positive radius will make Roomba drive forward while turning toward the left. A negative radius will make it turn toward the right. Special cases for the radius make Roomba turn in place or drive straight, as specified below. The robot must be in safe or full mode to accept this command.
## Motor Control
### void MainBrush_On()
Turns on Roomba’s Main Brush
### void MainBrush_Off()
Turns off Roomba’s Main Brush
### void Vacuum_On()
Turns on Roomba’s Vacuum
### void Vacuum_Off()
Turns off Roomba’s Vacuum
### void SideBrush_On()
Turns on Roomba’s Side Brush
### void SideBrush_Off()
Turns off Roomba’s Side Brush
### void AllCleaningMotors_On()
Turns on all Roomba Cleanning buttons
### void AllCleaningMotors_Off()
Turns off all Roomba Cleaning buttons
## Led Control
### void Led_Spot_On()
Turns on Spot Led
### void Led_Spot_Off()
Turns off Spot Led
### void Led_Max_On()
Turns on Max Led
### void Led_Max_Off()
Turns off Max Led
### void Led_Clean_On()
Turns on Clean Led
### void Led_Clean_Off()
Turns off Clean Led
### void Led_Dirt_On()
Turns on Dirt Led
### void Led_Dirt_Off()
Turns off Dirt Led
Status Led – Deprecated if using >= Roomba 500 series.
### void Led_Status_Green_On()
### void Led_Status_Red_On()
### void Led_Status_Ambar_On()
### void Led_Status_Off()
### void Led_Power_Color( int NewColor )
Sets new color for Power Led (battery led in Roomba 700)
Parameter:
NewColor: New Color of Power Led ( 0 – 255 )
### void Led_Power_Intensity( int NewIntensity )
Sets new intensity of Power Led (battery led in Roomba 700)
Parameter:
NewIntensity: New Intensity of Power Led ( 0 – 255 )
### void Led_Power_Off()
Turns off Power Led (battery led in Roomba 700)
## Direct Communication
With these functions you could send and receive sensor data directly through RooWifi with or without use of Auto Capture mode of this “API”
### void ExecuteCommand( int CommandToSend )
Parameter:
CommandToSend: Command you want to transmit to Roomba
### void ExecuteCommandWithParameter( int CommandToSend, int Data )
Parameters:
CommandToSend: Command you want to transmit to Roomba.
Data: Value to transmit alter the command header.
### void ExecuteCommandWithParameters ( int CommandToSend, int* Parameters, int NumParameters )
Parameters:
CommandToSend: Command you want to transmit to Roomba.
Parameters: Array with parameters.
NumParameters: Number of parameters apart of Command. Lenght of Parameters array.
### bytesAvailable()
Returns the number of bytes received from Roomba and available in the reception buffer.
### int Read(char *Data, int DataLen)
Parameters:
Data: Array where you want to sore received data
DataLen: Length of received data to read
## Auto Capture Mode
Once connected to RooWifi you could use this mode to refresh sensor data with the specified period of time in ms with SetAutoCaptureTime() automatically refreshing RoombaSensors structure.
### void StartAutoCapture()
Enables/starts auto capture mode
### void StopAutoCapture()
Disables/stops auto capture mode
### void SetAutoCaptureTime( int NewPeriod )
Sets new period time between sensor packet 0 requests
Parameters:
NewPeriod: Time in ms between requests
### int GetAutoCaptureTime()
Returns perdiod time between sensor packet 0 requests
### float GetBatteryLevel()
Returns a float with calculated value of battery level
