/****************************************************************************
 ** File
 ** · roowifi.h
 **
 ** Author
 ** · RooWifi Development Team (@roowifi)
 **
 ** Date
 ** · 13-06-2013
 **
 ** Description and comments:
 **
 ** ·This file contains headers and class definition of RooWifi class for Qt Creator
 **
 ** ·Use ExecuteCommand /Signals and Slots for gateway mode of RooWifi Device
 **  to talk with your Roomba directly
 **
 ** ·Use AutoCapture to read sensor values automatically with the
 **  specified time.
 **
 ** ·This version uses specific classes of Qt Environment
 **  ·QTcpSocket
 **  ·QTimer
 **
 ** ·Read the SCI Specification document from iRobot before use this
 **  source code.
 **
 ** Copyright (C) 2013 RooWifi.com
 ** Contact: http://www.roowifi.com/contact
 **
 ** This file is part of the RooWifi class of Qt API for developers.
 **
 ** Information summary of versions :
 **
 ** Version     Author          Date
 ** 1.0b        @roowifi        13 / 06 / 2013
 ****************************************************************************/
#ifndef ROOWIFI_H
#define ROOWIFI_H

#include "roomba.h"

#include <QString>
#include <QtNetwork>
#include <QTcpSocket>
#include <QTimer>
#include <QObject>

//Uncomment this to see debug output
#define ROOWIFI_DEBUG

//Default Period of AutoCapture Request
#define AUTO_CAPTURE_DEFAULT_PERIOD     500 //(ms)

//Default values of sensor structure (AutoCapture)
#define ZERO_BY_DEFAULT                 0

//TCP/IP Port in Gateway Mode
#define ROOWIFI_GATEWAY_PORT            9001

//IP ADDRESS by default
#define DEFAULT_ROOWIFI_IP              "10.0.0.1" //In Ad-Hoc Mode

class RooWifi : public QObject
{
    Q_OBJECT

public:

    //structure with all Roomba's sensors
    //Structure with all information related with sensors
    //in Auto Capture Mode
    //
    typedef struct
    {
        unsigned char BumpsWheeldrops;
        unsigned char Wall;
        unsigned char CliffLeft;
        unsigned char CliffFrontLeft;
        unsigned char CliffFrontRight;
        unsigned char CliffRight;
        unsigned char VirtualWall;
        unsigned char MotorOvercurrents;
        unsigned char DirtDetectorLeft;
        unsigned char DirtDetectorRight;
        unsigned char RemoteOpcode;
        unsigned char Buttons;
        short Distance;
        short Angle;
        unsigned char ChargingState;
        unsigned short Voltage;
        short Current;
        char Temperature;
        unsigned short Charge;
        unsigned short Capacity;
    } RoombaSensors;

    //PUBLIC SUB-CLASSES to easy access for the developer
    //Examples:
    // if ( MyRooWifi->Sensors.ChargingState == RooWifi::ChargingStates::ChargingState_NoCharging )
    // or
    // MyRooWifi->ExecuteCommand(RooWifi::Commands.Command_Song);

    //Possible Charging States for Sensors.ChargingState
    //Useful for comparations:
    // if ( MyRooWifi->Sensors.ChargingState == RooWifi::ChargingStates::ChargingState_NoCharging )
    class ChargingStates
    {
        public:
            static const int ChargingState_NoCharging    =   CHARGING_STATE_NO_CHARGING;
            static const int ChargingState_Recovery      =   CHARGING_STATE_CHARGING_RECOVERY;
            static const int ChargingState_Charging      =   CHARGING_STATE_CHARGING;
            static const int ChargingState_Trickle       =   CHARGING_STATE_TRICKLE_CHAGING;
            static const int ChargingState_Waiting       =   CHARGING_STATE_WAITING;
            static const int ChargingState_Error         =   CHARGING_STATE_CHARGING_ERROR;
    };

    //Possible Commands to tx to Roomba:
    //Useful with this functions:
    // ExecuteCommand
    // ExecuteCommandWithParameter
    // ExecuteCommandWithParameters

    class Commands
    {
        public:
            static const int Command_Safe       =   COMMAND_SAFE;
            static const int Command_Full       =   COMMAND_FULL;
            static const int Command_Power      =   COMMAND_POWER;
            static const int Command_Spot       =   COMMAND_SPOT;
            static const int Command_Clean      =   COMMAND_CLEAN;
            static const int Command_Max        =   COMMAND_MAX;
            static const int Command_Drive      =   COMMAND_DRIVE;
            static const int Command_Motors     =   COMMAND_MOTORS;
            static const int Command_Leds       =   COMMAND_LEDS;
            static const int Command_Song       =   COMMAND_SONG;
            static const int Command_Play       =   COMMAND_PLAY;
            static const int Command_Sensors    =   COMMAND_SENSORS;
            static const int Command_Dock       =   COMMAND_DOCK;
    };

    //Notes duration
    //Useful with this function:
    // StoreSong

    class NotesDuration
    {
        public:
            static const int Crotchet           =   NOTE_DURATION_CROTCHET;
            static const int Quaver             =   NOTE_DURATION_QUAVER;
            static const int SemiQuaver         =   NOTE_DURATION_SEMIQUAVER;
    };

    ////////////////
    //PUBLIC HEADERS
    ////////////////

    //Constructor
    explicit RooWifi( QObject *parent );

    //TCP Socket Connection / Disconnection
    // Connect - Try to connect with RooWifi ( see SetIP GetIP functions )
    void Connect();
     // Disconnect - Close TCP IP Socket connection to RooWifi
    void Disconnect();
    // IsConnected - returns true if connected or false if not
    bool IsConnected();

    //IP or NBNSBios Name of RooWifi Board
    // SetIP
    //  QString - could be an IP address or a name if you are using Windows OS
    // GetIP
    void SetIP( QString NewIP );
    QString GetIP();

    //Sensors / Roomba Telemetry
    //
    // RequestAllSensors - Request Sensor Packet 0 - All Sensors - 26 values
    void RequestAllSensors();

    //Working modes of Roomba
    //To put in passive mode do a clean, spot or dock action
    //
    // SafeMode - Puts Roomba in Safe Mode
    void SafeMode();
    // FullMode - Puts Roomba in Full Mode
    void FullMode();

    //Actions / Emulation of buttons
    //
    // Clean - Executes a Clean command - Emulation of Clean button
    void Clean();
    // Spot - Executes a Spot command - Emulation of Spot button
    void Spot();
    // GoDock - Executes a Force-Seeking Dock command - Emulation of Dock button
    void GoDock();

    //Moving Roomba as you would - Play with Roomba like with a RC Car :)
    //
    // Drive
    // ·Velocity - Velocity of Roomba - Max forward = 500mm/s. Max backward = -500mm/s
    // ·Radius - Radius of Roomba's movement - Max turn = 1 or -1. Margins= -2000mm , 2000mm
    // To know better the behaviour please read SCI Specs
    void Drive( int Velocity, int Radius );

    //MainBrush Motor
    //
    // MainBrush_On - Turns on Roomba's Main Brush
    void MainBrush_On();
    // MainBrush_Off - Turns off Roomba's Main Brush
    void MainBrush_Off();

    //Vacuum Motor
    //
    // Vacuum_On - Turns on Roomba's Vacuum
    void Vacuum_On();
    // Vacuum_Off - Turns off Roomba's Vacuum
    void Vacuum_Off();

    //SideBrush Motor
    //
    // SideBrush_On - Turns on Roomba's Side Brush
    void SideBrush_On();
    // SideBrush_Off - Turns off Roomba's Side Brush
    void SideBrush_Off();

    //All Motors at same time
    //
    // AllCleaningMotors_On - Turns on all Roomba Cleanning buttons
    void AllCleaningMotors_On();
    // AllCleaningMotors_Off - Turns off all Roomba Cleanning buttons
    void AllCleaningMotors_Off();

    //Spot Led
    //
    // Led_Spot_On - Turns on Spot Led
    void Led_Spot_On();
    // Led_Spot_Off - Turns off Spot Led
    void Led_Spot_Off();

    //Max Led
    //
    // Led_Max_On - Turns on Max Led
    void Led_Max_On();
    // Led_Max_Off - Turns off Max Led
    void Led_Max_Off();

    //Clean Led
    //
    // Led_Clean_On - Turns on Clean Led
    void Led_Clean_On();
    // Led_Clean_Off - Turns off Clean Led
    void Led_Clean_Off();

    //Dirt Led
    //
    // Led_Dirt_On - Turns on Dirt Led
    void Led_Dirt_On();
    // Led_Dirt_Off - Turns off Dirt Led
    void Led_Dirt_Off();

    //Status Led - Deprecated if using >= Roomba 500 series. In this case Turns On all leds
    //
    // Led_Status_Green_On
    void Led_Status_Green_On();
    // Led_Status_Red_On
    void Led_Status_Red_On();
    // Led_Status_Ambar_On
    void Led_Status_Ambar_On();
    // Led_Status_Off
    void Led_Status_Off();

    //Power Led
    //
    // Led_Power_Color - Sets new color for Power Led (battery led in Roomba 700)
    // ·NewColor - New Color of Power Led ( 0 - 255 )
    void Led_Power_Color( int NewColor );
    // Led_Power_Intensity - Sets new intensity of Power Led (battery led in Roomba 700)
    // ·NewIntensity - New Intensity of Power Led ( 0 - 255 )
    void Led_Power_Intensity( int NewIntensity );
    // Led_Power_Off - Turns off Power Led (battery led in Roomba 700)
    void Led_Power_Off();

    //Playing Music in Roomba
    //
    // StoreSong
    // ·SongNumber - Number of song that you would to store in Roomba ( 1 - 16 )
    // ·SongLength - Number of notes of new song ( 1 - 16 )
    // ·Song - Array with all notes to store. Max 16 notes
    // ·SongDuration - Aray with all notes duration to store SongDuration[0] corresponds with Song[0]
    void StoreSong( int SongNumber, int SongLength, int *Song, int *SongDuration);
    // PlaySong
    // ·SongNumber - Specifies the number of song you would to play ( 1 - 16 )
    void PlaySong( int SongNumber);

    //////////////////////
    //Direct Communication
    //////////////////////
    ///With these functions you could send and receive
    ///data directly through RooWifi using or not
    ///Auto Capture mode of this "API"
    //Executes Commands / Send Instructions directly to Roomba
    //
    // ExecuteCommand
    // ·CommandToSend - Command you want to transmit to Roomba
    void ExecuteCommand( int CommandToSend );
    // ExecuteCommandWithParameter
    // ·CommandToSend - Command you want to transmit to Roomba
    // ·Data - Value
    void ExecuteCommandWithParameter( int CommandToSend, int Data );
    // ExecuteCommandWithParameters
    // ·CommandToSend - Command you want to transmit to Roomba
    // ·Parameters - Array with parameters
    // ·NumParameters - Number of parameters apart of Command
    void ExecuteCommandWithParameters ( int CommandToSend, int* Parameters, int NumParameters );
    // bytesAvailable -
    int bytesAvailable();
    // Read
    // ·Data - Array where you want to sore received data
    // ·DataLen - Len of received data to read
    int Read(char *Data, int DataLen);

    //Auto Capture Mode
    // Once connected to RooWifi you could use this
    // mode to refresh sensor data with the specified
    // period of time in ms with SetAutoCaptureTime()
    // automatically

    //Enable / Disable Auto Capture Mode
    //
    // StartAutoCapture
    void StartAutoCapture();
    // StopAutoCapture
    void StopAutoCapture();

    //Configuration of Auto Capture Mode
    //
    // SetAutoCaptureTime - Sets new period time between sensor packet 0 requests
    void SetAutoCaptureTime( int NewPeriod );
    // GetAutoCaptureTime - Returns perdiod time between sensor packet 0 requests
    int GetAutoCaptureTime();

    //Calculated values
    //
    // GetBatteryLevel - Returns a float with calculated value of battery level
    float GetBatteryLevel();

    //PUBLIC PROPERTY (EASY ACCESS TO SENSORS)
    RoombaSensors Sensors;

signals:

    //SIGNALS (AutoCapture)
    //
    // Occurs when Sensors variable is up to date
    void AutoCaptureUpdated();

    //SIGNALS (ERROR)
    //
    // Occurs when a connection to specified IP couldn't be done
    void ErrorConnection(QAbstractSocket::SocketError socketError);

    //SIGNALS (Data available in reception from Rooma)
    //
    // Occurs when AutoCapture Mode is disabled and receive data from Roomba
    void DataReady();

private slots:
    //PRIVATE SLOTS
    void RooWifiResponse();
    void AutoCaptureProcess();
    void ConnectionError(QAbstractSocket::SocketError socketError);

private:
    //PRIVATE HEADERS
    void InitLeds();
    void InitMotors();
    void InitGatewayTCPIP();
    void InitSensors();
    void InitAutoCapture();
    void UpdateSensors( unsigned char* NewValues );
    void UpdateLeds();
    void UpdateMotors();
    //PRVATE PROPERTIES
    QTcpSocket *tcpSocket;
    QString qstIP;
    int TCPPort;
    QTimer *AutoCaptureTimer;
    int CaptureTime;
    float BatteryLevel;
    int Motors;
    int Leds[LEDS_NUM_PARAMETERS];
    bool AutoCaptureLoopFinished;
    bool AutoCaptureEnabled;
};

#endif // ROOWIFI_H
