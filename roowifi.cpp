/****************************************************************************
 ** File
 ** · roowifi.cpp
 **
 ** Author
 ** · RooWifi Development Team (@roowifi)
 **
 ** Date
 ** · 13-06-2013
 **
 ** Description and comments:
 **
 ** ·This file contains source code of RooWifi class for Qt Creator
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
#include "roowifi.h"

////////////////////////////////////////////////////////////////////////////////////////////////
/// PUBLIC:
////////////////////////////////////////////////////////////////////////////////////////////////

//
//RooWifi::RooWifi(QObject *parent):QObject(parent) CONSTRUCTOR
//
RooWifi::RooWifi( QObject *parent ) : QObject( parent )
{
    InitGatewayTCPIP();
    InitSensors();
    InitLeds();
    InitAutoCapture();
    AutoCaptureLoopFinished = true;
    AutoCaptureEnabled = false;


    connect( AutoCaptureTimer, SIGNAL(timeout() ), this,
                        SLOT( AutoCaptureProcess() ) );

    connect( tcpSocket, SIGNAL( readyRead() ), this,
                        SLOT( RooWifiResponse() ) );

    connect( tcpSocket, SIGNAL( error(QAbstractSocket::SocketError) ), this,
                        SLOT( ConnectionError(QAbstractSocket::SocketError) ) );
}

//
// RooWifi::Connect
//
void RooWifi::Connect()
{
    if( IsConnected() )
    {
        tcpSocket->close();
        tcpSocket->abort();
    }

    tcpSocket->connectToHost( qstIP, ROOWIFI_GATEWAY_PORT );
    AutoCaptureLoopFinished = true;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Connecting to: %1 : %2" ).arg( qstIP ).arg( ROOWIFI_GATEWAY_PORT );
    #endif
}

//
// RooWifi::Disconnect
//
void RooWifi::Disconnect()
{
    tcpSocket->close();

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Disconnection" );
    #endif
}

//
// RooWifi::SetIP
//
// param NewIP - Ip of Roomba in Qstring format (for newies - easy to manage - I am XD)
//
void RooWifi::SetIP( QString NewIP )
{
    qstIP = NewIP;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: New IP: %1" ).arg( qstIP );
    #endif
}

// RooWifi::GetIP
//
// returns qstIP - IP of Roomba in Qstring format (for newies - easy to manage - I am XD)
//
QString RooWifi::GetIP()
{
    return qstIP;
}

// RooWifi::IsConnected
//
// returns tcpSocket->state() - true if connected false if not
//
bool RooWifi::IsConnected()
{
    if( tcpSocket->state() == QAbstractSocket::ConnectedState )
        return true;
    else
        return false;
}

//
// RooWifi::GetAutoCaptureTime
//
// param NewPeriod - Time between sensor packet 0 requests
//
void RooWifi::SetAutoCaptureTime( int NewPeriod )
{
    CaptureTime = NewPeriod;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: AutoCapture Period Updated: %1 ms" ).arg( CaptureTime );
    #endif
}

//
// RooWifi::GetAutoCaptureTime
//
// returns CaptureTime - Time between sensor packet 0 requests
//
int RooWifi::GetAutoCaptureTime()
{
    return CaptureTime;
}

//
// RooWifi::StartAutoCapture
//
void RooWifi::StartAutoCapture()
{

    AutoCaptureEnabled = true;
    AutoCaptureTimer->start( CaptureTime );
    AutoCaptureLoopFinished = true;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: AutoCapture Started" );
    #endif
}

//
// RooWifi::StopAutoCapture
//
void RooWifi::StopAutoCapture()
{
    AutoCaptureEnabled = false;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: AutoCapture Stopped" );
    #endif
}

//
// RooWifi::GetBatteryLevel
//
// returns BatteryLevel - Calculated Battery Level in float format
//
float RooWifi::GetBatteryLevel()
{
    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Returning battery calculated value float: %1" ).arg( BatteryLevel );
    #endif

    return BatteryLevel;
}

//
// RooWifi::AutoCaptureProcess
//
void RooWifi::AutoCaptureProcess()
{
    if ( AutoCaptureLoopFinished == true)
    {
        AutoCaptureLoopFinished = false;

        if (AutoCaptureEnabled == false)
        {
                AutoCaptureTimer->stop();
        }
        else
        {
            RequestAllSensors();
            #ifdef ROOWIFI_DEBUG
                qDebug() << QString( "ROOWIFI:: AutoCapture Sensor Request" );
            #endif
        }
    }

    if (AutoCaptureEnabled == false)
        AutoCaptureTimer->stop();
}

//
// RooWifi::UpdateSensors
//
// param NewValues - Response array received from Roomba after sensor packet 0 request
//
void RooWifi::UpdateSensors( unsigned char* NewValues )
{
    //Update Sensor one by one
    Sensors.BumpsWheeldrops = NewValues[BUMPWHEELDROPS_SENSOR];
    Sensors.Wall = NewValues[WALL_SENSOR];
    Sensors.CliffLeft = NewValues[CLIFFT_LEFT_SENSOR];
    Sensors.CliffFrontLeft = NewValues[CLIFFT_FRONT_LEFT_SENSOR];
    Sensors.CliffFrontRight = NewValues[CLIFFT_FRONT_RIGHT_SENSOR];
    Sensors.CliffRight = NewValues[CLIFFT_RIGHT_SENSOR];
    Sensors.VirtualWall = NewValues[VIRTUAL_WALL_SENSOR];
    Sensors.MotorOvercurrents = NewValues[MOTOR_OVERCURRENTS_SENSOR];
    Sensors.DirtDetectorLeft = NewValues[DIRT_DETECTOR_LEFT_SENSOR];
    Sensors.DirtDetectorRight = NewValues[DIRT_DETECTOR_RIGHT_SENSOR];
    Sensors.RemoteOpcode = NewValues[REMOTE_OPCODE_SENSOR];
    Sensors.Buttons = NewValues[BUTTONS_SENSOR];

    Sensors.Distance =  ( ( short ) NewValues[DISTANCE_MSB_SENSOR] ) << 8 |
                        ( short ) NewValues[DISTANCE_LSB_SENSOR];

    Sensors.Angle = ( ( short ) NewValues[ANGLE_MSB_SENSOR] ) << 8 |
                    ( short ) NewValues[ANGLE_LSB_SENSOR];

    Sensors.ChargingState = NewValues[CHARGING_STATE_SENSOR];

    Sensors.Voltage =   NewValues[VOLTAGE_MSB_SENSOR]  << 8 |
                        NewValues[VOLTAGE_LSB_SENSOR];

    Sensors.Current =   ( ( short ) NewValues[CURRENT_MSB_SENSOR] ) << 8 |
                        ( short ) NewValues[CURRENT_LSB_SENSOR];

    Sensors.Temperature = ( char ) NewValues[TEMPERATURE_SENSOR];

    Sensors.Charge =    NewValues[CHARGE_MSB_SENSOR]  << 8 |
                        NewValues[CHARGE_LSB_SENSOR];

    Sensors.Capacity =  NewValues[CAPACITY_MSB_SENSOR]  << 8 |
                        NewValues[CAPACITY_LSB_SENSOR];
    //Class values
    BatteryLevel =  ( float ) Sensors.Charge / ( float) Sensors.Capacity;

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Sensor Structure up to date. Emits AutoCaptureUpdated SIGNAL" );
    #endif

    AutoCaptureLoopFinished = true;

    emit AutoCaptureUpdated();
}

//
//RooWifiResponse()
//
void RooWifi::RooWifiResponse()
{
    static unsigned char SensorValues[SCI_NUMBER_OF_SENSORS];

    if ( AutoCaptureTimer->isActive() )   //Manage the reception if it is in AutoCapture Mode
    {
        if( tcpSocket->bytesAvailable() >= SCI_NUMBER_OF_SENSORS )  //Received all bytes of sensor packet?
        {
            tcpSocket->read( (char*)SensorValues, SCI_NUMBER_OF_SENSORS) ;
            UpdateSensors( SensorValues );

            #ifdef ROOWIFI_DEBUG
                qDebug() << QString( "ROOWIFI:: AutoCapture Sensor Packet 0 response received" );
            #endif
        }
    }
    else
    {
        //Emit a signal with data
        emit DataReady();
    }
}

//
//RooWifi::bytesAvailable
//
int RooWifi::bytesAvailable()
{
   return tcpSocket->bytesAvailable();
}

//
//Read()
//
int RooWifi::Read(char *Data, int DataLen)
{
    int DataRead;

    if ( tcpSocket->bytesAvailable() >= DataLen)
    {
        DataRead = DataLen;
        tcpSocket->read( Data, DataLen);
    }
    else
    {
        DataRead = tcpSocket->bytesAvailable();
        tcpSocket->read( Data, DataRead);
    }

    return DataRead;
}

//
//RequestAllSensors()
//
void RooWifi::RequestAllSensors()
{
    ExecuteCommandWithParameter(Commands::Command_Sensors,0);
}

/////////////////
//Operation Modes
/////////////////

//
// RooWifi::SafeMode
//
void RooWifi::SafeMode()
{
    ExecuteCommand( Commands::Command_Safe );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Safe Mode" );
    #endif
}

//
// RooWifi::FullMode
//
void RooWifi::FullMode()
{
    ExecuteCommand( Commands::Command_Full );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Full Mode" );
    #endif
}

/////////////////////////
//Button Actions/Commands
/////////////////////////

//
//void RooWifi::Clean
//
void RooWifi::Clean()
{
    ExecuteCommand( Commands::Command_Clean );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Clean button action" );
    #endif
}

//
//void RooWifi::Spot
//
void RooWifi::Spot()
{
    ExecuteCommand( Commands::Command_Spot );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Spot button action" );
    #endif
}

//
// RooWifi::GoDock
//
void RooWifi::GoDock()
{
    ExecuteCommand( Commands::Command_Dock );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Dock button action" );
    #endif
}

//
// RooWifi::Drive
//
void RooWifi::Drive( int Velocity, int Radius )
{
    int Parameters[4];

    Parameters[0] = ( Velocity >> 8 ) & 0x00FF;
    Parameters[1] = Velocity & 0x00FF;

    Parameters[2] = ( Radius >> 8 ) & 0x00FF;
    Parameters[3] = Radius & 0x00FF;

    ExecuteCommandWithParameters ( Commands::Command_Drive, Parameters, 4 );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Call to Drive: Vel: %1 Rad: %2").arg(QString::number(Velocity)).arg(QString::number(Radius));
    #endif
}

//
// RooWifi::ExecuteCommandWithParameters
//
// param CommandToSend - Command that you want to transmit to Roomba
// param Parameters - Int "array" with the diferent value you want to transmit with the command
// param NumParameters - Number of parameters to transmit (valid length of Parameters array)
//
void RooWifi::ExecuteCommandWithParameters ( int CommandToSend, int* Parameters, int NumParameters )
{
    int ParameterCounter;

    if( tcpSocket->state() == QAbstractSocket::ConnectedState )
    {
        QDataStream out( tcpSocket );

        out << ( unsigned char ) CommandToSend;

        for ( ParameterCounter = 0; ParameterCounter < NumParameters; ParameterCounter++ )
            out << ( unsigned char ) Parameters[ ParameterCounter ];

    }
}

//
// RooWifi::ExecuteCommand sends CommandToSend to Roomba
//
void RooWifi::ExecuteCommand( int CommandToSend )
{
    if( tcpSocket->state() == QAbstractSocket::ConnectedState )
    {
        QDataStream out( tcpSocket );

        out << ( unsigned char ) CommandToSend;
    }
}

//
// RooWifi::ExecuteCommandWithParameter
//
void RooWifi::ExecuteCommandWithParameter( int CommandToSend, int Data )
{
    if( tcpSocket->state() == QAbstractSocket::ConnectedState )
    {
        QDataStream out( tcpSocket );

        out << ( unsigned char ) CommandToSend;
        out << ( unsigned char ) Data;
    }
}
//
// RooWifi::StoreSong
//
void RooWifi::StoreSong( int SongNumber, int SongLength, int *Song,int *SongDuration)
{
    int Parameters[34];
    int ParameterCounter;

    if ( SongLength > 16 )
        SongLength = 16;

    for ( ParameterCounter = 0; ParameterCounter < ( SongLength ); ParameterCounter ++ )
    {
        Parameters[(ParameterCounter*2)+2] = Song[ParameterCounter];
        Parameters[(ParameterCounter*2)+3] = SongDuration[ParameterCounter];
    }
    Parameters[0] = SongNumber;
    Parameters[1] = SongLength;


    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Store Song" );
    #endif

    ExecuteCommandWithParameters ( Commands::Command_Song, Parameters, (SongLength*2)+2 );
}

//
// RooWifi::PlaySong
//
void RooWifi::PlaySong( int SongNumber )
{
    ExecuteCommandWithParameter ( Commands::Command_Play, SongNumber );
}

///////////////
//Motor Control
///////////////

//MAIN BRUSH

//
// RooWifi::MainBrush_On
//
void RooWifi::MainBrush_On()
{
    Motors |= MAIN_BRUSH_ON;
    UpdateMotors();
}

//
// RooWifi::MainBrush_Off
//
void RooWifi::MainBrush_Off()
{
    Motors &= MAIN_BRUSH_OFF;
    UpdateMotors();
}

//VACUUM

//
// RooWifi::Vacuum_On
//
void RooWifi::Vacuum_On()
{
    Motors |= VACUUM_ON;
    UpdateMotors();
}
//
// RooWifi::Vacuum_Off
//
void RooWifi::Vacuum_Off()
{
    Motors &= VACUUM_OFF;
    UpdateMotors();
}

//SIDE BRUSH

//
// RooWifi::SideBrush_On
//
void RooWifi::SideBrush_On()
{
    Motors |= SIDE_BRUSH_ON;
    UpdateMotors();
}

//
// RooWifi::SideBrush_Off
//
void RooWifi::SideBrush_Off()
{
    Motors &= SIDE_BRUSH_OFF;
    UpdateMotors();
}

//ALL MOTORS AT SAME TIME

//
// RooWifi::AllCleaningMotors_On
//
void RooWifi::AllCleaningMotors_On()
{
    Motors |= ALL_CLEANING_MOTORS_ON;
    UpdateMotors();
}

//
// RooWifi::AllCleaningMotors_Off
//
void RooWifi::AllCleaningMotors_Off()
{
    Motors &= ALL_CLEANING_MOTORS_OFF;
    UpdateMotors();
}

///////////////
//LED   Control
///////////////

//STATUS

//
// RooWifi::Led_Status_Green_On()
//
void RooWifi::Led_Status_Green_On()
{
    //Leds[0] &= LED_STATUS_OFF;
    Leds[0] |= LED_STATUS_GREEN;
    UpdateLeds();
}

//
// RooWifi::Led_Status_Red_On()
//
void RooWifi::Led_Status_Red_On()
{
    //Leds[0] &= LED_STATUS_OFF;
    Leds[0] |= LED_STATUS_RED;
    UpdateLeds();
}

//
// RooWifi::Led_Status_Ambar_On()
//
void RooWifi::Led_Status_Ambar_On()
{
    //Leds[0] &= LED_STATUS_OFF;
    Leds[0] |= LED_STATUS_AMBAR;
    UpdateLeds();
}

//
// RooWifi::Led_Status_Off()
//
void RooWifi::Led_Status_Off()
{
    Leds[0] &= LED_STATUS_OFF;
    UpdateLeds();
}

//SPOT

//
// RooWifi::Led_Spot_On
//
void RooWifi::Led_Spot_On()
{
    Leds[0] |= LED_SPOT_ON;
    UpdateLeds();
}

//
// RooWifi::Led_Spot_Off
//
void RooWifi::Led_Spot_Off()
{
    Leds[0] &= LED_SPOT_OFF;
    UpdateLeds();
}

//CLEAN

//
// RooWifi::Led_Clean_On
//
void RooWifi::Led_Clean_On()
{
    Leds[0] |= LED_CLEAN_ON;
    UpdateLeds();
}

//
// RooWifi::Led_Clean_Off
//
void RooWifi::Led_Clean_Off()
{
    Leds[0] &= LED_CLEAN_OFF;
    UpdateLeds();
}

//MAX

//
// RooWifi::Led_Max_On
//
void RooWifi::Led_Max_On()
{
    Leds[0] |= LED_MAX_ON;
    UpdateLeds();
}

//
// RooWifi::Led_Max_Off
//
void RooWifi::Led_Max_Off()
{
    Leds[0] &= LED_MAX_OFF;
    UpdateLeds();
}

//DIRT DETECT

//
// RooWifi::Led_Dirt_On
//
void RooWifi::Led_Dirt_On()
{
    Leds[0] |= LED_DIRT_ON;
    UpdateLeds();
}

//
// RooWifi::Led_Dirt_Off
//
void RooWifi::Led_Dirt_Off()
{
    Leds[0] &= LED_DIRT_OFF;
    UpdateLeds();
}

//POWER LED

//
// RooWifi::Led_Power_Color
//
void RooWifi::Led_Power_Color( int NewColor )
{
    Leds[1] = NewColor;
    UpdateLeds();
}

//
// RooWifi::Led_Power_Intensity
//
void RooWifi::Led_Power_Intensity( int NewIntensity )
{
    Leds[2] = NewIntensity;
    UpdateLeds();
}

//
// RooWifi::Led_Power_Off
//
void RooWifi::Led_Power_Off()
{
    Leds[1] = 0;
    Leds[2] = 0;
    UpdateLeds();
}


////////////////////////////////////////////////////////////////////////////////////////////////
/// PRIVATE:
////////////////////////////////////////////////////////////////////////////////////////////////

//
// RooWifi::UpdateLeds
//
void RooWifi::UpdateLeds()
{
    ExecuteCommandWithParameters( Commands::Command_Leds ,Leds ,LEDS_NUM_PARAMETERS );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Update of Leds with hex 0x%1 0x%2 0x%3" )
                .arg( Leds[0], 2, 16, QLatin1Char( '0' ) )
                .arg( Leds[1], 2, 16, QLatin1Char( '0' ) )
                .arg( Leds[2], 2, 16, QLatin1Char( '0' ) );
    #endif
}

//
// RooWifi::UpdateMotors
//
void RooWifi::UpdateMotors()
{
    ExecuteCommandWithParameter( Commands::Command_Motors,Motors );

    #ifdef ROOWIFI_DEBUG
        qDebug() << QString( "ROOWIFI:: Update of Motors with hex 0x%1" )
                .arg( Motors, 2, 16, QLatin1Char( '0' ) );
    #endif
}

//
// RooWifi::ConnectionError
//
void RooWifi::ConnectionError(QAbstractSocket::SocketError socketError)
{
    emit ErrorConnection(socketError);
}

////////////////
//INIT FUNCTIONS
////////////////

//
// RooWifi::InitGatewayTCPIP
//
void RooWifi::InitGatewayTCPIP()
{
    qstIP = DEFAULT_ROOWIFI_IP;
    TCPPort = ROOWIFI_GATEWAY_PORT;
    tcpSocket = new QTcpSocket ( this );
}

//
// RooWifi::InitSensors
//
void RooWifi::InitSensors()
{
    Sensors.BumpsWheeldrops = ZERO_BY_DEFAULT;
    Sensors.Wall = ZERO_BY_DEFAULT;
    Sensors.CliffLeft = ZERO_BY_DEFAULT;
    Sensors.CliffFrontLeft = ZERO_BY_DEFAULT;
    Sensors.CliffFrontRight = ZERO_BY_DEFAULT;
    Sensors.CliffRight = ZERO_BY_DEFAULT;
    Sensors.VirtualWall = ZERO_BY_DEFAULT;
    Sensors.MotorOvercurrents = ZERO_BY_DEFAULT;
    Sensors.DirtDetectorLeft = ZERO_BY_DEFAULT;
    Sensors.DirtDetectorRight = ZERO_BY_DEFAULT;
    Sensors.RemoteOpcode = ZERO_BY_DEFAULT;
    Sensors.Buttons = ZERO_BY_DEFAULT;
    Sensors.Distance = ZERO_BY_DEFAULT;
    Sensors.Angle = ZERO_BY_DEFAULT;
    Sensors.ChargingState = ZERO_BY_DEFAULT;
    Sensors.Voltage = ZERO_BY_DEFAULT;
    Sensors.Current = ZERO_BY_DEFAULT;
    Sensors.Temperature = ZERO_BY_DEFAULT;
    Sensors.Charge = ZERO_BY_DEFAULT;
    Sensors.Capacity = ZERO_BY_DEFAULT;
}

//
// RooWifi::InitLeds
//
void RooWifi::InitLeds()
{
    int Counter;

    for( Counter = 0; Counter < LEDS_NUM_PARAMETERS; Counter++ )
        Leds[Counter] = ZERO_BY_DEFAULT;
}

//
// RooWifi::InitMotors
//
void RooWifi::InitMotors()
{
    Motors = ZERO_BY_DEFAULT;
}

//
// RooWifi::InitAutoCapture
//
void RooWifi::InitAutoCapture()
{
    BatteryLevel = ZERO_BY_DEFAULT;
    CaptureTime = AUTO_CAPTURE_DEFAULT_PERIOD;
    AutoCaptureTimer = new QTimer( this );
}
