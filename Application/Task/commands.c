/**
 * @file    commands.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "digital.h"
#include "serial.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"
#include "cli/FreeRTOS_CLI.h"
#include "message_buffer.h"
#include "Task/control.h"
#include "xprintf/xprintf.h"

#include <string.h>
#include <stdlib.h>

//==============================================================================
// Private definitions
//==============================================================================

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

//==============================================================================
// Extern variables
//==============================================================================

//==============================================================================
// Private function prototypes
//==============================================================================

static BaseType_t CLI_ResetCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString );
static BaseType_t CLI_TraceCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString );
static BaseType_t CLI_PidCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString );
static BaseType_t CLI_InfoCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString );
static BaseType_t CLI_DirectionCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString );

//==============================================================================
// Private variables
//==============================================================================

static const CLI_Command_Definition_t CliCmd_Reset =
{
    "reset", /* The command string to type. */
    "reset:\r\n Reinicia placa\r\n",
    CLI_ResetCommand, /* The function to run. */
    0 /* none number. */
};

static const CLI_Command_Definition_t CliCmd_Trace =
{
    "trace", /* The command string to type. */
    "trace:\r\n enable trace of car\r\n",
    CLI_TraceCommand, /* The function to run. */
    1 /* number of commands. */
};

static const CLI_Command_Definition_t CliCmd_PID =
{
    "pid", /* The command string to type. */
    "pid:\r\n set parameters of PID Control\r\n",
    CLI_PidCommand, /* The function to run. */
    2 /* number of commands. */
};

static const CLI_Command_Definition_t CliCmd_GetInfo =
{
    "info", /* The command string to type. */
    "info:\r\n Show informations of system\r\n",
    CLI_InfoCommand, /* The function to run. */
    0 /* number of commands. */
};

static const CLI_Command_Definition_t CliCmd_Direction =
{
    "direction", /* The command string to type. */
    "direction:\r\n Set direction of car\r\n",
    CLI_DirectionCommand, /* The function to run. */
    1 /* number of commands. */
};

//==============================================================================
// Private functions
//==============================================================================

static BaseType_t CLI_ResetCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString )
{
  /* Remove compile time warnings about unused parameters */
  ( void ) CommandString;
  ( void ) OutputBufferLen;

  NVIC_SystemReset();

  *OutputBuffer = 0x00;
  CommandString = 0x00;

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static BaseType_t CLI_TraceCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString )
{
  char *parameter;
  eCarDebugType_e traceType;
  BaseType_t parameterStringLength;

  parameter = (char *)FreeRTOS_CLIGetParameter( CommandString, 1, &parameterStringLength );
  parameter[ parameterStringLength ] = 0x00;

  if( strcmp( parameter, "digital" ) == 0 )
  {
    traceType = eCAR_TRACE_DIGITAL;
  }
  else if( strcmp( parameter, "analog" ) == 0 )
  {
    traceType = eCAR_TRACE_ANALOG;
  }
  else if( strcmp( parameter, "disable" ) == 0 )
  {
    traceType = eCAR_TRACE_DISABLE;
  }
  else
  {
    snprintf_( OutputBuffer, OutputBufferLen, "Invalid parameter\r\n" );
    return pdFALSE;
  }

  Control_EnableTrace( traceType );
  snprintf_(OutputBuffer, OutputBufferLen, "Trace %s\r\n" , parameter);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static BaseType_t CLI_PidCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString )
{
  float value;
  char *parameter1;
  char *parameter2;
  BaseType_t parameter1StringLength;
  BaseType_t parameter2StringLength;
  PidTypeParam_e typePidParam;

  /* Remove compile time warnings about unused parameters */
  ( void ) CommandString;
  ( void ) OutputBufferLen;

  parameter1 = (char *)FreeRTOS_CLIGetParameter( CommandString, 1, &parameter1StringLength );
  parameter2 = (char *)FreeRTOS_CLIGetParameter( CommandString, 2, &parameter2StringLength );

  parameter1[ parameter1StringLength ] = 0x00;
  parameter2[ parameter2StringLength ] = 0x00;

  if( !isDigitString( parameter2, parameter2StringLength ) )
  {
    snprintf_(OutputBuffer, OutputBufferLen, "Parameter 2 invalid\r\n");
    return pdFALSE;
  }

  value = atof(parameter2);

  if( strcmp( parameter1, "kp" ) == 0 )
  {
    typePidParam = ePID_KP;
  }
  else if( strcmp( parameter1, "ki" ) == 0 )
  {
    typePidParam = ePID_KI;
  }
  else if( strcmp( parameter1, "kd" ) == 0 )
  {
    typePidParam = ePID_KD;
  }
  else if( strcmp( parameter1, "setpoint" ) == 0 )
  {
    typePidParam = ePID_SetPoint;
  }
  else
  {
    snprintf_(OutputBuffer, OutputBufferLen, "Parameter 1 invalid\r\n");
    return pdFALSE;
  }

  Control_SetParamPID( typePidParam, value );
  snprintf_( OutputBuffer, OutputBufferLen, "New value accepted\r\n" );

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

static BaseType_t CLI_InfoCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString )
{
  OutputBuffer[0] = 0x00;

  Serial_Message( "      Kp: %f\r\n", Control_GetParamPID( ePID_KP ) );
  Serial_Message( "      Ki: %f\r\n", Control_GetParamPID( ePID_KI ) );
  Serial_Message( "      Kd: %f\r\n", Control_GetParamPID( ePID_KD ) );
  Serial_Message( "SetPoint: %f\r\n", Control_GetParamPID( ePID_SetPoint ) );
  Serial_Message( "     Dir: %s\r\n", Control_GetDirectionString( Control_GetDirection() ) );

  return pdFALSE;
}

static BaseType_t CLI_DirectionCommand( char *OutputBuffer, size_t OutputBufferLen, const char *CommandString )
{
  char *parameter;
  BaseType_t parameterStringLength;
  CarDirection_e direction;

  /* Remove compile time warnings about unused parameters */
  ( void ) CommandString;
  ( void ) OutputBufferLen;

  parameter = (char *)FreeRTOS_CLIGetParameter( CommandString, 1, &parameterStringLength );
  parameter[ parameterStringLength ] = 0x00;

  if( strcmp( parameter, "stop" ) == 0 )
  {
    direction = eCAR_DIR_STOP;
  }
  else if( strcmp( parameter, "front" ) == 0 )
  {
    direction = eCAR_DIR_FRONT;
  }
  else if( strcmp( parameter, "back" ) == 0 )
  {
    direction = eCAR_DIR_BACK;
  }
  else if( strcmp( parameter, "right" ) == 0 )
  {
    direction = eCAR_DIR_RIGHT;
  }
  else if( strcmp( parameter, "left" ) == 0 )
  {
    direction = eCAR_DIR_LEFT;
  }
  else
  {
    snprintf_( OutputBuffer, OutputBufferLen, "Invalid parameter\r\n" );
    return pdFALSE;
  }

  Control_SetDirection( direction );
  snprintf_(OutputBuffer, OutputBufferLen, "New direction set\r\n" , parameter);

  /* There is no more data to return after this single string, so return pdFALSE. */
  return pdFALSE;
}

//==============================================================================
// Exported functions
//==============================================================================

void CLI_RegisterCommands( void )
{
  FreeRTOS_CLIRegisterCommand ( &CliCmd_Reset );
  FreeRTOS_CLIRegisterCommand ( &CliCmd_Trace );
  FreeRTOS_CLIRegisterCommand ( &CliCmd_PID );
  FreeRTOS_CLIRegisterCommand ( &CliCmd_GetInfo );
  FreeRTOS_CLIRegisterCommand ( &CliCmd_Direction );
}
