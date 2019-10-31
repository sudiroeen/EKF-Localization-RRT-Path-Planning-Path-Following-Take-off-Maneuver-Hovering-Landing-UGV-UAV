// This file is part of the REMOTE API
// 
// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// The REMOTE API is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.1 on January 20th 2013

#include "common_include/extApiCustom.h"
#include "common_include/extApi.h"
#include "common_include/extApiInternal.h"
#include "common_include/extApiCustomConst.h"

/* Your custom remote API functions. */

EXTAPI_DLLEXPORT simxInt simxCustomGetObjectName(simxInt clientID, simxInt objectHandle, simxChar** objectName, simxInt operationMode) 
{
    simxChar* dataPointer;
    simxInt returnValue;

    //First catch a possible error:
    if (_communicationThreadRunning == 0)
        return(simx_error_initialize_error_flag);

    //Then take care of the "remove" operation mode:
    if (operationMode == simx_opmode_remove)
        //_int indicates that the command is identified by the command ID AND an integer value
        return(_removeCommandReply_int(clientID, simx_customcmd_get_object_name, objectHandle)); 

    //Now "execute" the command (i.e. place the command (with its associated command data) 
    //into the outbox, and retrieve a command reply from the inbox)
    //_int indicates that the command is identified by the command ID AND an integer value
    dataPointer = _exec_int(clientID, simx_customcmd_get_object_name, operationMode, 0, objectHandle, &returnValue);

    //Now extract the desired data:
    //pure data is the data that is not used to identify a command
    if ((dataPointer != 0) && (returnValue == 0)) {
	     objectName[0] = dataPointer + SIMX_SUBHEADER_SIZE + _getCmdDataSize(dataPointer);
    }

    //return the error code
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxCustomGetJointType(simxInt clientID, simxInt objectHandle, simxInt* jointType, simxInt operationMode)
{
    simxChar* dataPointer;
    simxInt returnValue;

    if (_communicationThreadRunning == 0)
        return(simx_error_initialize_error_flag);

    if (operationMode == simx_opmode_remove)
        return(_removeCommandReply_int(clientID, simx_customcmd_get_joint_type, objectHandle)); 

    dataPointer = _exec_int(clientID, simx_customcmd_get_joint_type, operationMode, 0, objectHandle, &returnValue);
    
    if ((dataPointer != 0) && (returnValue == 0)) {
	     jointType[0] = _readPureDataInt(dataPointer, 0, 0);
    }

    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxCustomGetJointInterval(simxInt clientID, simxInt objectHandle, simxChar* cyclic, simxFloat* interval, simxInt operationMode)
{
    simxChar* dataPointer;
    simxInt returnValue;

    if (_communicationThreadRunning == 0)
        return(simx_error_initialize_error_flag);

    if (operationMode == simx_opmode_remove)
        return(_removeCommandReply_int(clientID, simx_customcmd_get_joint_interval, objectHandle)); 

    dataPointer = _exec_int(clientID, simx_customcmd_get_joint_interval, operationMode, 0, objectHandle, &returnValue);
    
    if ((dataPointer != 0) && (returnValue == 0)) {
	     cyclic[0] = _readPureDataChar(dataPointer, 0, 0);
	     interval[0] = _readPureDataFloat(dataPointer, 0, 1);
	     interval[1] = _readPureDataFloat(dataPointer, 0, 5);
    }

    return(returnValue);
}

