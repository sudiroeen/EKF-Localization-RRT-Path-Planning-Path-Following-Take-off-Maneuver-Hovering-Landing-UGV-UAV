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

/* Your custom remote API functions. */

if (_rawCmdID == simx_customcmd_get_object_name)
{
    // First retrieve the data that is part of the command (i.e. the handle of the object). 
    // The server is in charge of handling little endian to big endian conversions:
    simInt handle = littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);

    // The new object name is pure data (i.e. not part of the command). The pure data is located in _data

    // Execute the command:
    simChar* objectName = simGetObjectName(handle);
    if (objectName != NULL) {
        retCmd->setDataReply_custom_copyBuffer(objectName, int(strlen(objectName)+1), true);
        simReleaseBuffer(objectName);
    }
    else {
        retCmd->setDataReply_nothing(false);
    }
}

if (_rawCmdID == simx_customcmd_get_joint_type)
{
    simInt handle = littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);
    simInt jointType = simGetJointType(handle);
	 retCmd->setDataReply_1int(jointType, (jointType != -1), otherSideIsBigEndian);
}

if (_rawCmdID == simx_customcmd_get_joint_interval)
{
    simInt handle = littleEndianIntConversion(((int*)(_cmdData+0))[0],otherSideIsBigEndian);

    simBool cyclic;
    simFloat interval[2];
    simInt success = simGetJointInterval(handle, &cyclic, interval);
    char buffer[9];
    buffer[0] = (char)cyclic;
    *((float*)&buffer[1]) = littleEndianFloatConversion(interval[0], otherSideIsBigEndian);
    *((float*)&buffer[5]) = littleEndianFloatConversion(interval[1], otherSideIsBigEndian);

    retCmd->setDataReply_custom_copyBuffer(buffer, 9, (success != -1));
}

