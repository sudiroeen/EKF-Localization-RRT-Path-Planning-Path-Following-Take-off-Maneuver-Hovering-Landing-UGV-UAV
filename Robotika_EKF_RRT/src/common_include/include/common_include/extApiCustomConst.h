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

#if !defined(EXTAPICUSTOMCONST_INCLUDED_)
#define EXTAPICUSTOMCONST_INCLUDED_

#include "v_repConst.h"

/* Your custom remote API command IDs. */
enum {	
    /* 
        from here on, commands are also identified by additional 4 bytes. For example,
        the command to retrieve an object type has no meaning if sent alone: an object
        handle to which the command applies is required. So the command is identified
        by a command ID (value here below), and an object handle (1 integer=4 bytes) 
    */
    simx_customcmd_get_object_name = simx_cmd4bytes_custom_start,
    simx_customcmd_get_joint_type,
    simx_customcmd_get_joint_interval,
};


#endif /* !defined(EXTAPICUSTOMCONST_INCLUDED_) */
