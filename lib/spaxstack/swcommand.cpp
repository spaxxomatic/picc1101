/**
 * Copyright (c) 2018 autonity <contact@autonity.de>
 * 
 * This file is part of the spaxxity project.
 * 
 * spaxxity is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * spaxxity is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with spaxxity; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer, Lucian Nutiu 
 * Creation date: 03/03/2011
 */

#include "swcommand.h"
#include "spaxstack.h"

/**
 * SWCOMMAND
 * 
 * Class constructor
 * 
 * 'dAddr'	  Destination address
 * 'rAddr'	  Register address
 * 'rId'	    Register id
 * '*val'	    New value
 * 'len'	    Buffer length
 */
SWCOMMAND::SWCOMMAND(byte dAddr, byte rAddr, byte registerId, byte *val, byte len)
{
  destAddr = dAddr;
  srcAddr = MASTER_ADDRESS;
  hop = 0;
  function = SWAPFUNCT_CMD;
  regAddr = rAddr;
  regId = registerId;
  value.data = val;
  value.length = len;
  value.is_string = false;
}

