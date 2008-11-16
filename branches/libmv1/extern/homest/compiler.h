/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-06  Manolis Lourakis (lourakis@ics.forth.gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef _COMPILER_H
#define _COMPILER_H

/***
compiler-defined macros:

__GNUC__, __GNUC_MINOR___  // gcc
_MSC_VER                   // msvc
***/

/* C compiler specific constants & macros */


#ifndef __GNUC__
#ifdef _MSC_VER
#define inline __inline
#else
#define inline // empty
#endif
#endif


#endif /* _COMPILER_H */
