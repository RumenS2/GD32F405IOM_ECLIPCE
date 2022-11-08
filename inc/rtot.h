/*
 * NTC thermistor library
 * Copyright (C) 2007 - SoftQuadrat GmbH, Germany
 * Contact: thermistor (at) softquadrat.de
 * Web site: thermistor.sourceforge.net
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
 * USA
 */
#ifndef __RTOT_H
#define __RTOT_H
/**************
* Prototyping *
**************/
#include "gd32f4xx.h"
typedef float  fprecicion;
//typedef double fprecicion;

/* Evaluates p(x) for a polynom p. */
fprecicion poly(fprecicion x, int degree, fprecicion p[]);
/* Conversion from resistance to temperature. */
fprecicion rtot(fprecicion r);

inline float Uvfrom12bit(float atmp1)
{
//  return(3.3f*atmp1/4096.0f);
	return(atmp1*8.056640625e-4f);
}

float OhmFromV(float Uv);

#endif
