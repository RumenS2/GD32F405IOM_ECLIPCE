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

/** @file rtot.c
 * Program calculating the temperature from resistance value of an NTC thermistor.
 * The Steinhart-Hart polynom allows calculation of absolute temperature
 * from resistance of an NTC thermistor
 *
 * <center><i>
 * 1/t = 1/t<sub>0</sub>
 *       + c<sub>1</sub> &middot; ln(r/r<sub>0</sub>)
 *       + c<sub>2</sub> &middot; ln(r/r<sub>0</sub>)<sup>2</sup>
 *       + c<sub>3</sub> &middot; ln(r/r<sub>0</sub>)<sup>3</sup>
 * </i></center>
 *
 * where (<b>r<sub>0</sub></b>,<b>t<sub>0</sub></b>) is a fixed resistance temperature pair.
 *
 * By substitution
 *
 * <center><i>
 * ln(r/r<sub>0</sub>) = ln(r) - ln(r<sub>0</sub>)
 * </i></center>
 *
 * this leads to a polynom in <b>ln(r)</b>
 *
 * <center><i>
 * 1/t = a<sub>0</sub>
 *       + a<sub>1</sub> &middot; ln r
 *       + a<sub>2</sub> &middot; (ln r)<sup>2</sup>
 *       + a<sub>3</sub> &middot; (ln r)<sup>3</sup>
 * </i></center>
 *
 * The program calculates the calculates the temperature from a given resistance value of an NTC
 * according to that formula.
 */

/***********
* Includes *
***********/
//#include <stdio.h>
#include "rtot.h"
#include <math.h>

//#include "arm_comm.h"
#include "gd32f4xx.h"


/**********
* Defines *
**********/
/** Absolute Zero. */
#define TABS (-273.15)

/************
* Variables *
************/
/** Coefficients of Steinhart-Hart polynom. */
//double a[] = {
//   4.524024725919526e-004,
//   3.934722516618191e-004,
//   -7.642331765196044e-006,
//   4.048572707661904e-007,
//};
fprecicion a[] = {
 1.121556212934832e-003,
 2.361479683360064e-004,
 -1.743711409727249e-007,
 9.245535641872339e-008,
};

/**************
* Prototyping *
**************/
/* Evaluates p(x) for a polynom p. */
//double poly(double x, int degree, double p[]);
/* Conversion from resistance to temperature. */
//double rtot(double r);

/************
* Functions *
************/
/**
 * Evaluates p(x) for a polynom p.
 * Calculates the value of polynom p at x accordings to
 * Horners schema.
 * @param p polynom.
 * @param x value to be inserted into the polynom.
 * @return calculated polynom value.
 */
fprecicion poly(fprecicion x, int degree, fprecicion p[]) {
  fprecicion retval = 0.0;
  int i;

  for (i = degree; i >= 0; i--)
  	retval = retval * x + p[i];
  return retval;
}

/* Conversion from resistance to temperature.
 * Calculates and returns temperature for given resistance.
 * @param t resistance (in Ohm).
 * @return corresponding temperature.
 */
fprecicion rtot(fprecicion r)
{
  fprecicion ti;

  ti = poly(log(r), 3, a);
  ti = 1.0 / ti + TABS;
  return ti;
}

///============================================================================================//

#define Rct 270.0f
float OhmFromV(float Uv)
{
  if (Uv<0.0001f) {return(13000000.0f);} else {return(((5.20f-Uv)*Rct)/Uv);}   //not 3.3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}
//===========================================================================================//



