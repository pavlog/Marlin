/*
  Hysteresis.cpp - A class that manages hysteresis by inserting extra plan_buffer_line when necessary
 
  Copyright (c) 2012 Neil James Martin
 
 Grbl is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Grbl is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Hysteresis.h"

#include "Marlin.h"
#include "planner.h"
#include "stepper.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"

//===========================================================================
#if defined (DEFAULT_HYSTERESIS_MM)
Hysteresis hysteresis( DEFAULT_HYSTERESIS_MM );
#else
Hysteresis hysteresis( 0,0,0,0 );
#endif
//===========================================================================
Hysteresis::Hysteresis( float x_mm, float y_mm, float z_mm, float e_mm )
{
  m_prev_direction_bits = 0;
  Set( x_mm, y_mm, z_mm, e_mm );
}

//===========================================================================
void Hysteresis::Set( float x_mm, float y_mm, float z_mm, float e_mm )
{
  m_hysteresis_mm[X_AXIS] = x_mm;
  m_hysteresis_mm[Y_AXIS] = y_mm;
  m_hysteresis_mm[Z_AXIS] = z_mm;
  m_hysteresis_mm[E_AXIS] = e_mm;
  m_hysteresis_bits = ((m_hysteresis_mm[X_AXIS]!=0.0f)?(1<<X_AXIS):0)
                    | ((m_hysteresis_mm[Y_AXIS]!=0.0f)?(1<<Y_AXIS):0)
                    | ((m_hysteresis_mm[Z_AXIS]!=0.0f)?(1<<Z_AXIS):0)
                    | ((m_hysteresis_mm[E_AXIS]!=0.0f)?(1<<E_AXIS):0);
}

//===========================================================================
void Hysteresis::SetAxis( int axis, float mm )
{
          SERIAL_PROTOCOLPGM(" sethyst axis:");      
        SERIAL_PROTOCOL(axis);
        SERIAL_PROTOCOLPGM(" val:");      
        SERIAL_PROTOCOL(mm);
        SERIAL_PROTOCOLLN("");


  m_hysteresis_mm[axis] = mm;
  if( mm != 0.0f )
  {
    m_hysteresis_bits |= (1<<axis);
  }
  else
  {
    m_hysteresis_bits &= ~(1<<axis);
  }
}

//===========================================================================
float Hysteresis::GetAxis( int axis ) const
{
  return m_hysteresis_mm[axis];
}

//===========================================================================
void Hysteresis::ReportToSerial()
{
  SERIAL_PROTOCOLPGM("H=X");
  SERIAL_PROTOCOL(m_hysteresis_mm[X_AXIS]);
  SERIAL_PROTOCOLPGM(" Y");
  SERIAL_PROTOCOL(m_hysteresis_mm[Y_AXIS]);
  SERIAL_PROTOCOLPGM(" Z");
  SERIAL_PROTOCOL(m_hysteresis_mm[Z_AXIS]);
  SERIAL_PROTOCOLPGM(" E");      
  SERIAL_PROTOCOL(m_hysteresis_mm[E_AXIS]);
  
  SERIAL_PROTOCOLLN("");
}

//===========================================================================
unsigned char calc_direction_bits( const float* current_position, const float* destination )
{
  unsigned char direction_bits = 0;
  if (destination[X_AXIS] < current_position[X_AXIS]) { 
    direction_bits |= (1<<X_AXIS); 
  }
  if (destination[Y_AXIS] < current_position[Y_AXIS]) { 
    direction_bits |= (1<<Y_AXIS); 
  }
  if (destination[Z_AXIS] < current_position[Z_AXIS]) { 
    direction_bits |= (1<<Z_AXIS); 
  }
  if (destination[E_AXIS] < current_position[E_AXIS]) { 
    direction_bits |= (1<<E_AXIS); 
  }
  return direction_bits;
}

//===========================================================================
// calculate the slowest max feedrate for the axis directions being used
float calc_best_feedrate( const float* current_position, const float* destination )
{
  float best_feedrate = max(max_feedrate[X_AXIS],max(max_feedrate[Y_AXIS],max(max_feedrate[Z_AXIS],max_feedrate[E_AXIS])));
  for(int axis=0;axis<NUM_AXIS;++axis)
  {
    if (destination[axis] != current_position[axis]) 
    { 
      best_feedrate = min( best_feedrate, max_feedrate[axis] );
    }
  }
  return best_feedrate;  
}

//===========================================================================
// insert a plan_buffer_line if required to handle any hysteresis
void Hysteresis::InsertCorrection(const float &x, const float &y, const float &z, const float &e,const float& target_xmm, const float& target_ymm)
{
  float destination[NUM_AXIS] = {x,y,z,e};
  unsigned char direction_bits = calc_direction_bits( current_position, destination );
  // if the direction has changed in any of the axis that need hysteresis corrections...
  unsigned char direction_change_bits = (direction_bits ^ m_prev_direction_bits);
  if( (direction_change_bits & m_hysteresis_bits) != 0 )
  {
    // calculate the position to move to that will fix the hysteresis
    float fixed_pos[NUM_AXIS];
    for(int axis=0;axis<NUM_AXIS;++axis)
    {
      fixed_pos[axis] = current_position[axis];
      // if this axis changed direction...
      if( direction_change_bits & (1<<axis) && (m_hysteresis_bits & (1<<axis)))
      {
        //... add the hysteresis
        float hyst =  (((direction_bits&(1<<axis))!=0)?m_hysteresis_mm[axis]:m_hysteresis_mm[axis]);
     
        //SERIAL_PROTOCOLPGM(" hyst axis:");      
        //SERIAL_PROTOCOL(axis);
        //SERIAL_PROTOCOLPGM(" val:");      
        //SERIAL_PROTOCOL(hyst);
        //SERIAL_PROTOCOLLN("");


        fixed_pos[axis] = fixed_pos[axis] + hyst;
      }
    }
    float best_feedrate = calc_best_feedrate( current_position, destination );

      #if ENABLED(DELTA)
        fixed_pos[X_AXIS] = destination[X_AXIS];
        fixed_pos[Y_AXIS] = destination[Y_AXIS];
        fixed_pos[Z_AXIS] = destination[Z_AXIS];
      #elif ENABLED(DELTAXY) || ENABLED(SCARA)
        fixed_pos[X_AXIS] = destination[X_AXIS];
        fixed_pos[Y_AXIS] = destination[Y_AXIS];
      #endif
///*
      // debug output to display any hysteresis corrections.
      SERIAL_PROTOCOLPGM("From=X");
      SERIAL_PROTOCOL(current_position[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y");
      SERIAL_PROTOCOL(current_position[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z");
      SERIAL_PROTOCOL(current_position[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E");      
      SERIAL_PROTOCOL(current_position[E_AXIS]);

      SERIAL_PROTOCOLLN("");

      SERIAL_PROTOCOLPGM("  To=X");
      SERIAL_PROTOCOL(fixed_pos[X_AXIS]);
      SERIAL_PROTOCOLPGM(" Y");
      SERIAL_PROTOCOL(fixed_pos[Y_AXIS]);
      SERIAL_PROTOCOLPGM(" Z");
      SERIAL_PROTOCOL(fixed_pos[Z_AXIS]);
      SERIAL_PROTOCOLPGM(" E");      
      SERIAL_PROTOCOL(fixed_pos[E_AXIS]);
      
      SERIAL_PROTOCOLPGM(" F");      
      SERIAL_PROTOCOL(best_feedrate);
      

      SERIAL_PROTOCOLLN("");
//*/
      m_prev_direction_bits = direction_bits; // need to set these now to avoid recursion as plan_buffer_line calls this function
      #if ENABLED(SCARA)
      plan_buffer_line(fixed_pos[X_AXIS], fixed_pos[Y_AXIS], fixed_pos[Z_AXIS], fixed_pos[E_AXIS], best_feedrate, active_extruder,target_xmm,target_ymm);
      #else
      plan_buffer_line(fixed_pos[X_AXIS], fixed_pos[Y_AXIS], fixed_pos[Z_AXIS], fixed_pos[E_AXIS], best_feedrate, active_extruder,target_xmm,target_ymm);
      #endif
  }
  m_prev_direction_bits = direction_bits;
}



