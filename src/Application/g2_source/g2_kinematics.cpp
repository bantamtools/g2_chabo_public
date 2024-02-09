
/*
 * kinematics.cpp - inverse kinematics routines
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2016 - 2019 Rob Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "g2core.h"
#include "g2_config.h"
#include "g2_canonical_machine.h"
#include "g2_stepper.h"
#include "g2_kinematics.h"
#include "g2_util.h"
#include "g2_settings.h"
#include "g2_encoder.h" // for encoder grabbing

#include <atomic>


// Helper classes
volatile int g_nan_or_inf_counter=0;
template<typename t>
struct Point3_ 
{
    t x;
    t y;
    t z;

    constexpr t& operator[] (const std::size_t index) 
    {
        // assert(index >= 0 && index < 3);
        switch (index) 
        {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }
    constexpr const t& operator[] (const std::size_t index) const 
    {
        // assert(index >= 0 && index < 3);
        switch (index) 
        {
            case 0: return x;
            case 1: return y;
            default: return z;
        }
    }

    Point3_<t> operator+ (const Point3_<t> &p) const 
    {
        return 
        {
            x+p.x,
            y+p.y,
            z+p.z
        };
    }

    t distance_to(const Point3_<t> &p) const 
    {
        t temp_x = (x-p.x);
        t temp_y = (y-p.y);
        t temp_z = (z-p.z);
        return std::sqrt(
            temp_x*temp_x +
            temp_y*temp_y +
            temp_z*temp_z
        );
    }
};
typedef Point3_<float> Point3F;
typedef Point3_<double> Point3D;

// gpioDigitalInputHandler _pin_input_handler;

// Actual kinematics classes

template <uint8_t axes, uint8_t motors>
struct CartesianKinematics : KinematicsBase<axes, motors> 
{
    static const uint8_t joints = axes; // For cartesian we have one joint per axis

    // Joints are defined as these axes in order:
    //  0 = X
    //  1 = Y
    //  2 = Z
    //  3 = A
    //  4 = B
    //  5 = C
    //  6 = U (maybe)
    //  7 = V (maybe)
    //  8 = W (maybe)
    volatile float steps_accum[motors];;//sme--diag
    volatile float steps_per_unit[motors];
    volatile float motor_offset[motors];
    bool needs_sync_encoders = true; // if true, we need to update the steps_offset
    volatile int8_t motor_map[motors];  // for each motor, which joint it maps from

    float joint_position[joints];

    void configure(const float new_steps_per_unit[motors], const int8_t new_motor_map[motors]) override
    {
        for (uint8_t motor = 0; motor < motors; motor++) 
        {
            motor_map[motor] = new_motor_map[motor];
            int8_t joint = motor_map[motor];
            if (joint == -1) 
            {
                motor_offset[motor] = 0;
                steps_per_unit[motor] = 1;
            } 
            else 
            {
                float steps = (joint_position[joint] * steps_per_unit[motor]) + motor_offset[motor];
                steps_per_unit[motor] = new_steps_per_unit[motor];
                motor_offset[motor] = steps - (joint_position[joint] * steps_per_unit[motor]);
            }
        }
    }

    void inverse_kinematics(const float target[axes], const float position[axes], const float start_velocity,
                            const float end_velocity, const float segment_time, float steps[motors]) override
    {
        volatile static int nan_counter=0;
        volatile static int inf_counter=0;
        volatile static int8_t joint=0; 
        volatile static float target_copy=0;
        
        // joint == axis in cartesian kinematics
        for (uint8_t motor = 0; motor < motors; motor++) 
        {
            int8_t joint = motor_map[motor];
            if (joint == -1) 
            {
                continue;
            }
            target_copy = target[joint];
            steps[motor] = (target[joint] * steps_per_unit[motor]) + motor_offset[motor];
            steps_accum[motor] +=steps[motor];//sme: diags
            
            if (isinf(steps[motor])==true)
            {
              inf_counter++;
            }
            if (isnan(steps[motor])==true)
             {  
                nan_counter++;
             }            
        }

        for (uint8_t joint = 0; joint < joints; joint++) 
        {
            joint_position[joint] = target[joint];
        }
        if (inf_counter>0)
        {           
            g_nan_or_inf_counter++;        
        }
        if (nan_counter>0)
        {            
            g_nan_or_inf_counter++;       
        }        
    }

    void get_position(float position[axes]) override
    {
        for (uint8_t axis = 0; axis < axes; axis++) 
        {
            position[axis] = joint_position[axis];
        }
    }

    float best_steps_per_unit[axes];

    void forward_kinematics(const float steps[joints], float position[axes]) override
    {
        // Setup
        for (uint8_t axis = 0; axis < axes; axis++) 
        {
            position[axis] = 0.0;
        }
        for (uint8_t motor = 0; motor < motors; motor++) 
        {
            int8_t joint = motor_map[motor];
            if (joint == -1) 
            {
                continue;
            }

            auto axis = joint; // it's cartesian, baby!
            best_steps_per_unit[axis] = -1.0;

            // If this motor has a better (or the only) resolution, then we use this motor's value
            if (best_steps_per_unit[axis] < steps_per_unit[motor]) 
            {
                best_steps_per_unit[axis] = steps_per_unit[motor];
                position[axis]            = (steps[motor]-motor_offset[motor]) / steps_per_unit[motor];
            }

            joint_position[joint] = position[joint];
        }
    }

    void sync_encoders(const float step_position[motors], const float position[axes]) override 
    {
        // We need to make joint_offset[joint] adjust any given position so that if it's given as a target
        // to inverse_kinematics then step_position[motor] will be given as the return steps[motor]
        // Why? Externally position[] may be unrelated to step_position[], so we need to adjust.
        for (uint8_t motor = 0; motor < motors; motor++) 
        {
            int8_t joint = motor_map[motor];
            if (joint == -1) 
            {
                continue;
            }

            // This, solved for motor_offset: step_position[motor] = (position[joint]] * steps_per_unit[motor]) + motor_offset[motor];
            motor_offset[motor] = step_position[motor] - (position[joint] * steps_per_unit[motor]);
        }
        __NOP();//sme
    }
};
 
/* Specific Functions
 *
 * From here we tie the generic classes to specific values provided in globals.
 *
 */


CartesianKinematics<AXES, MOTORS> cartesian_kinematics;
KinematicsBase<AXES, MOTORS> *kn = &cartesian_kinematics;
 
/*
 * kn_config_changed() - call to update the configuration from the globals
 */
void kn_config_changed() 
{
    // temporary load these up every time until we can hook them to the configuration
    int8_t motor_map[MOTORS];
    float steps_per_unit[MOTORS];

    for (uint8_t motor = 0; motor < MOTORS; motor++) 
    {
        auto axis = st_cfg.mot[motor].motor_map;
        if (cm->a[axis].axis_mode == AXIS_INHIBITED) 
        {
            motor_map[motor] = -1;
            steps_per_unit[motor] = 1; // this is the denominator above, avoid 0
        }
        else 
        {
            motor_map[motor] = axis;
            steps_per_unit[motor] = st_cfg.mot[motor].steps_per_unit;
        }
    }
#if 1//sme: work around strange link error
    __NOP();
    cartesian_kinematics.configure(steps_per_unit, motor_map);
    __NOP();
#else //strange that link cannot see the above pointer assigned to the file scope instance!!!!!
    kn->configure(steps_per_unit, motor_map);
#endif
    mp_set_steps_to_runtime_position();
}

/*
 * kn_forward_kinematics() - forward kinematics for a cartesian machine
 *
 * This is designed for PRECISION, not PERFORMANCE!
 *
 * This function is NOT to be used where high-speed is important. If that becomes the case,
 * there are many opportunities for caching and optimization for performance here.
 *
 */

void kn_forward_kinematics(const float steps[], float travel[]) 
{
 #if 1//sme: work around strange link error
    cartesian_kinematics.forward_kinematics(steps, travel);
#else //strange that link cannot see the above pointer assigned to the file scope instance!!!!! 
    // PRESUMPTION: inverse kinematics has been called at least once since the mapping or steps_unit has changed
    kn->forward_kinematics(steps, travel);
#endif    
}
//sme: force a wrapper function to work around C++ link issue
void kn_sync_encoders(const float step_position[], const float position[])
{
  cartesian_kinematics.sync_encoders(step_position,position);
}
