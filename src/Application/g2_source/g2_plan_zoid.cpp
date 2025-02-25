
/*
 * plan_zoid.cpp - acceleration managed line planning and motion execution - trapezoid planner
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2019 Rob Giseburt
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
#include "g2_planner.h"
#include "g2_report.h"
#include "g2_util.h"



stat_t _ramp_exit_logger(mpBuf_t* bf, const char *msg)
{
    #ifdef __PLANNER_DIAGNOSTICS
    if (mp_runtime_is_idle()) 
    {  // normally the runtime keeps this value fresh
        bf->plannable_time_ms += bf->block_time_ms; 
    }
    #endif

/* insert logger functions here if needed:

    // LOG_RETURN with full state dump
    #if IN_DEBUGGER == 1

    static char logbuf[128];

    sprintf(logbuf, "[%2d] %s (%d) mt:%5.2f, L:%1.3f [%1.3f, %1.3f, %1.3f] V:[%1.2f, %1.2f, %1.2f]\n",
                    bf->buffer_number, msg, bf->hint, (bf->block_time * 60000),
                    bf->length, bf->head_length, bf->body_length, bf->tail_length,
                    bf->pv->exit_velocity, bf->cruise_velocity, bf->exit_velocity);
    comms_mgr_write_msg(logbuf);
    #endif
*/
    return (STAT_OK);
}

// END DIAGNOSTICS

/* local functions */

// static float _get_target_length_min(const float v_0, const float v_1, const mpBuf_t *bf, const float min);
static float _get_meet_velocity(const float          v_0,
                                const float          v_2,
                                const float          L,
                                mpBuf_t*             bf,
                                mpBlockRuntimeBuf_t* block);

/****************************************************************************************
 * mp_calculate_ramps() - calculate trapezoid-like ramp parameters for a block
 *
 *  This long-ish function sets section lengths and velocities based on the move length
 *  and velocities requested. It modifies the incoming bf buffer and returns accurate
 *  head, body and tail lengths, and accurate or reasonably approximate velocities.
 *  We care about length accuracy, less so for velocity (as long as jerk is not exceeded).
 *
 *  We need velocities to be set even for zero-length sections (nb: sections, not moves)
 *  so plan_exec can compute entry and exits for adjacent sections.
 *
 *  Note we use three data structures: mr, bf, and block.
 *
 *  bf holds the data from aline and back-planning. For the most part it is immutable.
 *
 *  block is the data for forward-planning. There are only two block structures, and they
 *  are for the current block and the next block.
 *
 *  bf values treated as constants:
 *    All except cruise_vmax, override_factor, block_time and hint
 *
 *  mr holds the current velocity, which is the entry_velocity to the first block. The
 *  exit_velocity of the first block is the entry of the next. To save having to figure
 *  out what block this is, we pass mp_calculate_ramps the entry_velocity it is to use.
 *
 *    All values of block are expected to be setup by mp_calculate_ramps.
 *
 *  Quick cheat-sheet on which is in buffer (bf) and which is in block:
 *    buffer (bf):
 *      block_type
 *      hint
 *      {cruise,exit}_vmax
 *      block_time
 *      length
 *      (start values of {cruise,exit}_velocity)
 *
 *    block:
 *      {cruise,exit}_velocity (final)
 *      {head,body,tail}_length
 *      {head,body,tail}_time
 *
 */

// Hint will be one of these from back-planning: COMMAND_BLOCK, PERFECT_DECELERATION, PERFECT_CRUISE,
// MIXED_DECELERATION, ASYMMETRIC_BUMP
// We are incorporating both the forward planning and ramp-planning into one function, since we use the same data.

stat_t mp_calculate_ramps(mpBlockRuntimeBuf_t* block, mpBuf_t* bf, const float entry_velocity)
{
#if 1//debug only sme: 9-20-2021: something causes bf-> buffer_number for skip over the present number to the next.
  volatile uint8_t buffer_number=bf->buffer_number;
  volatile float len = bf->length;
  //volatile  mpBuf_t* bf_copy=bf;
#endif

    // *** Skip non-move commands ***
    if (bf->block_type == BLOCK_TYPE_COMMAND)
    {
        bf->hint = COMMAND_BLOCK;
        return (STAT_NOOP);                             // NOOP status is informative, not actionable
    }
    debug_trap_if_zero(bf->length, "mp_calculate_ramps() - got L=0");
    debug_trap_if_zero(bf->cruise_velocity, "mp_calculate_ramps() - got Vc=0");

    // Timings from *here*

    // initialize parameters to known values
    block->head_time = 0;
    block->body_time = 0;
    block->tail_time = 0;
    block->head_length = 0;
    block->body_length = 0;
    block->tail_length = 0;

    // handle overrides
    bf->override_factor = 1.0;
    if (bf->gm.motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) 
    {
        bf->override_factor = cm->gmx.mto_enable ? cm->gmx.mto_factor : 1.0;
    }
    else if ((bf->gm.motion_mode == MOTION_MODE_STRAIGHT_FEED) 
            || (bf->gm.motion_mode == MOTION_MODE_CW_ARC) 
            || (bf->gm.motion_mode == MOTION_MODE_CCW_ARC)) 
    {
        bf->override_factor = cm->gmx.mfo_enable ? cm->gmx.mfo_factor : 1.0;
    }

    // bf->cruise_vmax adjusted by override cannot go above absolute vmax,
    //   and should stay below the back-planned cruise velocity.
    bf->cruise_vmax = std::min(bf->absolute_vmax, 
                      std::min(bf->cruise_velocity, bf->override_factor * bf->cruise_vset));

    //   also cannot go below the entry velocity, but if it does,
    //   we have to make sure that the exit velocity reflects what we wanted cruise to be
    if (bf->cruise_vmax < entry_velocity) 
    {
        // make sure the bf->exit_velocity is not higher than we wanted to cruise
        bf->exit_velocity = std::min(bf->exit_velocity, bf->cruise_vmax);
        // now we set the cruise to entry, since it cannot stay lower
        bf->cruise_vmax = entry_velocity;
    }

    // these conditions should have been met earlier, but if they are not trap and correct them
    debug_trap_if_true((bf->exit_velocity > bf->exit_vmax), "mp_calculate_ramps() - Vexit > Vexit_max");
    block->exit_velocity   = std::min(bf->exit_velocity, bf->exit_vmax);

    // Update the stitching with the next move, if there is one, to ensure we don't try to exit too high
    if (bf->nx->buffer_state >= MP_BUFFER_BACK_PLANNED) 
    {
        auto nx_cruise_vmax = std::min(bf->nx->absolute_vmax, bf->override_factor * bf->nx->cruise_vset);
        block->exit_velocity = std::min(block->exit_velocity, nx_cruise_vmax);
    }

    // actual cruise velocity cannot be below entry or exit velocities, but can be equal to the highest
    block->cruise_velocity = std::max(block->exit_velocity, bf->cruise_vmax);

    // *** Perfect-Fit Cases (1) *** Cases where curve fitting has already been done

    // PERFECT_CRUISE (1c) Velocities all match (or close enough), treat as body-only
    // NOTE: PERFECT_CRUISE is set in back-planning without knowledge of pv->exit, since it can't see it yet.
    // Here we verify it moving forward, checking to make sure it still is true.
    // If so, we plan the "ramp" as flat, body-only.
    if (bf->hint == PERFECT_CRUISE) 
    {
        if ((!mp->entry_changed) && fp_EQ(entry_velocity, bf->cruise_vmax)) 
        {
            // We need to ensure that neither the entry or the exit velocities are
            // <= the cruise velocity even though there is tolerance in fp_EQ comparison.
            block->exit_velocity   = entry_velocity;
            block->cruise_velocity = entry_velocity;
            block->body_length = bf->length;
            block->body_time   = block->body_length / block->cruise_velocity;
            bf->block_time     = block->body_time;
            return (_ramp_exit_logger(bf, "1c"));
        }
        else 
        {  // degrade the hint to MIXED_ACCELERATION
            bf->hint = MIXED_ACCELERATION;
        }
    }

    // Quick test to ensure we haven't violated the hint

    if (entry_velocity > block->exit_velocity) 
    {    // Means this is a deceleration
        //  If entry_changed then entry_velocity is lower than the hints expect
        //  A deceleration will never become an acceleration (post-hinting)
        //  If it is marked as MIXED_DECELERATION, it means the entry was CRUISE_VMAX
        //  If it is marked as PERFECT_DECELERATION, it means the entry was as <= CRUISE_VMAX,
        //   but as high as possible
        //  So, this move is and was a DECELERATION, meaning it *could* achieve the previous (higher)
        //   entry safely, it will likely get a head section, so we will now degrade the hint to an
        //   ASYMMETRIC_BUMP
        if (mp->entry_changed) 
        {
            bf->hint = ASYMMETRIC_BUMP;
        }

        // MIXED_DECELERATION (2d) 2 segment BT deceleration move
        // Only possible if the entry has not changed since hinting.
        else if (bf->hint == MIXED_DECELERATION) 
        {
            block->tail_length = mp_get_target_length(block->exit_velocity, block->cruise_velocity, bf);
            block->body_length = bf->length - block->tail_length;
            if (block->body_length < 0) 
            {
                debug_trap_if_true((block->body_length < 0), "invalid negative body_length from MIXED_DECELERATION");
                // something went wrong and we missed it, try for an asymmetric bump
                bf->hint = ASYMMETRIC_BUMP;
            } 
            else 
            {
                block->head_length = 0;
                block->body_time = block->body_length / block->cruise_velocity;
                block->tail_time = block->tail_length * 2 / (block->exit_velocity + block->cruise_velocity);
                bf->block_time   = block->body_time + block->tail_time;
                return (_ramp_exit_logger(bf, "2d"));
            }
        }

        // PERFECT_DECELERATION (1d) single tail segment (deltaV == delta_vmax)
        // Only possible if the entry has not changed since hinting.
        else if (bf->hint == PERFECT_DECELERATION) 
        {
            block->tail_length     = bf->length;
            block->cruise_velocity = entry_velocity;
            block->tail_time       = block->tail_length * 2 / (block->exit_velocity + block->cruise_velocity);
            bf->block_time         = block->tail_time;
            return (_ramp_exit_logger(bf, "1d"));
        }

        // Reset entry_changed. We won't likely be changing the next block's entry velocity.
        mp->entry_changed = false;
        // Since we are not generally decelerating, this is effectively all of forward planning that we need.
    } 
    else 
    {
        // Note that the hints from back-planning are ignored in this section, since back-planing can only
        // predict decelerations and cruises
        float accel_velocity = mp_get_target_velocity(entry_velocity, bf->length, bf);

        if (accel_velocity < block->exit_velocity) 
        {  // still accelerating
            mp->entry_changed = true;  // we are changing the *next* block's entry velocity
            block->exit_velocity   = accel_velocity;
            block->cruise_velocity = accel_velocity;
            bf->hint = PERFECT_ACCELERATION;

            // PERFECT_ACCELERATION (1a) single head segment (deltaV == delta_vmax)
            block->head_length     = bf->length;
            block->cruise_velocity = block->exit_velocity;
            block->head_time       = (block->head_length * 2.0) / (entry_velocity + block->cruise_velocity);
            bf->block_time         = block->head_time;
            return (_ramp_exit_logger(bf, "1a"));
        }
        else 
        {  // it's hit the cusp
            mp->entry_changed = false;  // we are NOT changing the next block's entry velocity
            block->cruise_velocity = bf->cruise_vmax;
            if (block->cruise_velocity > block->exit_velocity) 
            {
                // We will likely have a head section, so hint the move as an ASYMMETRIC_BUMP
                bf->hint = ASYMMETRIC_BUMP;
            } 
            else 
            {
                // We know that exit_velocity is higher than cruise_vmax, so adjust it
                block->exit_velocity = bf->cruise_vmax;
                bf->hint = MIXED_ACCELERATION;
                // MIXED_ACCELERATION (2a) 2 segment HB acceleration move
                // watch out for acceleration taking more than length
                block->head_length = mp_get_target_length(entry_velocity, block->cruise_velocity, bf);
                block->body_length = bf->length - block->head_length;
                debug_trap_if_true((block->body_length < 0), "invlaid negative body_length from MIXED_ACCELERATION");

                block->tail_length = 0;  // we just set it, now we unset it
                block->head_time   = (block->head_length * 2.0) / (entry_velocity + block->cruise_velocity);
                block->body_time   = block->body_length / block->cruise_velocity;
                bf->block_time     = block->head_time + block->body_time;
                return (_ramp_exit_logger(bf, "2a"));
            }
        }
    }

    // We've eliminated the following at this point:

    // PERFECT_ACCELERATION
    // MIXED_ACCELERATION
    // PERFECT_DECELERATION
    // MIXED_DECELERATION

    // All that remains is ASYMMETRIC_BUMP and SYMMETRIC_BUMP.
    // We don't really care if it's symmetric, since the first test that _get_meet_velocity
    //  does is for a symmetric move. It's cheaper to just let it do that then to try and prevent it.

    // *** Requested-Fit cases (2) ***

    // Prepare the head and tail lengths for evaluating cases (nb: zeros head / tail < min length)
    block->head_length = mp_get_target_length(entry_velocity, block->cruise_velocity, bf);
    block->tail_length = mp_get_target_length(block->exit_velocity, block->cruise_velocity, bf);

    if ((bf->length - 0.0001) > (block->head_length + block->tail_length)) 
    {
        // 3 segment HBT move (2c) - either with a body or just a symmetric bump
        block->body_length = bf->length - (block->head_length + block->tail_length);  // body guaranteed to be positive
        block->head_time = (block->head_length * 2.0) / (entry_velocity + block->cruise_velocity);
        block->body_time = block->body_length / block->cruise_velocity;
        block->tail_time = (block->tail_length * 2.0) / (block->exit_velocity + block->cruise_velocity);
        bf->block_time   = block->head_time + block->body_time + block->tail_time;
        bf->hint = ASYMMETRIC_BUMP;
        return (_ramp_exit_logger(bf, "2c"));
    }

    // *** Rate-Limited-Fit cases (3) ***
    // This means that bf->length < (bf->head_length + bf->tail_length)

    // Rate-limited asymmetric cases (3)
    // compute meet velocity to see if the cruise velocity rises above the entry and/or exit velocities
    block->cruise_velocity = _get_meet_velocity(entry_velocity, block->exit_velocity, bf->length, bf, block);
    debug_trap_if_zero(block->cruise_velocity, "mp_calculate_ramps() Vc=0 asymmetric HT case");

    // We now store the head/tail lengths we computed in _get_meet_velocity.
    // treat as a full up and down (head and tail)
    bf->hint = ASYMMETRIC_BUMP;

    // Compute move times

    // save a few divides where we can
    if (fp_NOT_ZERO(block->head_length)) 
    {
        block->head_time = (block->head_length * 2.0) / (entry_velocity + block->cruise_velocity);
    }
    if (fp_NOT_ZERO(block->body_length)) 
    {
        block->body_time = block->body_length / block->cruise_velocity;
    }
    if (fp_NOT_ZERO(block->tail_length)) 
    {
        block->tail_time = (block->tail_length * 2.0) / (block->exit_velocity + block->cruise_velocity);
    }
    bf->block_time = block->head_time + block->body_time + block->tail_time;
    return (_ramp_exit_logger(bf, "3c"));  // 550us worst case
}


/**** Planner helpers ****
 *
 * mp_get_target_length()   - find accel/decel length from delta V and jerk
 * mp_get_target_velocity() - find velocity achievable from Vi, length and jerk
 * _get_target_length_min() - find target length with correction for minimum length moves
 * _get_meet_velocity()     - find velocity at which two lines intersect
 *
 *  The get_target functions know 3 things and return the 4th:
 *    Jm = maximum jerk of the move
 *    T  = time of the entire move
 *    Vi = initial velocity
 *    Vf = final velocity
 */

/*
 * mp_get_target_length()   - find accel/decel length from delta V and jerk
 */

// Just calling this tl_constant. It's full name is:
// static const float tl_constant = 1.201405707067378;      // sqrt(5)/( sqrt(2)pow(3,4) )

float mp_get_target_length(const float v_0, const float v_1, const mpBuf_t* bf) 
{
    const float q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;
    return q_recip_2_sqrt_j * sqrt(std::abs(v_1 - v_0)) * (v_1 + v_0);
}

/*
 * mp_get_target_velocity() - find the velocity we would achieve if we accelerated from v_0
 *
 * Get the velocity that we would end up at if we accelerated from v_0
 * over the provided L (length) and J (jerk, provided in the bf structure)
 */

// 14 *, 1 /, 1 sqrt, 1 cbrt
// time: 68 us
float mp_get_target_velocity(const float v_0, const float L, const mpBuf_t* bf) 
{
    if (fp_ZERO(L)) {  // handle exception case
        return (0);
    }

    const float j = bf->jerk;

    const float a80 = 7.698003589195;       // 80 * a
    const float a_2 = 0.00925925925926;     // a^2

    const float v_0_2 = v_0 * v_0;          // v_0^2
    const float v_0_3 = v_0_2 * v_0;        // v_0^3

    const float L_2 = L * L;                // L^2

    const float b_part1 = 9 * j * L_2;      // 9 j L^2
    const float b_part2 = a80 * v_0_3;      // 80 a v_0^3

    //              b^3 = a^2 (3 L sqrt(j (2 b_part2  +  b_part1))  +  b_part2  +  b_part1)
    const float b_cubed = a_2 * (3 * L * sqrt(j * (2 * b_part2 + b_part1)) + b_part2 + b_part1);
    const float b       = cbrtf(b_cubed);

    const float const1a = 0.8292422988276;    // 4 * 10^(1/3) * a
    const float const2a = 4.823680612597;     // 1/(10^(1/3) * a)
    const float const3  = 0.333333333333333;  // 1/3

    //          v_1 =    1/3 ((const1a v_0^2)/b  +  b const2a  -  v_0)
    const float v_1 = const3 * ((const1a * v_0_2) / b + b * const2a - v_0);

    return std::abs(v_1);
}

/*
 * mp_get_decel_velocity() - mp_get_target_velocity but ONLY for deceleration
 *
 *  Get the velocity that we would end up at if we decelerated from v_0,
 *  over the provided L (length) and J (jerk, provided in the bf structure).
 *
 *  We have to use a root finding solution, since there are actually three possible
 *  solutions. We can eliminate one quickly, since it's the acceleration case.
 *
 *  The other two cases are occasionally the same value, but this is rare.
 *  Otherwise there is a "high" value and a "low" value. The low value can be
 *  negative and then eliminated.
 *
 *  This function may generate minor errors in target velocity, and should only
 *  be used to compute feedholds or other cases where exact velocity is not mandatory. 
 *
 *  This function can fail if the length is too short to get a good answer. 
 *  Failures return (float)-1.0  Negative velocities should never be returned.
 */

float mp_get_decel_velocity(const float v_0, const float L, const mpBuf_t* bf) 
{
    const float q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;
    float v_1 = 0;              // start the guess at zero

    int i = 0;                  // limit the iterations
    while (i++ < 20) 
    {   // If it fails after 20 iterations something's wrong

        // l_t is the difference in length between the L provided and the current guessed deceleration length
        const float sqrt_delta_v_0 = sqrt(v_0 - v_1);
        const float l_t = q_recip_2_sqrt_j * (sqrt_delta_v_0 * (v_1 + v_0)) - L;

        // The return condition allows a minor error in length (in mm). 
        // Note: This comparison does NOT affect actual lengths or steps, which would be bad.
        //       The actual lengths traveled must be controlled by the caller.
        if (std::abs(l_t) < 0.001) 
        {
            break;
        }
        // For the first pass we tested velocity 0. If velocity 0 yields a l_t > 0, 
        // then we need to start searching at v_1 instead. 
        // (We can't start AT v_1, so we start at v_1 - 0.1)
        if (i==1 && (l_t > 0)) 
        {
            v_1 = v_0 - 0.1;
            continue;
        }
        const float v_1x3 = 3 * v_1;
        const float recip_l_t = (2 * sqrt_delta_v_0) / ((v_0 - v_1x3) * q_recip_2_sqrt_j);
        v_1 = v_1 - (l_t * recip_l_t);
        
        // In some extreme cases there is no solution because the length is too short
        if (v_1 > v_0) 
        {
            return (-1.0);    // cannot decelerate. Return an error
        }
    }
    return v_1;
}

//Is there a way to derive the average slope of a deceleration given the starting velocity, length and jerk? We don't need the

/*
 * _get_meet_velocity() - find intersection velocity
 *
 * This function, when given two velocities (v_0 and v_2) along with a length (L)
 * and jerk (J), will locate the velocity v_1 that will allow acceleration from v_0
 * at jerk J to v_1 and then deceleration at jerk J to v_2, all over total length L.
 *
 */

static float _get_meet_velocity(const float          v_0,
                                const float          v_2,
                                const float          L,
                                mpBuf_t*             bf,
                                mpBlockRuntimeBuf_t* block) 
{
    const float q_recip_2_sqrt_j = bf->q_recip_2_sqrt_j;

    // v_1 can never be smaller than v_0 or v_2, so we keep track of this value
    const float min_v_1 = max(v_0, v_2);

    // v_1 is our estimated return value.
    // We estimate with the speed obtained by L/2 traveled from the highest speed of v_0 or v_2.
    float v_1 = mp_get_target_velocity(min_v_1, L / 2.0, bf);
    // var v_1 = min_v_1 + 100;

    if (fp_EQ(v_0, v_2)) 
    {
        // Case (1)
        // We can catch a symmetric case early and return now
        // We'll have a head roughly equal to the tail, and no body
        block->head_length = L / 2.0;
        block->body_length = 0;
        block->tail_length = L - block->head_length;
        SET_PLANNER_ITERATIONS(-1);     // DIAGNOSTIC
        return v_1;
    }

    // Per iteration: 2 sqrt, 2 abs, 6 -, 4 +, 12 *, 3 /
    int i = 0;          // limit the iterations // 466us - 644us
    while (i++ < 30) 
    {  // If it fails after 30, something's wrong
        if (v_1 < min_v_1) 
        {
            // Case (2)
            // We have caught a rather nasty problem. There is no meet velocity.
            // This is due to an inversion in the velocities of very short moves.
            // We need to compute the head OR tail length, and the body will be the rest.
            // Yes, that means we're computing a cruise in here.

            v_1 = min_v_1;

            if (v_0 < v_2) 
            {
                // acceleration - it'll be a head/body
                block->head_length = mp_get_target_length(v_0, v_2, bf);
                if (block->head_length > L) 
                {
                    block->head_length = L;
                    block->body_length = 0;
                    v_1 = mp_get_target_velocity(v_0, L, bf);
                } 
                else 
                {
                    block->body_length = L - block->head_length;
                }
                block->tail_length = 0;
            } 
            else 
            {
                // deceleration - it'll be tail/body
                block->tail_length = mp_get_target_length(v_2, v_0, bf);
                if (block->tail_length > L) 
                {
                    block->tail_length = L;
                    block->body_length = 0;
                    v_1 = mp_get_target_velocity(v_2, L, bf);
                } 
                else 
                {
                    block->body_length = L - block->tail_length;
                }
                block->head_length = 0;
            }
            break;
        }

        // Precompute some common chunks -- note that some attempts may have v_1 < v_0 or v_1 < v_2
        const float sqrt_delta_v_0 = sqrt(std::abs(v_1 - v_0));
        const float sqrt_delta_v_2 = sqrt(std::abs(v_1 - v_2));  // 849us

        // l_c is our total-length calculation with the current v_1 estimate, minus the expected length.
        // This makes l_c == 0 when v_1 is the correct value.

        // GAMBLE: At the cost of one more multiply per iteration, we will keep the two length calculations separate.
        // This allows us to store the resulting head/tail lengths.
        const float l_h = q_recip_2_sqrt_j * (sqrt_delta_v_0 * (v_1 + v_0));
        const float l_t = q_recip_2_sqrt_j * (sqrt_delta_v_2 * (v_1 + v_2));
        const float l_c = (l_h + l_t) - L;

        block->head_length = l_h;
        block->tail_length = l_t;
        block->body_length = 0;

        // We need this level of precision, or our length computations fail to match the block length.
        // What we really want to ensure is that the two lengths down add up to be too much.
        // We can be a little under (and have a small body).

        // TODO: make these tunable
        if ((l_c < 0.00001) && (l_c > -1.0)) 
        {  // allow 0.00001 overlap, OR up to a 1mm gap
            if (l_c < 0.0) 
            {
                // Case (3a)
                block->body_length = -l_c;
            } 
            else 
            {
                // Case (3b)
                // fix the overlap
                block->tail_length = L - block->head_length;
            }
            break;
        }

        const float v_1x3     = 3 * v_1;
        const float recip_l_d = (2 * sqrt_delta_v_0 * sqrt_delta_v_2) /
                                ((sqrt_delta_v_0 * (v_1x3 - v_2) - (v_0 - v_1x3) * sqrt_delta_v_2) * q_recip_2_sqrt_j);

        v_1 = v_1 - (l_c * recip_l_d);
    }
    SET_MEET_ITERATIONS(i);     // DIAGNOSTIC
    return v_1;
}
