
/*
 * plan_arc.c - arc planning and motion execution
 * This file is part of the g2core project
 *
 * Copyright (c) 2010 - 2019 Alden S. Hart, Jr.
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
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
#include "g2_plan_arc.h"
#include "g2_planner.h"
#include "g2_util.h"

// Local functions
#define DISABLE_ARC_OFFSET_ERROR_ALARM //sme 8-5-2022 --allow to keep running for mini-mill life testing
static stat_t _compute_arc(const bool radius_f);
static void _compute_arc_offsets_from_radius(void);
static float _estimate_arc_time (void);
static stat_t _test_arc_soft_limits(void);

/*****************************************************************************
 * Canonical Machining arc functions (arc prep for planning and runtime)
 *
 * cm_arc_init()     - initialize arcs
 * cm_arc_feed()     - canonical machine entry point for arc
 * cm_arc_callback() - main-loop callback for arc generation
 * cm_abort_arc()    - stop an arc in process
 */

/*
 * cm_arc_init() - initialize arc structures
 */
void cm_arc_init(cmMachine_t *_cm)
{
    _cm->arc.magic_start = MAGICNUM;
    _cm->arc.magic_end = MAGICNUM;
}

/*
 * cm_abort_arc() - stop arc movement without maintaining position
 *
 *  OK to call if no arc is running
 */

void cm_abort_arc(cmMachine_t *_cm)
{
    _cm->arc.run_state = BLOCK_INACTIVE;
}

/*
 * cm_arc_callback() - generate an arc
 *
 *  cm_arc_cycle_callback() is called from the controller main loop. Each time it's called
 *  it queues as many arc segments (lines) as it can before it blocks, then returns.
 */

stat_t cm_arc_callback(cmMachine_t *_cm)
{
    if (_cm->arc.run_state == BLOCK_INACTIVE) {
        return (STAT_NOOP);
    }
    if (mp_planner_is_full(mp)) {
        return (STAT_EAGAIN);
    }
    _cm->arc.theta += _cm->arc.segment_theta;
    _cm->arc.gm.target[_cm->arc.plane_axis_0] = _cm->arc.center_0 + sin(_cm->arc.theta) * _cm->arc.radius;
    _cm->arc.gm.target[_cm->arc.plane_axis_1] = _cm->arc.center_1 + cos(_cm->arc.theta) * _cm->arc.radius;
    _cm->arc.gm.target[_cm->arc.linear_axis] += _cm->arc.segment_linear_travel;

    mp_aline(&(_cm->arc.gm));                            // run the line
    copy_vector(_cm->arc.position, _cm->arc.gm.target);   // update arc current position

    if (--(_cm->arc.segment_count) > 0) {
        return (STAT_EAGAIN);
    }
    _cm->arc.run_state = BLOCK_INACTIVE;
    return (STAT_OK);
}

/*
 * cm_arc_feed() - canonical machine entry point for arcs
 *
 * Generates an arc by queuing line segments to the move buffer. The arc is
 * approximated by generating a large number of tiny, linear segments.
 */

stat_t cm_arc_feed(const float target[], const bool target_f[],     // target endpoint
                   const float offset[], const bool offset_f[],     // IJK offsets
                   const float radius, const bool radius_f,         // radius if radius mode
                   const float P_word, const bool P_word_f,         // parameter
                   const bool modal_g1_f,                           // modal group flag for motion group
                   const cmMotionMode motion_mode)                  // defined motion mode
{
  volatile stat_t retval=STAT_OK;
    // Start setting up the arc and trapping arc specification errors

    // Trap some precursor cases. Since motion mode (MODAL_GROUP_G1) persists from the
    // previous move it's possible for non-modal commands such as F or P to arrive here
    // when no motion has actually been specified. It's also possible to run an arc as
    // simple as "I25" if CW or CCW motion mode was already set by a previous block.
    // Here are 2 cases to handle if CW or CCW motion mode was set by a previous block:
    //
    // Case 1: F, P or other non modal is specified but no movement is specified
    //         (no offsets or radius). This is OK: return STAT_OK
    //
    // Case 2: Movement is specified w/o a new G2 or G3 word in the (new) block.
    //         This is OK: continue the move
    //
    if ((!modal_g1_f) &&                                                // G2 or G3 not present
        (!(offset_f[AXIS_X] | offset_f[AXIS_Y] | offset_f[AXIS_Z])) &&  // no offsets are present
        (!radius_f)) {                                                  // radius not present
        return (STAT_OK);
    }

    // Some things you might think are errors but are not:
    //  - offset specified for linear axis (i.e. not one of the plane axes). Ignored
    //  - rotary axes are present. Ignored
    //  - no parameters are specified. This can happen when G2 or G3 motion mode persists but
    //    a non-arc, non-motion command is entered afterwards, such as M3 or T. Trapped here:

    // Trap if no parameters were specified while in CW or CCW arc motion mode. This is OK
    if (!(target_f[AXIS_X] | target_f[AXIS_Y] | target_f[AXIS_Z] |
          offset_f[AXIS_X] | offset_f[AXIS_Y] | offset_f[AXIS_Z] |
          radius_f | P_word_f)) {
        return(STAT_OK);
    }

    // trap missing feed rate
    if (fp_ZERO(cm->gm.feed_rate)) {
        return (STAT_FEEDRATE_NOT_SPECIFIED);
    }

    // Set the arc plane for the current G17/G18/G19 setting and test arc specification
    // Plane axis 0 and 1 are the arc plane, the linear axis is normal to the arc plane.
    if (cm->gm.select_plane == CANON_PLANE_XY) {         // G17 - the vast majority of arcs are in the G17 (XY) plane
        cm->arc.plane_axis_0 = AXIS_X;
        cm->arc.plane_axis_1 = AXIS_Y;
        cm->arc.linear_axis  = AXIS_Z;
    } else if (cm->gm.select_plane == CANON_PLANE_XZ) {  // G18
#ifdef REFACTOR_PLAN_ARC
        cm->arc.plane_axis_0 = AXIS_Z;
        cm->arc.plane_axis_1 = AXIS_X;
#else
        cm->arc.plane_axis_0 = AXIS_X;
        cm->arc.plane_axis_1 = AXIS_Z;
#endif
        cm->arc.linear_axis  = AXIS_Y;
    } else if (cm->gm.select_plane == CANON_PLANE_YZ) {  // G19
        cm->arc.plane_axis_0 = AXIS_Y;
        cm->arc.plane_axis_1 = AXIS_Z;
        cm->arc.linear_axis  = AXIS_X;
    } else {
#ifndef DISABLE_ARC_OFFSET_ERROR_ALARM 
        
        return(cm_panic(STAT_ACTIVE_PLANE_IS_MISSING, "cm_arc_feed() impossible value")); // plane axis has impossible value
#else
        return(STAT_ACTIVE_PLANE_IS_MISSING);
#endif
    }

    // test if no endpoints are specified in the selected plane
    cm->arc.full_circle = false;        // initial condition
    if (!(target_f[cm->arc.plane_axis_0] || target_f[cm->arc.plane_axis_1])) {
        if (radius_f) {             // in radius mode arcs missing both endpoints is an error
            return (STAT_ARC_AXIS_MISSING_FOR_SELECTED_PLANE);
        } else {
            cm->arc.full_circle = true; // in center format arc this specifies a full circle
        }
    }

    // test radius arcs for radius tolerance
    if (radius_f) 
    {
        cm->arc.radius = _to_millimeters(radius);           // set radius to internal format (mm)
        if (std::abs(cm->arc.radius) < MIN_ARC_RADIUS) {        // radius value must be > minimum radius
            return (STAT_ARC_RADIUS_OUT_OF_TOLERANCE);
        }
    }
    else 
    {  // test that center format absolute distance mode arcs have both offsets specified
        if (cm->gm.arc_distance_mode == ABSOLUTE_DISTANCE_MODE) 
        {
            if (!(offset_f[cm->arc.plane_axis_0] && offset_f[cm->arc.plane_axis_1])) 
            {  // if one or both offsets are missing
                retval=STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE;
                return (retval);//STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE);
            }
        }
    }

    // Set arc rotations using P word
    if (P_word_f) 
    {
        if (P_word < 0) {  // If P is present it must be a positive integer
            return (STAT_P_WORD_IS_NEGATIVE);
        }
        if (floor(P_word) - (P_word) > 0) 
        {
            return (STAT_P_WORD_IS_NOT_AN_INTEGER);
        }
        cm->arc.rotations = P_word;
    } 
    else 
    {
        if (cm->arc.full_circle) 
        {      // arc rotations default to 1 for full circles
            cm->arc.rotations = 1;
        } 
        else 
        {
            cm->arc.rotations = 0;      // no rotations
        }
    }

    // set values in the Gcode model state & copy it (linenum was already captured)
    cm_set_model_target(target, target_f);

    // in radius mode it's an error for start == end
    if (radius_f) 
    {
        if ((fp_EQ(cm->gmx.position[AXIS_X], cm->gm.target[AXIS_X])) &&
            (fp_EQ(cm->gmx.position[AXIS_Y], cm->gm.target[AXIS_Y])) &&
            (fp_EQ(cm->gmx.position[AXIS_Z], cm->gm.target[AXIS_Z]))) 
        {
            return (STAT_ARC_ENDPOINT_IS_STARTING_POINT);
        }
    }

    // *** now get down to the rest of the work setting up the arc for execution ***
    cm->gm.motion_mode = motion_mode;
    cm_set_display_offsets(MODEL);                        // capture the fully resolved offsets to gm
    memcpy(&(cm->arc.gm), MODEL, sizeof(GCodeState_t));   // copy GCode context to arc singleton - some will be overwritten to run segments
    copy_vector(cm->arc.position, cm->gmx.position);        // set initial arc position from gcode model

    // setup offsets if in center format mode
    if (!radius_f) 
    {
        cm->arc.ijk_offset[OFS_I] = _to_millimeters(offset[OFS_I]); // copy offsets with conversion to canonical form (mm)
        cm->arc.ijk_offset[OFS_J] = _to_millimeters(offset[OFS_J]);
        cm->arc.ijk_offset[OFS_K] = _to_millimeters(offset[OFS_K]);

        if (cm->arc.gm.arc_distance_mode == ABSOLUTE_DISTANCE_MODE) 
        {   // adjust offsets if in absolute mode
             cm->arc.ijk_offset[OFS_I] -= cm->arc.position[AXIS_X];
             cm->arc.ijk_offset[OFS_J] -= cm->arc.position[AXIS_Y];
             cm->arc.ijk_offset[OFS_K] -= cm->arc.position[AXIS_Z];
        }

        if ((fp_ZERO(cm->arc.ijk_offset[OFS_I])) &&                 // error if no offsets provided in center format mode
            (fp_ZERO(cm->arc.ijk_offset[OFS_J])) &&
            (fp_ZERO(cm->arc.ijk_offset[OFS_K])))
            {
#ifndef DISABLE_ARC_OFFSET_ERROR_ALARM                        
                return (cm_alarm(STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE, "arc offsets missing or zero"));
#else
                return(STAT_ARC_OFFSETS_MISSING_FOR_SELECTED_PLANE);
#endif            
            }
    }
#if 1//sme: 11-28-2022 allow err 154 to pass
    // compute arc runtime values
    retval=_compute_arc(radius_f);
    if (retval ==STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT)
    {
        retval = STAT_OK;//give this a pass for now
        __NOP();
    }
#else//original
    ritorno(_compute_arc(radius_f));
#endif

    // test arc soft limits
    stat_t status = _test_arc_soft_limits();
    if (status != STAT_OK) {
        cm->gm.motion_mode = MOTION_MODE_CANCEL_MOTION_MODE;
        copy_vector(cm->gm.target, cm->arc.position);       // reset model position
        return (cm_alarm(status, "arc soft_limits"));       // throw an alarm
    }

    cm_cycle_start();                                       // if not already started
    cm->arc.run_state = BLOCK_ACTIVE;                       // enable arc to be run from the callback
    cm_update_model_position();
    return (STAT_OK);
}

/*
 * _compute_arc() - compute arc from I and J (arc center point)
 *
 *  The theta calculation sets up an clockwise or counterclockwise arc from the current
 *  position to the target position around the center designated by the offset vector.
 *  All theta-values measured in radians of deviance from the positive y-axis.
 *
 *                    | <- theta == 0
 *                  * * *
 *                *      *
 *              *          *
 *              *    O ----T   <- theta_end (e.g. 90 degrees: theta_end == PI/2)
 *              *   /
 *                C   <- theta_start (e.g. -145 degrees: theta_start == -PI*(3/4))
 *
 *  Parts of this routine were informed by the grbl project.
 */

static stat_t _compute_arc(const bool radius_f)
{
    stat_t result = STAT_OK;
    // Compute IJK offsets and starting radius
    if (radius_f) {                         // indicates a radius arc
        _compute_arc_offsets_from_radius();
    } else {                                // compute start radius
        cm->arc.radius = hypotf(-cm->arc.ijk_offset[cm->arc.plane_axis_0], -cm->arc.ijk_offset[cm->arc.plane_axis_1]);
    }

    // Test arc specification for correctness according to:
    // https://github.com/synthetos/g2/wiki/Gcodes#g2-g3-arc-at-feed-rate
    // "It is an error if: when the arc is projected on the selected plane, the distance from
    //  the current point to the center differs from the distance from the end point to the
    //  center by more than (.05 inch/.5 mm) OR ((.0005 inch/.005mm) AND .1% of radius)."

    // Compute end radius from the center of circle (offsets) to target endpoint
    float end_0 = cm->arc.gm.target[cm->arc.plane_axis_0] -
                  cm->arc.position[cm->arc.plane_axis_0] -
                  cm->arc.ijk_offset[cm->arc.plane_axis_0];

    float end_1 = cm->arc.gm.target[cm->arc.plane_axis_1] -
                  cm->arc.position[cm->arc.plane_axis_1] -
                  cm->arc.ijk_offset[cm->arc.plane_axis_1];

    float err = std::abs(hypotf(end_0, end_1) - cm->arc.radius);   // end radius - start radius
    if ((err > ARC_RADIUS_ERROR_MAX) ||
       ((err > ARC_RADIUS_ERROR_MIN) && (err > cm->arc.radius * ARC_RADIUS_TOLERANCE))) 
    {
#if 1//sme: 11-28-2022 
        result = STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT;
#else//original
        return (STAT_ARC_HAS_IMPOSSIBLE_CENTER_POINT);
#endif
    }

    // Compute the angular travel
    // Calculate the theta angle of the current position (theta is also needed for calculating center point)
    // Note: gcc atan2 reverses args, i.e.: atan2(Y,X)
    cm->arc.theta = atan2(-cm->arc.ijk_offset[cm->arc.plane_axis_0], -cm->arc.ijk_offset[cm->arc.plane_axis_1]);

    // Compute angular travel if not a full circle arc
    if (!cm->arc.full_circle) {
        cm->arc.angular_travel = atan2(end_0, end_1) - cm->arc.theta; // travel = theta_end - theta_start

        // correct for atan2 output quadrants
        if (cm->arc.gm.motion_mode == MOTION_MODE_CW_ARC) {
            if (cm->arc.angular_travel <= 0) { cm->arc.angular_travel += 2*M_PI; }
        } else {
            if (cm->arc.angular_travel > 0)  { cm->arc.angular_travel -= 2*M_PI; }
        }
        // add in travel for rotations
        if (cm->arc.angular_travel >= 0) { cm->arc.angular_travel += 2*M_PI * cm->arc.rotations; }
        else                             { cm->arc.angular_travel -= 2*M_PI * cm->arc.rotations; }
    }
    // Compute full-circle arcs
    else {
        if (cm->arc.gm.motion_mode == MOTION_MODE_CCW_ARC) { cm->arc.rotations *= -1; }
        cm->arc.angular_travel = 2 * M_PI * cm->arc.rotations;
    }

    // Trap zero movement arcs
    if (fp_ZERO(cm->arc.angular_travel)) {
        return (STAT_ARC_ENDPOINT_IS_STARTING_POINT);
    }

    // Calculate travel in the plane and the depth axis of the helix
    // Length is the total mm of travel of the helix (or just the planar arc)
    cm->arc.linear_travel = cm->arc.gm.target[cm->arc.linear_axis] - cm->arc.position[cm->arc.linear_axis];
    cm->arc.planar_travel = cm->arc.angular_travel * cm->arc.radius;
    cm->arc.length = hypotf(cm->arc.planar_travel, std::abs(cm->arc.linear_travel));

    // Find the minimum number of segments that meet accuracy and time constraints...
    // Note: removed segment_length test as segment_time accounts for this (build 083.37)
    float arc_time = _estimate_arc_time();
    float segments_for_minimum_time =  arc_time * (MICROSECONDS_PER_MINUTE / MIN_ARC_SEGMENT_USEC);
    float segments_for_chordal_accuracy = cm->arc.length / sqrt(4*cm->chordal_tolerance * (2 * cm->arc.radius - cm->chordal_tolerance));
    cm->arc.segments = floor(min(segments_for_chordal_accuracy, segments_for_minimum_time));
    cm->arc.segments = max(cm->arc.segments, (float)1.0);        //...but is at least 1 segment

    if (cm->arc.gm.feed_rate_mode == INVERSE_TIME_MODE) {
        cm->arc.gm.feed_rate /= cm->arc.segments;
    }
    // setup the rest of the arc parameters
    cm->arc.segment_count = (int32_t)cm->arc.segments;
    cm->arc.segment_theta = cm->arc.angular_travel / cm->arc.segments;
    cm->arc.segment_linear_travel = cm->arc.linear_travel / cm->arc.segments;
    cm->arc.center_0 = cm->arc.position[cm->arc.plane_axis_0] - sin(cm->arc.theta) * cm->arc.radius;
    cm->arc.center_1 = cm->arc.position[cm->arc.plane_axis_1] - cos(cm->arc.theta) * cm->arc.radius;
    cm->arc.gm.target[cm->arc.linear_axis] = cm->arc.position[cm->arc.linear_axis];    // initialize the linear target
#if 1//sme: 11-28-2022 
     return result;
#else  
    return (STAT_OK);
#endif
}

/*
 * _compute_arc_offsets_from_radius() - compute arc center (offset) from radius.
 *
 *  Needs to calculate the center of the circle that has the designated radius and
 *  passes through both the current position and the target position
 *
 *  This method calculates the following set of equations where:
 *  `  [x,y] is the vector from current to target position,
 *      d == magnitude of that vector,
 *      h == hypotenuse of the triangle formed by the radius of the circle,
 *           the distance to the center of the travel vector.
 *
 *  A vector perpendicular to the travel vector [-y,x] is scaled to the length
 *  of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2]
 *  to form the new point [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the
 *  center of the arc.
 *
 *      d^2 == x^2 + y^2
 *      h^2 == r^2 - (d/2)^2
 *      i == x/2 - y/d*h
 *      j == y/2 + x/d*h
 *                                      O <- [i,j]
 *                                   -  |
 *                         r      -     |
 *                             -        |
 *                          -           | h
 *                       -              |
 *         [0,0] ->  C -----------------+--------------- T  <- [x,y]
 *                   | <------ d/2 ---->|
 *
 *      C - Current position
 *      T - Target position
 *      O - center of circle that pass through both C and T
 *      d - distance from C to T
 *      r - designated radius
 *      h - distance from center of CT to O
 *
 *  Expanding the equations:
 *      d -> sqrt(x^2 + y^2)
 *      h -> sqrt(4 * r^2 - x^2 - y^2)/2
 *      i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
 *      j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
 *
 *  Which can be written:
 *      i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
 *      j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
 *
 *  Which we for size and speed reasons optimize to:
 *      h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
 *      i = (x - (y * h_x2_div_d))/2
 *      j = (y + (x * h_x2_div_d))/2
 *
 * ----Computing clockwise vs counter-clockwise motion ----
 *
 *  The counter clockwise circle lies to the left of the target direction.
 *  When offset is positive the left hand circle will be generated -
 *  when it is negative the right hand circle is generated.
 *
 *                                 T  <-- Target position
 *
 *                                 ^
 *    Clockwise circles with       |     Clockwise circles with
 *      this center will have        |     this center will have
 *    > 180 deg of angular travel  |     < 180 deg of angular travel,
 *                      \          |      which is a good thing!
 *                       \         |         /
 *  center of arc when  ->  x <----- | -----> x <- center of arc when
 *  h_x2_div_d is positive           |             h_x2_div_d is negative
 *                                 |
 *                                 C  <-- Current position
 *
 *
 *  Assumes arc singleton has been pre-loaded with target and position.
 *  Parts of this routine were informed by the grbl project.
 */
static void _compute_arc_offsets_from_radius()
{
    // Calculate the change in position along each selected axis
    float x = cm->arc.gm.target[cm->arc.plane_axis_0] - cm->arc.position[cm->arc.plane_axis_0];
    float y = cm->arc.gm.target[cm->arc.plane_axis_1] - cm->arc.position[cm->arc.plane_axis_1];

    // *** From Forrest Green - Other Machine Co, 3/27/14
    // If the distance between endpoints is greater than the arc diameter, disc will be
    // negative indicating that the arc is offset into the complex plane beyond the reach
    // of any real CNC. However, numerical errors can flip the sign of disc as it approaches
    // zero (which happens as the arc angle approaches 180 degrees). To avoid mishandling
    // these arcs we use the closest real solution (which will be 0 when disc <= 0). This
    // risks obscuring g-code errors where the radius is actually too small (they will be
    // treated as half circles), but ensures that all valid arcs end up reasonably close
    // to their intended paths regardless of any numerical issues.
    float disc = 4 * square(cm->arc.radius) - (square(x) + square(y));

    // h_x2_div_d == -(h * 2 / d)
    float h_x2_div_d = (disc > 0) ? -sqrt(disc) / hypotf(x,y) : 0;

    // Invert the sign of h_x2_div_d if circle is counter clockwise (see header notes)
    if (cm->arc.gm.motion_mode == MOTION_MODE_CCW_ARC) {
        h_x2_div_d = -h_x2_div_d;
    }

    // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel"
    // (go figure!), even though it is advised against ever generating such circles in a
    // single Gcode block. By inverting the sign of h_x2_div_d the center of the circles is
    // placed on the opposite side of the line of travel and thus we get the inadvisably
    // long arcs as prescribed.
    if (cm->arc.radius < 0) {
        h_x2_div_d = -h_x2_div_d;
        cm->arc.radius *= -1;           // and flip the radius sign while you are at it
    }

    // Complete the operation by calculating the actual center of the arc
    cm->arc.ijk_offset[cm->arc.plane_axis_0] = (x-(y*h_x2_div_d))/2;
    cm->arc.ijk_offset[cm->arc.plane_axis_1] = (y+(x*h_x2_div_d))/2;
    cm->arc.ijk_offset[cm->arc.linear_axis] = 0;
}

/*
 * _estimate_arc_time ()
 *
 *  Returns a naiive estimate of arc execution time to inform segment calculation.
 *  The arc time is computed not to exceed the time taken in the slowest dimension
 *  in the arc plane or in linear travel. Maximum feed rates are compared in each
 *  dimension, but the comparison assumes that the arc will have at least one segment
 *  where the unit vector is 1 in that dimension. This is not true for any arbitrary arc,
 *  with the result that the time returned may be less than optimal.
 */
static float _estimate_arc_time (void)
{
  float arc_time=0;
    // Determine move time at requested feed rate
    if (cm->arc.gm.feed_rate_mode == INVERSE_TIME_MODE) {
        arc_time = cm->arc.gm.feed_rate;    // inverse feed rate has been normalized to minutes
    } else {
        arc_time = cm->arc.length / cm->gm.feed_rate;
    }

    // Downgrade the time if there is a rate-limiting axis
    arc_time = max(arc_time, (float)std::abs(cm->arc.planar_travel/cm->a[cm->arc.plane_axis_0].feedrate_max));
    arc_time = max(arc_time, (float)std::abs(cm->arc.planar_travel/cm->a[cm->arc.plane_axis_1].feedrate_max));
    if (std::abs(cm->arc.linear_travel) > 0) {
        arc_time = max(arc_time, (float)std::abs(cm->arc.linear_travel/cm->a[cm->arc.linear_axis].feedrate_max));
    }
    return (arc_time);
}

/*
 * _test_arc_soft_limits() - return error code if soft limit is exceeded
 *
 *  Test if arc extends beyond arc plane boundaries set in soft limits.
 *
 *  The arc starting position (P) and target (T) define 2 points that divide the
 *  arc plane into 9 rectangles. The center of the arc is (C). P and T define the
 *  endpoints of two possible arcs; one that is less than or equal to 180 degrees (acute)
 *  and one that is greater than 180 degrees (obtuse), depending on the location of (C).
 *
 *  -------------------------------  plane boundaries in X and Y
 *  |         |         |         |
 *  |    1    |    2    |    3    |
 *  |                   |         |
 *  --------- P -------------------
 *  |                   |         |
 *  |    4    |    5    |    6    |
 *  |         |                   |
 *  ------------------- T ---------
 *  |        C|                   |  C shows one of many possible center locations
 *  |    7    |    8    |    9    |
 *  |         |         |         |
 *  -------------------------------
 *
 *  C will fall along a diagonal bisecting 7, 5 and 3, but there is some tolerance in the
 *  circle algorithm that allows C to deviate from the centerline slightly. As the centerline
 *  approaches the line connecting S and T the acute arcs will be "above" S and T in sections
 *  5 or 3, and the obtuse arcs will be "below" in sections 5 or 7. But it's simpler, because
 *  we know that the arc is > 180 degrees (obtuse) if the angular travel value is > pi.
 *
 *  The example below only tests the X axis (0 axis), but testing the other axis is similar
 *
 *    (1) If Cx <= Px and arc is acute; no test is needed
 *
 *    (2) If Cx <= Px and arc is obtuse; test if the radius is greater than
 *          the distance from Cx to the negative X boundary
 *
 *    (3) If Px < Cx < Tx and arc is acute; test if the radius is greater than
 *          the distance from Cx to the positive X boundary
 *
 *    (4) If Px < Cx < Tx and arc is obtuse; test if the radius is greater than
 *          the distance from Cx to the positive X boundary
 *
 *  The arc plane is defined by 0 and 1 depending on G17/G18/G19 plane selected,
 *  corresponding to arc planes XY, XZ, YZ, respectively.
 *
 *  Must be called with all the following set in the arc struct
 *    -    arc starting position (arc.position)
 *    - arc ending position (arc.gm.target)
 *    - arc center (arc.center_0, arc.center_1)
 *    - arc.radius (arc.radius)
 *    - arc angular travel in radians (arc.angular_travel)
 *    - max and min travel in axis 0 and axis 1 (in cm struct)
 */
/*
static stat_t _test_arc_soft_limit_plane_axis(float center, uint8_t plane_axis)
{
    if (center <= arc.position[plane_axis]) {
        if (arc.angular_travel < M_PI) {                            // case (1)
            return (STAT_OK);
        }
        if ((center - arc.radius) < cm->a[plane_axis].travel_min) {    // case (2)
            return (STAT_SOFT_LIMIT_EXCEEDED);
        }
    }
    if ((center + arc.radius) > cm->a[plane_axis].travel_max) {        // cases (3) and (4)
        return (STAT_SOFT_LIMIT_EXCEEDED);
    }
    return(STAT_OK);
}
*/
static stat_t _test_arc_soft_limits()
{
/*
    if (cm->soft_limit_enable == true) {

        // Test if target falls outside boundaries. This is a 3 dimensional test
        // so it also checks the linear axis of the arc (helix axis)
        ritorno(cm_test_soft_limits(arc.gm.target));

        // test arc extents
        ritorno(_test_arc_soft_limit_plane_axis(arc.center_0, arc.plane_axis_0));
        ritorno(_test_arc_soft_limit_plane_axis(arc.center_1, arc.plane_axis_1));
    }
*/
    return(STAT_OK);
}
