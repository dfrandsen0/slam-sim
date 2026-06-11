#ifndef CONFIG_H_
#define CONFIG_H_

#include <utility>
#include <cmath>


// Feel free to organize this differently.
// Currently split into:
// - Controls
// - Required definitions
// - Feel free to change any of these
// - Changeable; but be wary
// - Change at your own risk
// - Do not touch any parameters here!

////////////////////////////////////////////////
////                Controls                ////
////////////////////////////////////////////////

// Click top right: send robot appropriate bearing/heading to move robot
// Spacebar: pause/unpause (note, does not pause threads, though they will run at correct periods when unpaused again)

////////////////////////////////////////////////
////          Required definitions          ////
////////////////////////////////////////////////

// Do not change any of these. Required definitions for parameters lower down.


// Possible SLAM options
#define SLAM_OPTION_EKF 0
#define SLAM_OPTION_GMAPPING 1

// AI/Motion Commands
enum class RobotCommand : int {
    FORWARD,
    STOP,
    LEFT,
    RIGHT
};

////////////////////////////////////////////////
////    Feel free to change any of these    ////
////////////////////////////////////////////////

// Change these all you want. Be ye cautioned: changing one thing will sometimes
//   affect other things. Often in weird, unexpected ways. Applicable Defaults
//   have been put in comments.

const int STARTING_MAP = 3;

// Optional Toggles
const bool SHOW_POSSIBLE_STARTING_LOCATIONS = false;
const bool SHOW_PARTICLES_TOP_LEFT = true;
const bool HIGHLIGHT_STRONGEST_POSE_BOTTOM_RIGHT = true;
const bool ACCOUNT_FOR_MOTION_BLUR = true;  // Just affects algorithm; sensor attempts to account regardless (you will note a small difference)
const bool START_AI_WITH_STOP = true;       // AI starts with a single STOP command.
const bool HARD_OVERRIDE_COMMANDS = false;  // Toggle true to override all commands with a constant value, OVERRIDE
const RobotCommand OVERRIDE = RobotCommand::LEFT;
const bool RANDOM_START = true;

// Sensor Model Details; moves in 360 degrees
const int SENSOR_MODEL_POINTS_PER_SCAN = 1455;       // 1455 (8000 / 5.5Hz)
const double SENSOR_MODEL_TIME_PER_SCAN = 1.0 / 5.5; // 5.5 Hz (1/5.5)
const int SENSOR_MODEL_MEASURE_MIN = 150;            // 150 mm
const int SENSOR_MODEL_MEASURE_MAX = 12000;          // 12000 mm

// Percent noise within distance x in mm.
// Defaults based on lidar specs {1% <= 3000, 2% 3-5, 2.5% 5+}. Be sure to cover measurement range.
const int SENSOR_MODEL_ACCURACY_TIERS = 3;
const std::pair<double, double> SENSOR_MODEL_ACCURACY[SENSOR_MODEL_ACCURACY_TIERS] = {
    {0.01, 3000},
    {0.02, 5000},
    {0.025, 25000}
};

// Default based on lidar specs is 1% < 12 meters. For simplicity, one range resolution for all distances
//   all measurements outside that range get rejected anyway.
const double SENSOR_MODEL_RANGE_RESOLUTION = 0.01;

// Period details (except sensor, which is given by SENSOR_MODEL_TIME_PER_SCAN)
const double GRAPHICS_FPS = 1.0 / 60.0;                                            // (1/60) How often are graphics updated? Also can affect how often motion is updated
const double MOTION_PERIOD = GRAPHICS_FPS;                                         // (1/60) No point having it faster than graphics; but ideally as fast as possible
const int SLAM_MINIMUM_PERIOD_COUNT = std::ceil(60 * SENSOR_MODEL_TIME_PER_SCAN);  // (11) At least how many frames must pass before the algorithm considers running (num slam_period). Consider keeping at a minimum of scan period (10.9, or 11)
const int SLAM_MAXIMUM_PERIOD_COUNT = 1/GRAPHICS_FPS;                              // (60) The maximum frames that can pass before the algorithm must run
const double SLAM_MINIMUM_DISTANCE_PERCENT = 0.8;                                  // (0.8) Percentage of maximum movement within minimum period count (rotation scaled appropriately);
                                                                                   //   helper equation defined lower as SLAM_MINIMUM_DISTANCE (0.8 == 80%; 1.5 == 150%)


// Motion Model Details (suggested noise below in multi-line comment)
const double MOTION_MODEL_ACCELERATION = 150.0 * MOTION_PERIOD; // 150 mm/s^2, converted to period amount
const double MOTION_MODEL_MAX_VELOCITY = 300.0 * MOTION_PERIOD; // 300 mm/s, converted to period amount

const double MOTION_MODEL_ROTATION = 1.0 * MOTION_PERIOD; // 1 rad/s, or 57.2 degrees per second
const double MOTION_MODEL_FORWARD_DEVIATION = 0.175; //0.05; //5%: hardwood floor
const double MOTION_MODEL_FORWARD_ROTATION_DEVIATION = 0.025 * std::sqrt(MOTION_MODEL_MAX_VELOCITY / 1000); //0.02 radians per meter, growing proportional to distance
const double MOTION_MODEL_ROTATION_DEVIATION = 0.075; //0.02; //2%: hardwood floor
const double MOTION_MODEL_ROTATION_FIXED = MOTION_MODEL_ROTATION_DEVIATION * std::sqrt(MOTION_PERIOD); // motion period / 1.0, computational speedup

// I found Gmapping did better with a minimum sigma (one, because the math requires it, and two, it genuinely does better).
//   MUST be above 0.
const double MOTION_MODEL_BASE_ROTATION_DEVIATION = 0.25; // (0.25) local searched guesses that worked out, could potentially be defined proportional to rotational deviation
const double MOTION_MODEL_BASE_FORWARD_DEVIATION = 0.25; // (0.25)  //^^

/*** suggested standard deviations ***
 * 
 * Represented in code as a double: 0.1 == 10%.
 * 
 * Translation
 *   0-1% (heaven)
 *   2-5% (hard floor)
 *   10-15% (low pile)
 *   15-20% (medium pile)
 *   20-40% (high pile)
 * 
 *  Rotation when moving forward between 0.02 and 0.03 rad/m
 *  Why so small? 9/10 expert engineers say:
 *       "If it's worse than that, seriously, get better wheels dude."
 *  (It can do very, very poorly with more than that. Mostly because your robot
 *   literally cannot drive to save it's life. Let alone figure out where it is.)
 * 
 * Rotation
 *   0% (heaven)
 *   1-2% (hardwood floor)
 *   3-5% (low pile carpet)
 *   5-10% (medium pile carpet)
 *   10-15% (high pile carpet)
 * 
 * */

 // Gmapping Configurations
const int GMAPPING_NUM_PARTICLES = 30;              // (30) Num particles; paper suggested 30-40, though another source suggested 100... seemed a little overkill.
const double GMAPPING_GRID_CELL_SIZE = 50;          // (50) mm (min 40, max tested was 500 haha)
const int GMAPPING_SECTOR_SIZE = 16;                // (16) Each sector is 16 x 16 cells

const double GMAPPING_MAX_LOG_ODDS = 5.0;           // (5) Occupancy Grid's capped at +- this much
const double GMAPPING_LOG_ODDS_HIT = 0.85;          // (0.85) This one was really hard to find. Theoretically, these could be based on distance. (sensor noise gets worse with distance idea)
const double GMAPPING_LOG_ODDS_MISS = -0.2;         // (-0.2) Also hard to find, was suggested -0.4--0.5, but I found -0.2 to be better. Though I'd even more strongly consider basing off distance (and not hits at all, or only affect misses before a certain threshold distance)
const double GMAPPING_LOG_ODDS_WALL_VALUE = 1.0;    // (1.0) What must log odds be to be treated as "wall" in scan matching. Log odds of 1.0 represents 73% likely  (p = e^L / (1 + e^L))

const double GMAPPING_SCAN_MATCHING_PERCENT_LASERS_USED = 0.3; // (0.3) (though 0.1 and 0.2 were suggested in papers) We don't use every laser when scan matching in gmapping, for 2 reasons: (1) computationally expensive, (2) changes how "peaky" a scan is. More beams == bigger differences between two slight nudges. Given as percentage: 1 in every 5 would be 0.2
const double GMAPPING_SCAN_MATCHING_MAX_DIST = 10 * GMAPPING_GRID_CELL_SIZE*GMAPPING_GRID_CELL_SIZE; // (10) The max number of cells as distance as "matched" in scan matching. A wall further than this is considered infinitely far
const double GMAPPING_SCAN_MATCHING_MAX_NUDGES = 3; // (3) The max number of nudges each particle undergoes before sampling

const double GMAPPING_MINIMUM_NEFF = 0.5;           // (0.5) what minimum percentage of particles must be "pulling their weight" at any given time

// Normally, Gmapping uses partial derivatives to "map" the curve of the likelihood field.
//   I've never done the math behind partial derivatives, and didn't want to learn. So I
//   just made a basic "jump" (translation) and "twist" (rotation) amount based on noise. Default was half of max noise given max velocity for minimum period. (At defaults, that's (5 * 11) * sigma / 2)
//   Works in conjunction with GMAPPING_SCAN_MATCHING_MAX_NUDGES. If too much moving, will "snap" to a position with little variety. Too little causes crappy particles.
const double GMAPPING_SCAN_MATCHING_NUDGE_JUMP = (MOTION_MODEL_FORWARD_DEVIATION * MOTION_MODEL_MAX_VELOCITY * SLAM_MINIMUM_PERIOD_COUNT) / 2.0;
const double GMAPPING_SCAN_MATCHING_NUDGE_TWIST = (MOTION_MODEL_ROTATION_DEVIATION * MOTION_MODEL_ROTATION * SLAM_MINIMUM_PERIOD_COUNT) / 2.0;



////////////////////////////////////////////////
////        Changeable; but be wary        /////
////////////////////////////////////////////////

// These are changeable if you take the time to figure out exactly what they do.
//   Some are untested (screen width/height). Best to see what each one is
//   in the code before changing.

// Graphics
// Screen Size
// Different screens render differently. Currently, average projectors and my laptop like these specs.
//   4k monitors would hate it.
const int CLIENT_SCREEN_WIDTH = 1280; // (1280)
const int CLIENT_SCREEN_HEIGHT = 720; // (720)

// Map Specs (max lines/starts within a map. If you make a map with more pieces than this,
//   you must increase these.)
const int MAX_MAP_LINES = 64;  // (64)
const int MAX_MAP_STARTS = 16; // (16)

// Colors; denoted by (colors x)
// To add brush/color, you must: 
//  1) increase size
//  2) add color to const ints
//  3) add that const int to value set
// Once added, you don't necessarily have to remove them. In fact, maybe don't remove them in case you forget if they're used.

//(colors 1)
const int COLOR_PALETTE_SIZE = 6;

//(colors 2)
const int COLOR_PALETTE_BACKGROUND = 0xe3e3e3; // Background lines, text, etc.
const int COLOR_PALETTE_RED = 0xFF0000;        // Like highlighted pose.
const int COLOR_PALETTE_WHITE = 0xFFFFFF;      // Not sure I use this. 
const int COLOR_PALETTE_BLACK = 0x000000;      // like best robot pose
const int COLOR_PALETTE_LIGHT_GRAY = 0xA0A0A0; // like robot reference bottom right
const int COLOR_PALETTE_BLUE = 0x0051FF;       // like particles

//(colors 3)
const int COLOR_PALETTE_VALUES[COLOR_PALETTE_SIZE] = {
    COLOR_PALETTE_BACKGROUND,
    COLOR_PALETTE_RED,
    COLOR_PALETTE_WHITE,
    COLOR_PALETTE_BLACK,
    COLOR_PALETTE_LIGHT_GRAY,
    COLOR_PALETTE_BLUE
};

// These are the shadings for the wall. The darker the more "sure", linearly, even though
//   it's measuring log odds, which is definitely not linear. If you change this, each
//   shade from top down is increasing confidence. earlier in the list == less confident.
const int COLOR_PALETTE_WALLS_SIZE = 10;
const unsigned int COLOR_PALETTE_WALLS[COLOR_PALETTE_WALLS_SIZE] = {
    0xFFE1E1E1, //1: 225
    0xFFC8C8C8, //2: 200
    0xFFAFAFAF, //3: 175
    0xFF969696, //4: 150
    0xFF7D7D7D, //5: 125
    0xFF646464, //6: 100
    0xFF4B4B4B, //7: 75
    0xFF323232, //8: 50
    0xFF191919, //9: 25
    0xFF000000 //10: 0
};

// Lines, Shapes, Sizes
const int BACKGROUND_LINE_WIDTH = 4; // (4) side lines
const int MAP_LINE_WIDTH = 2;        // (2) map lines as shown; in the "real world" (what it represents) walls are infinitely thin... which... is questionable....

// Text
inline const wchar_t* neffText = L"Neff: "; //text bottom left. If you change this, you'll need to change the offsets
const int neffOffset = 4;       // (4) offset neff word (shifts right (4) dips)
const int neffNumOffset = 38;   // (38) offset neff num (shifts right (38) dips)


////////////////////////////////////////////////
////        Change at your own risk        /////
////////////////////////////////////////////////

const double GMAPPING_SCAN_MATCHING_DEFAULT_SIGMA = 85.0; // (85) I honestly have no clue where this number came from. I can't remember. Change at your own risk. (In theory, it's not used... in theory.)

const int COLOR_WALL_BRUSH = 0xFFFFFFFF; // (0xFFFFFFFF) Fear made me make this. Fear and ignorance. Powerful combo. Maybe don't mess with this either.

// World in mm. Note that maps are defined in absolute positions, so if you make the width/height too small, some maps might crash.
const int WORLD_SIZE_METERS_WIDTH = 12000; // (12000) "Meters". It's actually mm. Silly, huh?
const int WORLD_SIZE_METERS_HEIGHT = 6750; // (6750) "Meters". It's actually mm.


const int ROBOT_REAL_RADIUS = 150;      // 150 mm. Affects both top left and bottom right graphics.
const int ROBOT_RADIUS = 8;             // (8) Affects graphics
const float LIDAR_POINT_RADIUS = 2.0;   // (2) Affects graphics

const float GMAPPING_PARTICLE_RADIUS = 3.0;          // (3) Affects graphics
const double GMAPPING_PARTICLE_POINTER_LENGTH = 6.0; // (6) Affects graphics. double type for cos/sin precision
const float GMAPPING_PARTICLE_POINTER_WIDTH = 2.0;   // (2) Affects graphics

const float GMAPPING_BIG_PARTICLE_RADIUS = 6.0;           // (6) Affects graphics
const double GMAPPING_BIG_PARTICLE_POINTER_LENGTH = 12.0; // (12) Affects graphics. double type for cos/sin precision
const float GMAPPING_BIG_PARTICLE_POINTER_WIDTH = 4.0;    // (4) Affects graphics

// Sensor
const int SENSOR_NUM_TRACKED_PACKETS = 4; // (4) How many scans does the sensor keep in it's rotating library at any given time



////////////////////////////////////////////////
////   Do not touch any parameters here!   /////
////////////////////////////////////////////////

const double MOTION_MODEL_ROTATION_AMP = MOTION_MODEL_MAX_VELOCITY / MOTION_PERIOD; // For minimum movement, affects how much velocity affects min distance to make comparable to distance
const double SLAM_MINIMUM_DISTANCE = SLAM_MINIMUM_DISTANCE_PERCENT * (SLAM_MINIMUM_PERIOD_COUNT * MOTION_MODEL_MAX_VELOCITY); // must move 40 mm before running full algorithm (see rotation amp)

const int GMAPPING_SECTOR_NUM_CELLS = GMAPPING_SECTOR_SIZE*GMAPPING_SECTOR_SIZE; // Constant
const int GMAPPING_SECTOR_MM_SIZE = GMAPPING_SECTOR_SIZE*GMAPPING_GRID_CELL_SIZE; // Constant
const int GMAPPING_HISTORY_SIZE = SLAM_MAXIMUM_PERIOD_COUNT * 2; // total max history

const double MM_PER_DIP = WORLD_SIZE_METERS_WIDTH / (CLIENT_SCREEN_WIDTH / 2.0); // mm to dip conversion
const float GMAPPING_CELL_SIZE_DIPS = (float)(GMAPPING_GRID_CELL_SIZE / MM_PER_DIP); // cell size to dips conversion

// Quadrants
const int TOP_LEFT = 1;
const int TOP_RIGHT = 2;
const int BOTTOM_LEFT = 3;
const int BOTTOM_RIGHT = 4;

// Max distance infinity (squarable without overflow for the math)
const double GMAPPING_INF = 1e15;
const double GMAPPING_STARTING_WEIGHT = 1.0 / GMAPPING_NUM_PARTICLES; // basic belief weights
const double GMAPPING_NEFF_THRESHOLD = GMAPPING_MINIMUM_NEFF * GMAPPING_NUM_PARTICLES; // threshold neff; turns percentage defined in "paramters you can change" to useful code

const int STARTING_SLAM = SLAM_OPTION_GMAPPING; // The only implemented algorithm.

const double MOTION_MODEL_TIME_TO_STOP = (MOTION_MODEL_MAX_VELOCITY / MOTION_MODEL_ACCELERATION); // Physics
const double MOTION_MODEL_DISTANCE_TO_STOP = (MOTION_MODEL_MAX_VELOCITY*MOTION_MODEL_MAX_VELOCITY) / (2.0*MOTION_MODEL_ACCELERATION); // Physics

////////////////////////////////////////////////
////           Technically unused           ////
////////////////////////////////////////////////

// EKF Configurations
const double EKF_CLUSTERING_THRESHOLD = 150.0; // Points within x mm are considered same cluster; potentially same line/object
const int EKF_MIN_CLUSTER_SIZE = 10;           // Clusters smaller than this are discarded


// The end!

#endif