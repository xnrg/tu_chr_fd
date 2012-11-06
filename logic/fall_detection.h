// *************************************************************************************************

#ifndef FALL_DETECTION_H_
#define FALL_DETECTION_H_


// *************************************************************************************************
// Include section


// *************************************************************************************************
// Prototypes section



// *************************************************************************************************
// Defines section

#define ACCEL_MODE_OFF      (0u)
#define ACCEL_MODE_ON       (1u)

// Stop acceleration measurement after 60 minutes to save battery
#define ACCEL_MEASUREMENT_TIMEOUT       (60*60u)

// Fall detection defines
#define ACC_SAMPLING_RATE 40
#define FALL_DETECTION_WINDOW_IN_SECONDS 4
#define FALL_DETECTION_WINDOW_IN_SAMPLES (FALL_DETECTION_WINDOW_IN_SECONDS * ACC_SAMPLING_RATE)
#define FREE_FALL_BACKTRACK_IN_SECONDS 1
#define FREE_FALL_BACKTRACK_IN_SAMPLES (FREE_FALL_BACKTRACK_IN_SECONDS * ACC_SAMPLING_RATE)
#define MAX_IMPACT_LENGTH_SAMPLES ACC_SAMPLING_RATE
#define MAX_MOTIONLESSNESS_SAMPLES 2*ACC_SAMPLING_RATE
// TODO: Use 7-bit values or 8192 max ???
#define IMPACT_SLEWRATE_THRESHOLD 16    /*1024*/    // Difference between 2 samples (equals around 1G)
#define IMPACT_STRENGTH_THRESHOLD 32    /*2048*/
#define FREE_FALL_THRESHOLD 8           /*640*/     // This is the average of 40 samples of free falling
#define MOTIONLESSNESS_THESHOLD 40      /*2840*/    // This is the sum of the deltas between 80 samples of motionless
#define RATING_THRESHOLD 5                          // TODO: This is just an example - modify it appropriately.


// *************************************************************************************************
// Global Variable section
struct peaks
{
    // Storage for the peaks addresses
    u16 * peaksBuff;

    // Sample index
    u8 index;
};

struct accel
{
	// ACC_MODE_OFF, ACC_MODE_ON
	u8          mode;

    // Temporary buffer for acceleration data
    u16         data;
};
extern struct accel sAccel;


// *************************************************************************************************
// Extern section
extern void reset_acceleration(void);
extern void sx_fall_detection(u8 line);
extern void display_fall_detection(u8 line, u8 update);
extern u8 is_acceleration_measurement(void);
extern void do_fall_detection(void);

#endif /*FALL_DETECTION_H_*/
