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
#define IMPACT_SLEWRATE_THRESHOLD 1024 // Difference between 2 samples TODO: This is just an example.
#define IMPACT_STRENGTH_THRESHOLD 2048      // TODO: Use 7-bit values or 8192 max ???
#define FREE_FALL_THRESHOLD 640             // TODO: Use 7-bit values or 8192 max ???
#define RATING_THRESHOLD 5                  // TODO: This is just an example - modify it appropriately.


// *************************************************************************************************
// Global Variable section
struct accel
{
	// ACC_MODE_OFF, ACC_MODE_ON
	u8          mode;

    // Temporary buffer for acceleration data
    u16         data;

	// Timeout
	u16         timeout;
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
