// *************************************************************************************************
// Include section

// system
#include "project.h"

// driver
#include "buzzer.h"
#include "display.h"
#include "vti_as.h"
#include "ports.h"

// logic
#include "alarm.h"
#include "fall_detection.h"
#include "simpliciti.h"
#include "user.h"
#include <math.h>

// *************************************************************************************************
// Global Variable section
struct accel sAccel;

u16 fall_data[FALL_DETECTION_WINDOW_IN_SAMPLES];
u16 * readIndex = NULL;

// Conversion values from data to mgrav taken from CMA3000-D0x datasheet (rev 0.4, table 4)
const u16 mgrav_per_bit[7] = { 18, 36, 71, 143, 286, 571, 1142 };
// TODO: Implement use (#ifdef) of the 8g values and uncomment the 2 lines below
//const u16 mgrav_per_bit_2g[7] = { 18, 36, 71, 143, 286, 571, 1142 };
//const u16 mgrav_per_bit_8g[7] = { 71, 143, 286, 571, 1142, 2286, 4571 };

// *************************************************************************************************
// Extern section

// Global flag for proper acceleration sensor operation
extern u8 as_ok;



// *************************************************************************************************
// @fn          write_data_to_fifo_buffer
// @brief       Adds data to the FIFO buffer and removes old data if buffer is full.
// @param       u16 * buff_addr     - Pointer to FIFO buffer.
//              u16 data            - Data to be added to FIFO buffer.
// @return      none
// *************************************************************************************************
void write_data_to_fifo_buffer(u16 * buff_addr, u16 data)
{
    static u8 writeIndex = 0;

    if (writeIndex >= FALL_DETECTION_WINDOW_IN_SAMPLES) {
        writeIndex = 0;
    }
    *(u16 *)(buff_addr + writeIndex) = data;
    writeIndex++;

    readIndex = (u16 *)(buff_addr + writeIndex - 1);
}


// *************************************************************************************************
// @fn          read_data_from_fifo_buffer
// @brief       Reads data from the FIFO buffer with sample offset.
// @param       u16 * buff_addr - Address of FIFO buffer.
//              u8 backsamples  - Sample back offset.
// @return      u16 *           - Address of a previous sample.
// *************************************************************************************************
u16 * read_data_from_fifo_buffer(u16 * buff_addr, u8 backsamples)
{
    u8 offset;
    offset = (readIndex - buff_addr)/sizeof(u16);
    if (backsamples > offset) {
        return ((u16 *)(buff_addr + FALL_DETECTION_WINDOW_IN_SAMPLES + offset - backsamples));
    } else {
        return ((u16 *)(buff_addr + offset - backsamples));
    }
}


// *************************************************************************************************
// @fn          reset_acceleration
// @brief       Reset acceleration variables.
// @param       none
// @return      none
// *************************************************************************************************
void reset_acceleration(void)
{
    // Default mode is off
    sAccel.mode           = ACCEL_MODE_OFF;
}


// *************************************************************************************************
// @fn          start_acceleration
// @brief       Starts acceleration sensor.
// @param       none
// @return      none
// *************************************************************************************************
void start_acceleration(void)
{
    // Start acceleration sensor
    if (!is_acceleration_measurement())
    {
        // Set initial acceleration value corresponding to 1G to prevent false alarms on startup
        sAccel.data = 16; // TODO: Check if 16 is the correct value corresponding to 1G

        // Set mode
        sAccel.mode = ACCEL_MODE_ON;



        // Start sensor
        as_start();
    }
}


// *************************************************************************************************
// @fn          stop_acceleration
// @brief       Stops acceleration sensor.
// @param       none
// @return      none
// *************************************************************************************************
void stop_acceleration(void)
{
    // Stop acceleration sensor
    as_stop();

    // Clear mode
    sAccel.mode = ACCEL_MODE_OFF;
}


// *************************************************************************************************
// @fn          acceleration_value_is_positive
// @brief       Returns 1 if 2's complement number is positive
// @param       u8 value    2's complement number
// @return      u8          1 = number is positive, 0 = number is negative
// *************************************************************************************************
u8 acceleration_value_is_positive(u8 value)
{
  return ((value & BIT7) == 0);
}


// *************************************************************************************************
// @fn          abs_acceleration
// @brief       Return absolute value of an acceleration measurement
// @param       u8 value    g data from sensor
// @return      u8          Absolute value of g data from sensor
// *************************************************************************************************
u8 abs_acceleration(u8 value)
{
  if (!acceleration_value_is_positive(value))
  {
      // Convert 2's complement negative number to positive number
      value = ~value;
      value += 1;
  }
  return (value);
}


// *************************************************************************************************
// @fn          is_acceleration_measurement
// @brief       Returns 1 if acceleration is currently measured.
// @param       none
// @return      u8        1 = acceleration measurement ongoing
// *************************************************************************************************
u8 is_acceleration_measurement(void)
{
  return (sAccel.mode == ACCEL_MODE_ON);
}


// *************************************************************************************************
// @fn          convert_acceleration_value_to_mgrav
// @brief       Converts measured value to mgrav units
// @param       u8 value    g data from sensor
// @return      u16         Acceleration (mgrav)
// *************************************************************************************************
u16 convert_acceleration_value_to_mgrav(u8 value)
{
  u16 result;
  u8 i;

  if (!acceleration_value_is_positive(value))
  {
      // Convert 2's complement negative number to positive number
      value = ~value;
      value += 1;
  }

  result = 0;
  for (i=0; i<7; i++)
  {
      result += ((value & (BIT(i)))>>i) * mgrav_per_bit[i];
  }

  return (result);
}


// *************************************************************************************************
// @fn          detect_free_fall
// @brief       Detect if free fall has happened
// @param       none
// @return      u8 event_weight     Free fall event evaluation
// *************************************************************************************************
u8 detect_free_fall(void)
{
    u8 event_weight = 0;
    u8 sample_index = 0;
    u16 FreeFallSum = 0;

    for (sample_index = FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES; sample_index < FALL_DETECTION_WINDOW_IN_SAMPLES; sample_index++) {
        FreeFallSum += *(read_data_from_fifo_buffer(fall_data, sample_index)); // Read the oldest samples stored.
    }

    // TODO: Tune the numbers for this part of the algorithm if needed.
    if (FreeFallSum <= (FREE_FALL_THRESHOLD * FREE_FALL_BACKTRACK_IN_SAMPLES)) {
        event_weight = ((FREE_FALL_THRESHOLD * FREE_FALL_BACKTRACK_IN_SAMPLES) - FreeFallSum)/8;
        if ((FREE_FALL_THRESHOLD*FREE_FALL_BACKTRACK_IN_SAMPLES - FreeFallSum)%8 >= 4) {
            event_weight++;
        }
        if (event_weight > 0 && event_weight <=13) {
            event_weight = 1;
        } else if (event_weight > 13 && event_weight <=26) {
            event_weight = 2;
        } else if (event_weight > 26 && event_weight <=40) {
            event_weight = 3;
        }
    }

    return event_weight;
}


// *************************************************************************************************
// @fn          detect_impact
// @brief       Detect impact after free fall
// @param       none
// @return      u8 event_weight     Impact event evaluation
// *************************************************************************************************
u8 detect_impact(void)
{
    u8 sample_index = 0;
    u8 peak_index = 0;
    u8 i = 0;
    u8 event_weight = 0;
    u16 ImpactSlewRate = 0;
    u16 * temp_buff[3];
    struct peaks sPeaks[5];
    u8 lowest_peak_index;
    u8 highest_peak_index;

    /* Find the 5 highest acceleration peaks during the impact */
    for (sample_index = FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES; sample_index >= FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES - MAX_IMPACT_LENGTH_SAMPLES; sample_index--) {
        temp_buff[0] = read_data_from_fifo_buffer(fall_data, sample_index - 0);
        temp_buff[1] = read_data_from_fifo_buffer(fall_data, sample_index - 1);
        temp_buff[2] = read_data_from_fifo_buffer(fall_data, sample_index - 2);
        if ((*temp_buff[1] > *temp_buff[0]) && (*temp_buff[1] > *temp_buff[2])) {
            if (peak_index < 5) {
                sPeaks[peak_index].peaksBuff = temp_buff[1];
                sPeaks[peak_index].index = sample_index - 1;
                peak_index++;
            } else {
                for (i = 0; i < 4; i++) {
                    if (*sPeaks[i].peaksBuff <= *sPeaks[i+1].peaksBuff) {
                        lowest_peak_index = i;
                    } else {
                        lowest_peak_index = i+1;
                    }
                }
                sPeaks[lowest_peak_index].peaksBuff = temp_buff[1];
                sPeaks[lowest_peak_index].index = sample_index - 1;
            }
        }
    }

    /* Find the highest acceleration peak during the impact */
    for (i = 0; i < 4; i++) {
        if (*sPeaks[i].peaksBuff >= *sPeaks[i+1].peaksBuff) {
            highest_peak_index = i;
        } else {
            highest_peak_index = i+1;
        }
    }

    /* Calculate the impact slew rate */
    ImpactSlewRate = *sPeaks[highest_peak_index].peaksBuff - *(read_data_from_fifo_buffer(fall_data, sPeaks[highest_peak_index].index - 2));

    // TODO: Tune the numbers for this part of the algorithm if needed.
    if ((ImpactSlewRate >= IMPACT_SLEWRATE_THRESHOLD) && (*sPeaks[highest_peak_index].peaksBuff >= IMPACT_STRENGTH_THRESHOLD)) {
        event_weight = (*sPeaks[highest_peak_index].peaksBuff - IMPACT_STRENGTH_THRESHOLD)/32;
        if ((*sPeaks[highest_peak_index].peaksBuff - IMPACT_STRENGTH_THRESHOLD)%32 >= 16) {
            event_weight++;
        }
    }

    return event_weight;
}


// *************************************************************************************************
// @fn          detect_motionlessness
// @brief       Detect very little or no motion
// @param       none
// @return      u8 event_weight     Motionlessness event evaluation
// *************************************************************************************************
u8 detect_motionlessness(void)
{
    u8 event_weight = 0;
    u8 sample_index = 0;
    u16 temp_buff1 = 0;
    u16 temp_buff2 = 0;
    u16 MotionSum = 0;

    temp_buff1 = *(read_data_from_fifo_buffer(fall_data, MAX_MOTIONLESSNESS_SAMPLES - 1));

    for (sample_index = MAX_MOTIONLESSNESS_SAMPLES - 2; sample_index >= 0; sample_index--) {
        temp_buff2 = *(read_data_from_fifo_buffer(fall_data, sample_index));
        if (temp_buff1 >= temp_buff2) {
            MotionSum += temp_buff1 - temp_buff2;
        } else {
            MotionSum += temp_buff2 - temp_buff2;
        }
        temp_buff1 = temp_buff2;
    }

    // TODO: Tune the numbers for this part of the algorithm if needed.
    if (MotionSum <= MOTIONLESSNESS_THESHOLD) {
        event_weight = (MOTIONLESSNESS_THESHOLD - MotionSum)/13;
        if ((MOTIONLESSNESS_THESHOLD - MotionSum)%13 >= 6) {
            event_weight++;
        }
    }

    return event_weight;
}


// *************************************************************************************************
// @fn          sx_fall_detection
// @brief       Fall detection direct user function. Button DOWN starts/stops the fall detection.
// @param       u8 line    LINE2
// @return      none
// *************************************************************************************************
void sx_fall_detection(u8 line)
{
    if (sAlarm.state != ALARM_ON) { // During ALARM_ON any button stops the alarm. (in ports.c)
        // BUTTON UP: RUN, STOP
        if(button.flag.up) {
            if (sAccel.mode == ACCEL_MODE_OFF) {
                // (Re)start fall detection (acceleration sensor)
                start_acceleration();
            } else {
                // Stop fall detection (acceleration sensor)
                stop_acceleration();
            }
        }
    }
}


// TODO: lock buttons after fall detection is activated, but unlock them if the alarm has been triggered
// TODO: mx_fall_detection() - when NUM or STAR button is long pressed (for L1 or L2) - is it needed???
// TODO: maybe put the alarm stop in the mx_fall_detection function (long press of NUM button)


// *************************************************************************************************
// @fn          do_fall_detection
// @brief       Process acceleration data and detect falls.
// @param       none
// @return      none
// *************************************************************************************************
void do_fall_detection(void) // main()
{
    u8 acc_data[3];
    u16 acc_sum = 0;
    static u8 isDelayOver = 0;
    static u8 sample_index = 0;
    u8 impact_rating = 0;
    u8 free_fall_rating = 0;
    u8 motionlessness_rating = 0;

    as_get_data(acc_data);

    acc_data[0] = abs_acceleration(acc_data[0]);
    acc_data[1] = abs_acceleration(acc_data[1]);
    acc_data[2] = abs_acceleration(acc_data[2]);

    acc_sum = sqrt(acc_data[0]*acc_data[0] + acc_data[1]*acc_data[1] + acc_data[2]*acc_data[2]);

    // Filter acceleration data (Low pass filter)
    acc_sum = (u16)((acc_sum + sAccel.data * 4)/5);

    // Store average acceleration
    sAccel.data = acc_sum;

    write_data_to_fifo_buffer(fall_data, acc_sum);

    if (isDelayOver) {
        if (sAlarm.state != ALARM_ON) {
            // Start fall detection algorithm
            free_fall_rating = detect_free_fall();
            if (free_fall_rating > 0) {
                impact_rating = detect_impact();
            }
            if (impact_rating > 0) {
                motionlessness_rating = detect_motionlessness();
            }
            if ((free_fall_rating + impact_rating + motionlessness_rating) >= RATING_THRESHOLD) {

                // Stop fall detection and start alarm. (Alarm timeout is 10 seconds.)
                sAlarm.state = ALARM_ON;
                // Use this flag to display that fall has happened in display function.
                // TODO: Later add blinking backlight support.
                // Alarm is disabled on any button press and fall detection is resumed. (in ports.c)
                // TODO: Later add functionality to stop alarm only on long button press.
            }
        }
    } else {
        if (++sample_index > FALL_DETECTION_WINDOW_IN_SAMPLES) {    // Wait until data array is filled with data.
            isDelayOver = 1;
        }
    }
    // Update display function
    display.flag.update_fall_detection = 1;
}


// *************************************************************************************************
// @fn          display_fall_detection
// @brief       Display routine.
// @param       u8 line            LINE1
//              u8 update          DISPLAY_LINE_UPDATE_FULL, DISPLAY_LINE_CLEAR
// @return      none
// *************************************************************************************************
void display_fall_detection(u8 line, u8 update)
{
    // Show warning if acceleration sensor was not initialized properly
    if (!as_ok)
    {
        display_chars(LCD_SEG_L1_2_0, (u8*)"ERR", SEG_ON);
    }
    else
    {
        // Redraw whole screen
        if (update == DISPLAY_LINE_UPDATE_FULL)
        {
            if (sAccel.mode == ACCEL_MODE_OFF) {
                display_chars(LCD_SEG_L1_3_0, (u8 *)"FDOF", SEG_ON);
            } else if (sAccel.mode == ACCEL_MODE_ON) {
                display_chars(LCD_SEG_L1_3_0, (u8 *)"FDON", SEG_ON);
            }
        }
        else if (update == DISPLAY_LINE_UPDATE_PARTIAL)
        {
            if (sAlarm.state == ALARM_ON) {
                display_chars(LCD_SEG_L1_3_0, (u8 *)"FALL", SEG_ON_BLINK_ON);
            } else {
                if (sAccel.mode == ACCEL_MODE_OFF) {
                    display_chars(LCD_SEG_L1_3_0, (u8 *)"FDOF", SEG_ON);
                } else if (sAccel.mode == ACCEL_MODE_ON) {
                    display_chars(LCD_SEG_L1_3_0, (u8 *)"FDON", SEG_ON);
                }
            }
        }
        else if (update == DISPLAY_LINE_CLEAR)
        {
            if (sAccel.mode == ACCEL_MODE_ON) {
                stop_acceleration();
            }

            // Clean up display
            display_symbol(LCD_SEG_L1_3_0, SEG_OFF_BLINK_OFF);
        }
    }
}
