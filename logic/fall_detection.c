// *************************************************************************************************
// Include section

// system
#include "project.h"

// driver
#include "display.h"
#include "vti_as.h"

// logic
#include "fall_detection.h"
#include "simpliciti.h"
#include "user.h"
#include <math.h>

// *************************************************************************************************
// Global Variable section
struct accel sAccel;

// TODO: static????????????
u16 fall_data[FALL_DETECTION_WINDOW_IN_SAMPLES];
u16 * readIndex = NULL;

// Conversion values from data to mgrav taken from CMA3000-D0x datasheet (rev 0.4, table 4)
// TODO: create array for 8g version
const u16 mgrav_per_bit[7] = { 18, 36, 71, 143, 286, 571, 1142 };

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
    static u16 * position = NULL;

    if (writeIndex >= FALL_DETECTION_WINDOW_IN_SAMPLES) {
        writeIndex = 0;
    }
    (u16 *)(buff_addr + writeIndex) = data;
    writeIndex++;

    readIndex = (u16 *)(buff_addr + writeIndex - 1);
}


// *************************************************************************************************
// @fn          read_data_from_fifo_buffer
// @brief       Reads data from the FIFO buffer with sample offset.
// @param       u16 * buff_addr - Address of FIFO buffer.
//              u8 backsamples  - Sample back offset.
// @return      u16             - Value of a previous sample.
// *************************************************************************************************
u16 read_data_from_fifo_buffer(u16 * buff_addr, u8 backsamples)
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
  // Clear timeout counter
  sAccel.timeout        = 0;

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
        sAccel.data = 32; // TODO: Check if 32 is the correct value corresponding to 1G

        // Start sensor
        as_start();

        // Set timeout counter
        sAccel.timeout = ACCEL_MEASUREMENT_TIMEOUT;

        // Set mode
        sAccel.mode = ACCEL_MODE_ON;
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
  return ((sAccel.mode == ACCEL_MODE_ON) && (sAccel.timeout > 0));
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
    u8 i = 0;
    u16 FreeFallSum = 0;

    for (i = FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES; i < FALL_DETECTION_WINDOW_IN_SAMPLES; i++) {
        FreeFallSum += *(read_data_from_fifo_buffer(fall_data, i)); // Read the oldest samples stored.
    }

    if (FreeFallSum <= (FREE_FALL_THRESHOLD * FREE_FALL_BACKTRACK_IN_SAMPLES)) {
        event_weight = ((FREE_FALL_THRESHOLD * FREE_FALL_BACKTRACK_IN_SAMPLES) - FreeFallSum)/640;  // TODO: This is just an example.
        if ((FREE_FALL_THRESHOLD*FREE_FALL_BACKTRACK_IN_SAMPLES - FreeFallSum)%640 >= 320) {        // TODO: This is just an example.
            event_weight++;
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
    u16 * peaks_buff[5];
    u8 lowest_peak_index;
    u8 highest_peak_index;

    /* Find the 5 highest acceleration peaks during the impact */
    for (sample_index = 0; sample_index < MAX_IMPACT_LENGTH_SAMPLES - 1; sample_index++) {
        temp_buff[0] = read_data_from_fifo_buffer(fall_data, FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES - sample_index - 1);
        temp_buff[1] = read_data_from_fifo_buffer(fall_data, FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES - sample_index - 2);
        temp_buff[2] = read_data_from_fifo_buffer(fall_data, FALL_DETECTION_WINDOW_IN_SAMPLES - FREE_FALL_BACKTRACK_IN_SAMPLES - sample_index - 3);
        if ((*temp_buff[1] > *temp_buff[0]) && (*temp_buff[1] > *temp_buff[2])) {
            if (peak_index < 5) {
                peaks_buff[peak_index] = temp_buff[1];
                peak_index++;
            } else {
                for (i = 0; i < 4; i++) {
                    if (*peaks_buff[i] <= *peaks_buff[i+1]) {
                        lowest_peak_index = i;
                    } else {
                        lowest_peak_index = i+1;
                    }
                }
                peaks_buff[lowest_peak_index] = temp_buff[1];
            }
        }
    }

    /* Find the highest acceleration peak during the impact */
    for (i = 0; i < 4; i++) {
        if (*peaks_buff[i] >= *peaks_buff[i+1]) {
            highest_peak_index = i;
        } else {
            highest_peak_index = i+1;
        }
    }

    /* Calculate the impact slew rate */
    ImpactSlewRate = *peaks_buff[highest_peak_index] - *(peaks_buff[highest_peak_index]-2);

    if ((ImpactSlewRate >= IMPACT_SLEWRATE_THRESHOLD*2) && (*peaks_buff[highest_peak_index] >= IMPACT_STRENGTH_THRESHOLD)) {
        event_weight = (*peaks_buff[highest_peak_index] - IMPACT_STRENGTH_THRESHOLD)/1024;    // TODO: This is just an example.
        if ((*peaks_buff[highest_peak_index] - IMPACT_STRENGTH_THRESHOLD)%1024 >= 512) {      // TODO: This is just an example.
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
    if(alarm_is_on) {
        if(button.flag.up) { // Stop alarm if UP button is pressed.
            stop_alarm();
        }
    } else {
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
// TODO: mx_fall_detection() - when NUM or STAR button is long pressed (for L1 or L2)
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
    u16 * buff_current_position = NULL;
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
        free_fall_rating = detect_free_fall();
        if (free_fall_rating > 0) {
            impact_rating = detect_impact();
        }
        if (impact_rating > 0) {
            motionlessness_rating = detect_motionlessness();
        }
        if ((free_fall_rating + impact_rating + motionlessness_rating) > RATING_THRESHOLD) {
            // TODO: Stop fall detection.
            // TODO: Start alarm. (Maybe use just a few beeps the first 10 seconds.)
            // TODO: Wait for button press to disable the alarm.
            // TODO: Restart fall detection.
            display.flag.update_fall_detection = 1;
        }
    } else {
        if (++sample_index > FALL_DETECTION_WINDOW_IN_SAMPLES) {    // Wait until data array is filled with data.
            isDelayOver = 1;
        }
    }
}


// TODO: FIX the display function
// *************************************************************************************************
// @fn          display_fall_detection
// @brief       Display routine.
// @param       u8 line            LINE1
//              u8 update          DISPLAY_LINE_UPDATE_FULL, DISPLAY_LINE_CLEAR
// @return      none
// *************************************************************************************************
void display_fall_detection(u8 line, u8 update)
{
  u8 * str;
  u8 raw_data;
  u16 accel_data;

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
          {
              // Display decimal point
              display_symbol(LCD_SEG_L1_DP1, SEG_ON);
          }
      }
      else if (update == DISPLAY_LINE_UPDATE_PARTIAL)
      {
          display_char(LCD_SEG_L1_3, 'X', SEG_ON);

          display_chars(LCD_SEG_L1_2_0, str, SEG_ON);

          // Display sign
          if (acceleration_value_is_positive(raw_data))
          {
              display_symbol(LCD_SYMB_ARROW_UP, SEG_ON);
              display_symbol(LCD_SYMB_ARROW_DOWN, SEG_OFF);
          }
          else
          {
              display_symbol(LCD_SYMB_ARROW_UP, SEG_OFF);
              display_symbol(LCD_SYMB_ARROW_DOWN, SEG_ON);
          }
      }
      else if (update == DISPLAY_LINE_CLEAR)
      {
          if (sAccel.mode == ACCEL_MODE_ON) {
              stop_acceleration();
          }

          // Clean up display
          display_symbol(LCD_SEG_L1_DP1, SEG_OFF);
          display_symbol(LCD_SYMB_ARROW_UP, SEG_OFF);
          display_symbol(LCD_SYMB_ARROW_DOWN, SEG_OFF);
      }
  }
}





// TODO: WTF IS THIS - CHECK HOW IT WORKS !!!!!!
u16 fast_sqrt(u32 value)
{
//    MSP430 Family Mixed-Signal Microcontroller Application Reports - SLAA024
//
//    5.1.8.1 Square Root for 32-Bit Integer Numbers
//
//    The square root of a 30-bit integer number is calculated. The result contains
//    15 correct fractional bits. The subroutine uses the method known from the find-
//    ing of a square root by hand. This method is much faster than the widely known
//    NEWTONIAN method and only 720 cycles are needed. This subroutine was
//    developed by Jürg Müller Software–Art GmbH/Zurich. The C program code
//    needed is also shown:

    u32 y, h;
    u8 i = 0;
    h = value;
    value = y = 0;

    for (i = 0; i < 32; i++) {
        // value is actually 2*value
        value <<= 1; value++;   // 4*value + 1
        if (y < value) {
            value -= 2;
        } else {
            y -= value;
        }
        value++;
        y <<= 1;    // <y, h> <<= 2
        if (h & Minus) y++;
        h <<= 1;
        y <<= 1;
        if (h & Minus) y++;
        h <<= 1;
    }
    return value;
}



/* ========================================================================== */
/**
* @fn aaaSqRoot(). Calculate sqrt of m.
*/
/* ========================================================================== */

static int aaaSqRoot(int m)
{
    int    i = 0;
    int    shiff = 0;

    while( m > 100 ) {
        shiff++;
        m >>= 2;
    }

    while((i * i) < m ) {
        i+=1;
    }

    return (i << shiff);
}
