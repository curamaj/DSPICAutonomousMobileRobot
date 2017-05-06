/* Include file for all of teh roobocar definitions */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#ifndef _ROBOCARGPS_
#define _ROBOCARGPS_
typedef bool boolean; //tdbool for some reason does not define this ??

#define MAXLINELENGTH 128 // Max line length is around 80.
#define pi 3.14159265358979323846 // Definition of the magic number Pi

typedef struct NMEA_record { 
	// This is taken from Aurdino, many fields are not used but I do not
	// want to much around with the for now. We can clean this up later.
	// I added and calculate my own fields called longitude2, latitude2
	// which I use. 
							
  boolean valid; // true => Valid record, false => invalid
  char raw[MAXLINELENGTH];
  uint8_t hour, minute, seconds, year, month, day;
  uint16_t milliseconds;
  // Floating point latitude and longitude value in degrees.
  double latitude, longitude;
  double latitude2, longitude2;
  // Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
  // and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
  //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
  int32_t latitude_fixed, longitude_fixed;
  double latitudeDegrees, longitudeDegrees;
  double latitudeDegrees1, longitudeDegrees1;
  double latitudeDegrees2, longitudeDegrees2;
  double latitudeMinutes, longitudeMinutes;
  double latitudeMinutes2, longitudeMinutes2;
  double geoidheight, altitude;
  double speed, angle, magvariation, HDOP;
  char lat, lon, mag;
  boolean fix;
  uint8_t fixquality, satellites;
} nmea_rec;

// Microcontroller: dspic33fj128mc802 Magic 
#define FREQ62_5HZ 7605 // Use this to get 62.5 Hz 
#define FREQ50HZ 9350 / / 50.74 Hz
double TIMETOTRAVELAMETER = 1.5; // This is adhooc at best!!! Calculated based
								 // on actual car movement :(

/* Global data  Please note that They all start wuith rc_ */
nmea_rec rc_src; // This is teh co-ordinates of the car got via GPSFix.
nmea_rec rc_dst; // Destination Point B. Human entered co ordinates
double rc_dist2target; // Straignt line distance to target from source
double rc_proximity_distance = .10; // 1 Meter . When we call Stop() one meter
									// before, the momentum carries it one meter									// Based on theory and experimentation.
double	rc_angle = 0.0;				// Left or Right angle to turn.
double rc_angle_per_pulse = 1.0;
// PWM duty cycle definitions 
int rc_full_cycle = 0;
int	rc_left = 0; 
int	rc_right =  0;
int	rc_middle = 0;

int	rc_fwd = 0;
int	rc_still = 0;
int	rc_back = 0;
  
// Function prototypes
boolean parseNMEA(char *); // Parse NMEA record. 
uint8_t parseHex(char); // NMEA parser assist.

// distance() is what I find on internet and it does not work.
// distance2() is my routine, which accurately calculates the 
// src to dst distance. I use distance2().
double distance(double lat1, double lon1, double lat2, double lon2, char unit);
double distance2(double lat1, double lon1, double lat2, double lon2, char unit);

// Car Parser structure
typedef struct Parser {
	void (*func)(void);
	char help[80];
} ParserFunctions;

#define CMDS 128 
ParserFunctions parse_function[CMDS];
#define BTBUF 128
char  btbuf[BTBUF];
int	exit_parser  = 0;
int	log_data = 0;

// Our parser entry point routines.
void Help(void); // Help command to display the functions
unsigned char GetInp(void); // Helper to UART input
double GetFloatArg(void); // Helper to get the float number from input.
void GetGPSFix(void); 	// Get GPS records, parse and once you 
						// find a GPRMC or GPGGA with a fix fict it as source.
void printfBT(char *buf ); // Bluetooth printf() function helper.
boolean GetGPRMCorGPGGA( char *buf, nmea_rec *gps_rec); // Helper to GSFix()
						// to look at only GPRMC or GPGGA record in GPS data.
boolean GetGPSLine(char *);
void Stop(void); // STop the car. 


#endif

