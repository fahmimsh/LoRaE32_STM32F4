

/*
 *
 * Author:          Daniel Bernhard
 * Version:         v0.1
 */
#ifndef INC_GPS_H_
#define INC_GPS_H_

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#define RX_BUFFER_SIZE 256

struct gps_t{
  long int FIXTIME;									//Systicktimer at verrification of checksum
  double lat,lon,hdop,alt,bla;
  short index_begin,index_end,hour,min,sec,millis,fixtype,sats,checksum,checksum_calc;
  char NS_ind,EW_ind;
  bool valid;
  char* message;
  char* message2;

};

struct gps_t GPS;

bool update;
char Rx_data[RX_BUFFER_SIZE];


void GPS_p();														//should be called if data was copied to .message variable of GPS_t variable
double min_to_degree(double lonlat, char indicator);				//could be called with lat or lon informations to convert this to degree Format

#endif /* INC_GPS_H_ */
