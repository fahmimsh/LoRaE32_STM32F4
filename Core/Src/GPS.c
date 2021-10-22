#include "GPS.h"

void GPS_p(){
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //Reset aller variablen, falls es was schief geht steht überall null
    GPS.valid = false;
    GPS.lat,GPS.lon,GPS.hdop,GPS.alt,GPS.bla,GPS.index_end,GPS.hour,GPS.min,GPS.sec,GPS.millis,GPS.sats,GPS.checksum,GPS.checksum_calc = 0;

    if (strstr(GPS.message, "$GNGGA")){
        //Valid message in stream
        //Beispiel von Position zuhause $GNGGA,151017.00,4916.72813,N,01127.64911,E,1,09,0.93,435.5,M,46.6,M,,*4A
        GPS.message2 = strstr(GPS.message,"$GNGGA");         //pointer auf begin des GNGGA satzes setzen
        sscanf(GPS.message2, "$GNGGA,%2hd%2hd%2hd.%2hd,%lf,%c,%lf,%c,%hd,%hd,%lf,%lf,M,%lf,M,,*%x",&GPS.hour,&GPS.min,&GPS.sec,&GPS.millis,&GPS.lat,&GPS.NS_ind,&GPS.lon,&GPS.EW_ind,&GPS.fixtype,&GPS.sats,&GPS.hdop,&GPS.alt,&GPS.bla,&GPS.checksum);

        printf("position %f %c , %f %c @ %f M\nfix_info:\nSATS: %d\nFIX: %d\nHDOP: %f\nchecksum: 0x%2hx\r\n",GPS.lat,GPS.NS_ind,GPS.lon,GPS.EW_ind,GPS.alt,GPS.sats,GPS.fixtype,GPS.hdop,GPS.checksum);

        //Resette alles für Checksumme
        GPS.checksum_calc=0;
        GPS.index_end=1;
        char x = 'A';

        //solange das zeichen kein * ist weiter prüfen und den index mit hoch zählen
        while(x!= '*'){
            GPS.index_end++;
            x = GPS.message2[GPS.index_end];
        }

        //warum -2 erklärt sich mir nicht ganz
        GPS.index_end -=2;

        //Exclusiv ODER verknüpfung aller empfangenen Zeichen im Satz bildet die Checksumme
        for (int i=1; i<GPS.index_end;i++){
            GPS.checksum_calc=GPS.checksum_calc^GPS.message2[i];

        }
            printf("berechnete checksumme: 0x%x",GPS.checksum_calc);

        //Wenn die empfangene Chekcsumme = berechnette Checksumme sind die Daten Korrekt
            if(GPS.checksum_calc==GPS.checksum){
                GPS.valid = true;
                GPS.FIXTIME = (HAL_GetTick());
            printf("\nZEIT UTC: %d:%d:%d.%d\n",GPS.hour,GPS.min,GPS.sec,GPS.millis);
            }
    }


}

double min_to_degree(double lonlat, char indicator){
	char temp_str[20];
	sprintf(temp_str,"%lf", lonlat);
	char * str_point = strchr(temp_str, '.');
	double mins;
	sscanf(str_point-2, "%lf", &mins);
	unsigned short int degrees;

	sscanf(str_point-5, "%3hd", &degrees);
	if (degrees == 0){
		sscanf(str_point-4, "%2hd", &degrees);
	}
	double end_degrees = degrees+(mins/60);
	if(indicator == 'S'||indicator == 'W'){
		end_degrees =end_degrees*-1;
	}
	return(end_degrees);
}

double get_lat_degree(){
	double degrees = min_to_degree(GPS.lat, GPS.NS_ind);
	return (degrees);
}
double get_lon_degree(struct gps_t *GPS){
	double degrees = min_to_degree((*GPS).lon, (*GPS).EW_ind);
	return (degrees);
}
double get_alt(struct gps_t *GPS){
	return (*GPS).alt;
}
char get_GPS_time(struct gps_t *GPS){
	char temp_str[14];
	sprintf(temp_str, "%d:%d:%d.%d", (*GPS).hour,(*GPS).min,(*GPS).sec,(*GPS).millis);
	return (*temp_str);
}
