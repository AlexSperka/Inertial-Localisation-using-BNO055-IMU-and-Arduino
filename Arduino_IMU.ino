/*
   *3D-Lokalisierung Labor Mechatronik
   *Stand: 18.06.2017
   *Board: Arduino Mega
   *Editor: Alexander Sperka

   Connections BNO055                              Connections GPS-Ultimate
   ===========                                     ===========
   Connect SCL to 21 (=SCL)                        Connect Rx to 18 (=TX1) purple cable
   Connect SDA to 20 (=SDA)                        Connect TX to 19 (=RX1) grey cable
   Connect VDD to 3-5V DC                          Connect VDD to 3-5V DC
   Connect GROUND to common ground                 Connect GROUND to common ground
    
   Connections Hall Sensor                         Connections LEDs
   ===========                                     ===========
   Connect Data to 2 (Interrupt Pin)               Connect LED red to Pin 8 (red cable)
   Connect VDD to 3-5V DC                          Connect LED blue to Pin 9 (blue cable)
   Connect GROUND to common ground                 Connect Ground to common ground (brown), only one cable is needed
                                                   
   Connections SD Card         
   ===========                                     
   Connect MISO to Pin 50 green cable              
   Connect SCK  to Pin 52 yellow cable             
   Connect CS   to Pin 53 red cable             
   Connect MOSI to Pin 51 orange cable
   Connect GND  to GND
   Connect VCC  to 3-5V Dc
   ´                              
*/
 
/*#include <SoftwareSerial.h> //not needed for Arduino Mega, -> Hardware Serial */
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

/* Func prototype SD*/
void init_SD(void);                       /*Initialize SD modul, checks if connected*/
void create_new_SD_File(void);            /*Checks which filenames already excist, creates new one*/
void print_setup_data_to_console(void);   /*Called in the setup, gives basic informations after reset*/
void print_data_to_SD(void);              /*Called once every 100ms, saves data from BNO, GPS and Hall Sensor on SD-Card*/
void print_data_to_Console(void);         /*Only for debugging, print data once every 100ms to console*/

/* Func prototype GPS*/
void init_GPS(void);                      /*Initialize Ultimate GPS modul, checks if connected*/
void useInterrupt(boolean);               /*Enable interrupt, if turned off nothing will work*/

/* Func prototype BNO055*/
void init_BNO(void);                      /*Initialize BNO055, checks if connected, checks calibrations status, ...*/
void displaySensorDetails(void);          /*... waits until it is calibrated enough, calls functions below*/
void displaySensorStatus(void);
int  displayCalStatus(void);

/* Func prototype Hall Sensor*/
void init_HallSensor(void);               /*Initialize Hall Sensor, checks if connected, enables special interrupt*/    
void magnet_detect(void);                 /*Called every time a magnet crosses the Hall Sensor, saves data to variable*/


/*******************************************************************************************************/
/********************************         LEDs              ********************************************/
unsigned int led_red = 8;
unsigned int led_blue = 9;

/*******************************************************************************************************/
/********************************         SD Shield         ********************************************/

File myFile;                              /*object of typ FILE, struct array for data*/

const int chipSelect = 53;                /*Could be any pin, sometimes called SS pin*/

char array_sd[] = "test00.txt";           /*Name of File, do not change 00!, first 4 letters can be changed */
unsigned int SD_file_full = 0;            /*Check if SD File is full*/

/*******************************************************************************************************/
/********************************         BNO055            ********************************************/

#define BNO055_SAMPLERATE_DELAY_MS (100)  /* Set the delay between fresh samples */

Adafruit_BNO055 bno = Adafruit_BNO055(55);/* Struct of typ Adafruit_BNO055 */

imu::Quaternion quat;                     /* Struct array for Quaternion data */

sensors_event_t event;                    /* Type is used to perform and handle a specific sensor reading, called an 'event',...
                                             ...and contains data from the sensor from a specific moment in time.*/

/*******************************************************************************************************/
/********************************           GPS              ******************************************/

HardwareSerial mySerial = Serial1;        /* Select Hardware Serial Port 1 if using Arduino Mega */

Adafruit_GPS GPS(&Serial1);               /* Hand over pointer -> GPS-Port to struct GPS of typ Adafrut_GPS (see Lib) */

#define GPSECHO  true                     /* Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console...
                                          ... Set to 'true' if you want to debug and listen to the raw GPS sentences.*/
                                          
boolean usingInterrupt = false;           /* Says if interrupt is on/off, off by default*/

uint32_t timer = millis();                /* Timer for GPS parsing, checking */

/*******************************************************************************************************/
/********************************         HALL Sensor            ***************************************/

volatile unsigned int counter_hall = 0;   /* counter for hall sensor events since last print to SD/console */

/*******************************************************************************************************/
/*******************************************************************************************************/

/*******************************************************************************************************/
/*
        Print Initialize Data to Console, use if reading Data from Serial Port
*/
/*******************************************************************************************************/
void print_setup_data_to_console(void)
{
  /* Clears up any data left from previous projects*/
  Serial.println("CLEARDATA");
  /* Always write LABEL, so excel knows the next things will be the names of the columns (instead of Acolumn you could write Time for instance)*/
  Serial.println("LABEL,Time,Timer,Quat_w, Quat_x, Quat_y, Quat_z,, Google maps Location_lat, Google maps Location_long,,Hall Sensor Data,,Altidue");
  /* Reset Timer to 0*/
  Serial.println("RESETTIMER");
  /* Print project title*/
  Serial.println("3D-Lokalisierung Fahrrad");
  /* Print date */
  Serial.println("11.04.2017");
}

/*******************************************************************************************************/
/*
        Initialize SD Card, checks if SD-Card connected, prints SD status, calls create_new_SD_File()
*/
/*******************************************************************************************************/
void init_SD(void)
{
  /* Initialise SD Card */
  while(!SD.begin(chipSelect)) {
    Serial.println("initialization SD failed!");
  }
  Serial.println("initialization SD done.");
  
  Serial.println("Creating test_.txt...");
  /* Call Function to check which files still excists and create new one, see below*/
  create_new_SD_File();
}

/*******************************************************************************************************/
/*
    check if name excists, create new name, create new SD File, print header line to text file 
*/
/*******************************************************************************************************/
void create_new_SD_File(void)
{
  noInterrupts();
  
  static int i = 1;
  static int j = 1;
  /*Check if File exists*/
  while (SD.exists(array_sd)) 
  {
    if(i>9)
    {
      i=0;
      array_sd[4] = j+48,
      j++;
      if(j>9) j = 0;  
    }
    Serial.print(array_sd);
    Serial.println(" exists");
    array_sd[5] = i+48;
    i++;
  }
  /* Open new File and Write into it*/
  myFile = SD.open(array_sd, FILE_WRITE);
  Serial.print(array_sd);
  Serial.println("SD File open");
  //Serial.println("Time,Timer,Quat_w, Quat_x, Quat_y, Quat_z,, Google maps Location_lat, Google maps Location_long,,Hall Sensor Data"); 
  /* Header for Text Sheet*/
  myFile.println("Quat_w; Quat_x; Quat_y; Quat_z;; Google maps Location_lat; Google maps Location_long;;Hall Sensor Data;;Altitude"); 
  myFile.close();

  interrupts();
}

/*******************************************************************************************************/
/*
        Initialize GPS, set update rate, enable Interrupt for GPS data
*/
/*******************************************************************************************************/
void init_GPS(void)
{ 
  /* Initialise GPS sensor*/
  /* 9600 NMEA is the default baud rate*/
  GPS.begin(9600);
  /* Turn on RMC (recommended minimum) and GGA (fix data) including altitude */
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  /* Set the update rate */
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  /* Request updates on antenna status */
  GPS.sendCommand(PGCMD_ANTENNA);
}

/*******************************************************************************************************/
/*
        Initialize BNO055, check if connected, display sensor data
*/
/*******************************************************************************************************/
void init_BNO(void)
{
  /* Initialise BNO sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    /*while(1);*/
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();
  /* Optional: Display current status */
  displaySensorStatus();
  /* Display Calibration Status of BNO */
  displayCalStatus();
  /* Use and set external Crystal */
  bno.setExtCrystalUse(true);
}

/*******************************************************************************************************/
/*
        Initialize Hall Sensor, enable Interrupt for Sensor Pin
*/
/*******************************************************************************************************/
void init_HallSensor(void)
{
  /*Initialize the intterrupt pin (Arduino digital pin 2) for Hall Sensor*/
  attachInterrupt(0, magnet_detect, RISING);
}

/*******************************************************************************************************/
/*******************************************************************************************************/
/*
    Setup, connect with 115200, Initialize GPS, BNO and Hall Sensor (+Barometer?), display Status
*/
/*******************************************************************************************************/
/*******************************************************************************************************/
void setup() 
{
  Serial.begin(115200);                             /* Set baudrate to 115200 */

  //print_data_to_console();                        /* Use if reading data over Serial port, turn of if SD-Card is used*/
  pinMode(led_red, OUTPUT);                     
  pinMode(led_blue, OUTPUT);                        /* Set pin 8 and 9 as Output for status LEDs*/

  digitalWrite(led_red, HIGH);                      /* Turn red led on, signal for user that setup is not finished*/

  init_BNO();                                       /* initialize BNO */
  Serial.println("Init BNO");
  init_HallSensor();                                /* initialize Hall Sensor */
  Serial.println("Init HS");
  init_GPS();                                       /* initialize GPS */
  Serial.println("Init GPS");

  init_SD();                                        /* initialize SD */
  Serial.println("Init SD");
//  myFile = SD.open(array_sd);                     /* open new SD-File */

  useInterrupt(true);                               /*enable Interrupt, needed for getting and saving data*/
  digitalWrite(led_red, LOW);                       /* Turn red led off, signal for user that setup is finished*/

  delay(1000);
}


/*******************************************************************************************************/
/*******************************************************************************************************/
/*
    Main_loop, quat_readout in Array, GPS-parsing, Timerreset, other Stuff
*/
/*******************************************************************************************************/
/*******************************************************************************************************/
void loop()                     // run over and over again
{
  bno.getEvent(&event);                             /* Get sensor event from BNO, needs to be in loop */

  quat = bno.getQuat();                             /* Get Quat fro BNO, save in struct array quat[]*/
  
  if (GPS.newNMEAreceived())                        /* if a sentence is received check the checksum and parse it*/
  {
    if (!GPS.parse(GPS.lastNMEA()))                 /* Sets the newNMEAreceived() flag to false*/
    return;                                         /* If failed to parse a sentence wait for another */
  }

  if (timer > millis())  timer = millis();          /* if millis() or timer wraps around, we'll just reset it*/

  if (millis() - timer > 2000) timer = millis();    /* print out the current stats every 2 sec, resets the timer*/
   
  delay(BNO055_SAMPLERATE_DELAY_MS);                /* Wait the specified delay before requesting nex data */
}


/*******************************************************************************************************/
/*******************************************************************************************************/
/*
    Interrupt called once a millisecond: GPS_readout, GPS-Data and BNO Quat_ges send to Serial, ...
*/
/*******************************************************************************************************/
/*******************************************************************************************************/
SIGNAL(TIMER0_COMPA_vect) 
{
//  noInterrupts(); /*no Interrupts while sending Data*/
  
  char c = GPS.read();                              /*GPS readout and store Data*/
  
  static uint16_t time_counter;                     /* timer for 1 sec */

  if(time_counter>85)                               /* ca. once every 100 milliseconds*/
  { 
    /* For debugging purpose only! Turn off if not needed, prints all Data to Console */
    //print_data_to_Console();

    /* print GPS-, Quaternion- and Hall Sensor-Data to SD-Card */
    print_data_to_SD();

    counter_hall=0;                                 /*set back to 0, for next interrupt*/
    
    time_counter=0;                                 /*set timer of 1 sek. back*/
  }
  
  time_counter++;                                   /*increment timer */
//  interrupts();                                   /*interrupt enable*/
}


/*******************************************************************************************************/
/*
        Called from Interrupt, Print GPS data, Quaternion and Hall-Sensor-Contacts to SD-Card
*/
/*******************************************************************************************************/
void print_data_to_SD(void)
{
//  noInterrupts();

    myFile = SD.open(array_sd, FILE_WRITE);         /* open new file for next data strings */

    if(myFile==0)                                   /* if myFile is NULL, error, turn red LED on, blue LED off*/
    {
        digitalWrite(led_red, HIGH);                
        digitalWrite(led_blue, LOW);
        
        Serial.println("Error Print Data to SD");
        myFile.close();                             /* close file after 1000 data strings are saved in SD File */
        create_new_SD_File();                       /* try to create new file */
        return 0;                                   /* leave function, back to interrupt*/
    }
    
    if(!SD.begin(chipSelect))                       /* if SD modul can be adressed and myFile is not NULL then print data to SD*/
    {
      if(myFile!=0)
      {
          digitalWrite(led_red, LOW);
          digitalWrite(led_blue, HIGH);

          myFile.print(quat.w(), 6);
          myFile.print(";");                        /* for SD File Sheet */              
          myFile.print(quat.x(), 6);
          myFile.print(";");                        /* for SD File Sheet */
          myFile.print(quat.y(), 6);
          myFile.print(";");                        /* for SD File Sheet */
          myFile.print(quat.z(), 6);
                  
          myFile.print(";");                        /* for SD File Sheet */
          myFile.print(";");                        /* for SD File Sheet */
      
          myFile.print(GPS.latitudeDegrees, 6);
          myFile.print(";");                        /* for SD File Sheet */
          myFile.print(GPS.longitudeDegrees, 6);
      
          myFile.print(";");                        /* for SD File Sheet */
          myFile.print(";");                        /* for SD File Sheet */
      
          myFile.print(counter_hall);               /*print contacts of hallsensor during last 100 milliseconds*/
          
          myFile.print(";"); 
          myFile.print(";"); 
          
          myFile.print(GPS.altitude);               /*GPS Altitude*/
          
          myFile.println("");
      } 
    }

    myFile.close();                                 /* close file after data strings are saved in SD File */

//  interrupts();
}

/*******************************************************************************************************/
/*
        For Debugging only!!! Called from Interrupt if enabled, Print GPS, Quaternion and Hall-Sensor-Contacts to Console
*/
/*******************************************************************************************************/
void print_data_to_Console(void)
{
//  noInterrupts();
    /*  Print data to Consol */
    Serial.print("DATA,TIME,TIMER");                /* for Excel sheet */
    Serial.print(",");                              /* for Excel sheet */
    Serial.print(quat.w(), 6);
    Serial.print(",");                              /* for Excel sheet */              
    Serial.print(quat.x(), 6);
    Serial.print(",");                              /* for Excel sheet */ 
    Serial.print(quat.y(), 6);
    Serial.print(",");                              /* for Excel sheet */ 
    Serial.print(quat.z(), 6);
            
    Serial.print(",");                              /* for Excel sheet */
    Serial.print(",");                              /* for Excel sheet */

    Serial.print(GPS.latitudeDegrees, 6);
    Serial.print(",");                              /* for Excel sheet */
    Serial.print(GPS.longitudeDegrees, 6);

    Serial.print(",");                              /* for Excel sheet */
    Serial.print(",");                              /* for Excel sheet */
    Serial.print(counter_hall);                     /*print contacts of hallsensor during last second*/
    
    Serial.print(","); 
    Serial.print(","); 

    Serial.print(GPS.altitude);                     /*GPS altitude*/
    
    Serial.println(""); 

//    interrupts();
}
/*******************************************************************************************************/
/*******************************************************************************************************/
/*
    magne_detect, Called whenever a magnet/interrupt is detected
*/
/*******************************************************************************************************/
/*******************************************************************************************************/
void magnet_detect(void)
 {
  noInterrupts();                                   /*no Interrupt through Serial print*/
  counter_hall++;
  
  /*for debug only:*/
//   static int i = 0;
//   Serial.print("detect");Serial.println(i);
//   i++;
  interrupts();
 }

/*******************************************************************************************************/
/*
        use Interrupt (on/off) , for more details look into GPS header
*/
/*******************************************************************************************************/
void useInterrupt(boolean v) 
{
  if (v)                                           /* Timer0 is already used for millis() - we'll just interrupt somewhere
                                                   ...in the middle and call the "Compare A" function above*/
  {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else 
  {
    TIMSK0 &= ~_BV(OCIE0A);                        /* do not call the interrupt function COMPA anymore*/
    usingInterrupt = false;
  }
}

/*******************************************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/*******************************************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/*******************************************************************************************************/
/*
    Display some basic info about the sensor status
*/
/*******************************************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/*******************************************************************************************************/
/*
    Display sensor calibration status
*/
/*******************************************************************************************************/
int displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

/* The data should be ignored until the system calibration is > 0 */
  /*  Serial.print("\t");*/
  while(system)
  {
    Serial.println("BNO not calibrated");
    delay(100);
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
  }

  Serial.println("BNO Full calibrated");
  return(system);
}

 
/*********************************    END OF PROGRAM    ************************************************/
/*******************************************************************************************************/
