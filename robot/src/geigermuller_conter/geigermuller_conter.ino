#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <SPI.h>

// #define USE_PACK_DATA_GEIGE/R
#define SHOW_LOG_MONITOR
#define USE_LONG_DATA

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch
#define MAX_DOSIS 9460.8  // batas aman dosis radiasi = 9460,8 Sv/detik ( berdasarkan Perka BAPETEN -> 0.3 mSV/tahun, untuk perkalenderan 1 tahun =365 hari) 
#define CAP_PERIOD 5000



#ifdef USE_LONG_DATA
typedef std_msgs::Int64   sDataPack;
#else
typedef std_msgs::Int32   sDataPack;
#endif

unsigned long   counts, counts_minutes;                 //variable for GM Tube events
unsigned long   cpm, cps;                    //variable for CPM
//unsigned long   cps;
unsigned int    multiplier;             //variable for calculation CPM in this sketch
unsigned long   previousMillis = 0, previousMillis_inMinutes = 0;     //variable for time measurement
unsigned long   Dosis;                  //variable for dose measurement
unsigned int    t;                      //variable for waktu batas
unsigned int    eror;                   //variable for waktu batas
unsigned int    n_cacah = 1;


void tube_impulse(){       //subprocedure for capturing events from Geiger Kit
  counts++;
}

ros::NodeHandle nh;



#if !defined USE_PACK_DATA_GEIGER
sDataPack   cps_msgs;
sDataPack   cpm_msgs;
#else
struct _geiger_pack
{
  sDataPack cps;
  sDataPack cpm;
};
//_geiger_pack geiger_counter_pack;
 sDataPack   geiger_counter_array[2];
#endif



#if !defined USE_PACK_DATA_GEIGER
ros::Publisher    CPS("/exa_robot/geiger_sensor_cps", &cps_msgs);
ros::Publisher    CPM("/exa_robot/geiger_sensor_cpm", &cpm_msgs);
#else
 ros::Publisher    geiger_counter("geiger_counter", &geiger_counter_msgs[2]);
//ros::Publisher    geiger_counter("/exa_robot/geiger_sensor", &geiger_counter_pack);
#endif

void setup(){             //setup subprocedure
  nh.initNode();
#if !defined USE_PACK_DATA_GEIGER
  nh.advertise(CPS);
  nh.advertise(CPM);
#else
  nh.advertise(geiger_counter);
#endif
  counts = 0;
  cpm = 0;
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
  Serial.begin(115200);
  attachInterrupt(0, tube_impulse, FALLING); //define external interrupts 
}

void loop(){                                 //main cycle
  unsigned long currentMillis = millis();
  
//  if((currentMillis - previousMillis) > (LOG_PERIOD/5))
  if((currentMillis - previousMillis) >= CAP_PERIOD)
  {
//    n_cacah++;
//    if(n_cacah>5)
    {
      cps   = (counts) * multiplier;
      counts_minutes += counts;
      previousMillis = currentMillis;
#if !defined USE_PACK_DATA_GEIGER      
      cps_msgs.data = cps;
      CPS.publish(&cps_msgs);
#else      
      // geiger_counter_msgs[0].data = cps;
      // geiger_counter.publish(&geiger_counter_msgs[0]);
      geiger_counter_pack.cps.data = cps;
      // geiger_counter.publish(&geiger_counter_pack);
#endif
      
#if defined SHOW_LOG_MONITOR
      Serial.print("second : ");     
      Serial.println(counts);
      Serial.print("cps : ");     
      Serial.println(cps);
#endif

      counts = 0;
    }     
  }
  if((currentMillis - previousMillis_inMinutes) >= MAX_PERIOD)
  {
    cpm = (counts_minutes) * multiplier;
    previousMillis_inMinutes = currentMillis;
#if !defined USE_PACK_DATA_GEIGER    
    cpm_msgs.data = cpm;
    CPM.publish(&cpm_msgs);
#else    
    // geiger_counter_msgs[1].data = cpm;
    // geiger_counter.publish(&geiger_counter_msgs[1]);
    geiger_counter_pack.cpm.data = cpm;
    // geiger_counter.publish(&geiger_counter_pack.cpm);
#endif

#ifdef SHOW_LOG_MONITOR
      Serial.print("minutes : ");      
      Serial.println(counts_minutes);
      Serial.print("cpm : ");     
      Serial.println(cpm);
#endif
    
    counts_minutes = 0;
  }

#if defined USE_PACK_DATA_GEIGER
  geiger_counter.publish(&geiger_counter_pack);
#endif

  nh.spinOnce();
}
