// define must ahead #include <M5Stack.h>
//#define M5STACK_MPU6886
#define M5STACK_MPU9250
// #define M5STACK_MPU6050
// #define M5STACK_200Q
#define MEAN_POINT_IMU   16    //nb point for mean meas calculation
#define PERIOD_WAIT_DISPLAY  20  //wait 1000ms for counter

#include <M5Stack.h>

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

// interval envoi status toute les 60 secondes
unsigned long ulong_time_now      = 0;   // will store last time LED was updated
unsigned long ulong_old_time_now      = 0;
unsigned long ulong_wait_time_display     = 0;
unsigned int int_delta_time = 0;

int int_mat_accx[MEAN_POINT_IMU];
int int_mat_accy[MEAN_POINT_IMU];
int int_mat_accz[MEAN_POINT_IMU];
int int_mat_gyrx[MEAN_POINT_IMU];
int int_mat_gyry[MEAN_POINT_IMU];
int int_mat_gyrz[MEAN_POINT_IMU];

int int_indice_imu = 0;

int min_accx = 30000;
int min_accy = 30000;
int min_accz = 30000;
int max_accx = -30000;
int max_accy = -30000;
int max_accz = -30000;
int min_max_accx = 0;
int min_max_accy = 0;
int min_max_accz = 0;

int min_gyrx = 30000;
int min_gyry = 30000;
int min_gyrz = 30000;
int max_gyrx = -30000;
int max_gyry = -30000;
int max_gyrz = -30000;
int min_max_gyrx = 0;
int min_max_gyry = 0;
int min_max_gyrz = 0;


// the setup routine runs once when M5Stack starts up
void setup() {

  // Initialize the M5Stack object
  M5.begin();
  /*
    Power chip connected to gpio21, gpio22, I2C device
    Set battery charging voltage and current
    If used battery, please call this function in your project
  */
  M5.Power.begin();

  Serial.begin(115200);  //2000000 start serial for output
  delay(10);
  Serial.println("Start programm.");


  M5.IMU.Init();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(2);
}

// the loop routine runs over and over again forever
void loop() {

  //period with millis()
  ulong_time_now = millis();
  int_delta_time = ulong_time_now - ulong_old_time_now;
  ulong_old_time_now = millis();

  // put your main code here, to run repeatedly:
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.IMU.getAccelData(&accX, &accY, &accZ);
  // M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  // M5.IMU.getTempData(&temp);

  int_mat_accx[int_indice_imu % MEAN_POINT_IMU] = (int)( accX * 1000);
  int_mat_accy[int_indice_imu % MEAN_POINT_IMU] = (int)( accY * 1000);
  int_mat_accz[int_indice_imu % MEAN_POINT_IMU] = (int)( accZ * 1000);

  int_mat_gyrx[int_indice_imu % MEAN_POINT_IMU] = (int)(  gyroX * 1000);
  int_mat_gyry[int_indice_imu % MEAN_POINT_IMU] = (int)( gyroY * 1000);
  int_mat_gyrz[int_indice_imu % MEAN_POINT_IMU] = (int)(  gyroZ * 1000);

  //calcul min max
  int_min_max_acc_gyr();

  int_indice_imu++;


  //wait time
  if ((ulong_time_now >= ulong_wait_time_display))
  {
    ulong_wait_time_display = millis() + PERIOD_WAIT_DISPLAY;

    //display
    //  M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("%6.2d  %6.2d  %6.2d      ", min_max_gyrx, min_max_gyry, min_max_gyrz);
    M5.Lcd.setCursor(100, 42);
    M5.Lcd.print(" max-min mdps");
    M5.Lcd.setCursor(0, 65);
    M5.Lcd.printf(" %5.2d  %5.2d   %5.2d   ", min_max_accx, min_max_accy, min_max_accz);
    M5.Lcd.setCursor(100, 87);
    M5.Lcd.print(" max-min mg");
    //  M5.Lcd.setCursor(0, 110);
    //  M5.Lcd.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);
    //  M5.Lcd.setCursor(220, 132);
    //  M5.Lcd.print(" degree");
    //  M5.Lcd.setCursor(0, 155);
    //  M5.Lcd.printf("Temperature : %.2f C", temp);

    M5.Lcd.setCursor(10, 110);
    M5.Lcd.printf("timestamp: %5.2d               ", ulong_time_now);

    M5.Lcd.setCursor(10, 155);
    M5.Lcd.printf(" cycle : %.2d ms           ", int_delta_time);

    delay(1);
  
        Serial.print(min_max_accx);
        Serial.print(",");
        Serial.print(min_max_accy);
        Serial.print(",");
        Serial.print(min_max_accz);
        Serial.println(",");
  }
}



/*
     _____  .__            _____                  .___   _____   ____ ___
    /     \ |__| ____     /     \ _____  ___  ___ |   | /     \ |    |   \
   /  \ /  \|  |/    \   /  \ /  \\__  \ \  \/  / |   |/  \ /  \|    |   /
  /    Y    \  |   |  \ /    Y    \/ __ \_>    <  |   /    Y    \    |  /
  \____|__  /__|___|  / \____|__  (____  /__/\_ \ |___\____|__  /______/
          \/        \/          \/     \/      \/             \/
*/
void int_min_max_acc_gyr(void)
{
  //reset value
  init_value_imu();

  //ACCX
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_accx > int_mat_accx[z]) min_accx = int_mat_accx[z];
    if ( max_accx < int_mat_accx[z]) max_accx = int_mat_accx[z];
  }
  min_max_accx = (max_accx - min_accx);

  //ACCY
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_accy > int_mat_accy[z]) min_accy = int_mat_accy[z];
    if ( max_accy < int_mat_accy[z]) max_accy = int_mat_accy[z];

  }
  min_max_accy = (max_accy - min_accy);

  //ACCZ
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_accz > int_mat_accz[z]) min_accz = int_mat_accz[z];
    if ( max_accz < int_mat_accz[z]) max_accz = int_mat_accz[z];

  }
  min_max_accz = (max_accz - min_accz);

  //GYRX
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_gyrx > int_mat_gyrx[z]) min_gyrx = int_mat_gyrx[z];
    if ( max_gyrx < int_mat_gyrx[z]) max_gyrx = int_mat_gyrx[z];
  }
  min_max_gyrx = (max_gyrx - min_gyrx) ;

  //GYRY
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_gyry > int_mat_gyry[z]) min_gyry = int_mat_gyry[z];
    if ( max_gyry < int_mat_gyry[z]) max_gyry = int_mat_gyry[z];
  }
  min_max_gyry = (max_gyry - min_gyry) ;

  //GYRZ
  for (int z = 0; z < MEAN_POINT_IMU; z++)
  {
    if ( min_gyrz > int_mat_gyrz[z]) min_gyrz = int_mat_gyrz[z];
    if ( max_gyrz < int_mat_gyrz[z]) max_gyrz = int_mat_gyrz[z];
  }
  min_max_gyrz = (max_gyrz - min_gyrz) ;

}


void init_value_imu(void)
{
  min_accx = 30000;
  min_accy = 30000;
  min_accz = 30000;
  max_accx = -30000;
  max_accy = -30000;
  max_accz = -30000;
  min_max_accx = 0;
  min_max_accy = 0;
  min_max_accz = 0;

  min_gyrx = 30000;
  min_gyry = 30000;
  min_gyrz = 30000;
  max_gyrx = -30000;
  max_gyry = -30000;
  max_gyrz = -30000;
  min_max_gyrx = 0;
  min_max_gyry = 0;
  min_max_gyrz = 0;
}
