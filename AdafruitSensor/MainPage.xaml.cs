using System;
using System.Diagnostics;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;
using System.Threading.Tasks;
using System.Runtime.Serialization.Json;
using System.Text;
using IotWeb.Server;
using IotWeb.Common.Util;
using IotWeb.Common.Http;
using Windows.UI.Core;
using Windows.System;

// Very small IMU https://www.tindie.com/products/femtoduino/femtobeacon-kit-basic/
// Sensor: http://www.ebay.com.au/itm/400757364389?_trksid=p2057872.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT
// 5v = RPi pin2 (5v), 3.3v = unconnected, Gnd = RPi pin6, SCL = RPi pin5, SDA = RPi pin3
// 10DOF L3GD20 LSM303D BMP180 Gyro Accelerometer Compass Altimeter For Arduino 
// http://www.winddeal.net/image/30191/30191-5.jpg
// 8ma current, 3.3v will bypass linear regulator
// BMP180 pressure sensor Bosch is phasing it out, use BMP280 instead. BMP280 has same relative accuracy +-1m but better absolute accuracy (we are after relative accuracy which is down to 100mm)
//      Pressure sensor is impacted by temperature & humidity, but less so for measuring relative pressure as temp affects density. Work out the mass of air based on desity between 2 sensors for relative readign.
//      Accuracy is impacted by noise but by averaging the readings more accuracy can be gained to 0.25m. Only accurate when measuring at heights lower than 3000m 
//      Ultra high resolution mode takes 25.5ms to sample. For even lower noise set the oversampling register but sampling takes 76ms.
//      https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-/measuring-weather-and-altitude
//      MS5611 $7/1000 Digikey is supposedly higher resolution than BMP280. http://electronics-from-t.blogspot.com.au/2016/02/lps25h-vs-ms5611-vs-bmp180.html but BMP280 is $1.50/1000 much cheaper



// webserver / websockets from https://github.com/sensaura-public/iotweb

//wifi http://192.168.137.1/sensor.html
//Dont setup a wifi profile or softAP won't be run
//softAP & administrator password p@ssw0rd
// ssh to pi via putty and enter the following commands
// reg add hklm\system\currentcontrolset\services\mpssvc\parameters /v IoTInboundLoopbackPolicy /t REG_DWORD /d 1
// checknetisolation loopbackexempt -a -n=IoTOnboardingTask-uwp_1w720vyc4ccym 
// reboot
// Set appxmanifest to enable internet and private client and servers

//BUG: Occassionally the wifi on the raspberry pi 3 won't be enabled when powering on. Reboot and its OK
//BUG: When starting VS to debug on the RPi for the first time, the app won't run. Restart the app and OK
//BUG: Due to power supply pull at power up, the wifi won't work sometimes, so put BOOT_DELAY=2 in CONFIG.TXT in the EFIESP directory to spread out the power draw on startup. Else reboot after a poweron
//TODO: Remote restart from web browser
//TODO: Autostart web server / sensors
//TODO: Move to local github directory
//TODO: Stop screen blanking
//TODO: Measure altitude
//TODO: Use moving averages not sample averages (use 8 samples then becomes easy to compute by right shifting divide)
//TODO: COnvert strings sent to browser to JSON
//TODO: Stop application breaking if client websocket gets disconnected
//TODO: Retry if the sensors can't be found "the opps text"
//TODO: Better version of web server
//TODO: Automatically work out if in landscape or portrait mode
//TODO: Compensate for hook spinning

// Accelerator angle calculation: http://www.nxp.com/docs/en/application-note/AN3461.pdf
// Compensation for yaw (spining around y axis for hook spin) http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf 

namespace AdafruitSensor
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {

        private BT myBT = new BT();

        // I2C Device
        private I2cDevice I2CDevbmp;
        private I2cDevice I2CDevgyro;
        private I2cDevice I2CDevaccel;
        private I2cDevice I2CDevmag;

        Adafruit_BMP085_Unified bmp;
        Adafruit_L3GD20_Unified gyro;
        Adafruit_LSM303_Accel_Unified accel;
        Adafruit_LSM303_Mag_Unified mag;

        private Adafruit_BMP085_Unified.sensor_t bmpSensor;
        private Adafruit_L3GD20_Unified.sensor_t gyroSensor;
        private Adafruit_LSM303_Accel_Unified.sensor_t accelSensor;
        private Adafruit_LSM303_Mag_Unified.sensor_t magSensor;

        private Adafruit_BMP085_Unified.sensors_event_t Pressevent;
        private Adafruit_L3GD20_Unified.sensors_event_t Gyroevent;
        private Adafruit_LSM303_Accel_Unified.sensors_event_t Accelevent;
        private Adafruit_LSM303_Mag_Unified.sensors_event_t Magevent;

        public bool started = false;

        WebSocketHandler mySocket = new WebSocketHandler();

        // Timer
        private DispatcherTimer ReadSensorTimer;
        HttpServer server1 = new HttpServer(80);

        public double Pi180 = 180 / Math.PI;


        public MainPage()
        {
            this.InitializeComponent();

            initSensors();

           // myBT.Connect();

            try
            {
                server1.AddHttpRequestHandler("/", new myHandler());
                server1.AddWebSocketRequestHandler(
                    "/sockets/",
                    mySocket
                    );
                server1.Start();
                mySocket.ConnectedEvent += WSConn;
            }
            catch (Exception ex)
            {
                var tt = ex;
                throw;
            }
        }

        public void WSConn()
        {
            var ignored = Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                // Start on the dispatcher thread
                Start();
            });
        }

        private async Task initSensors()
        {
            await InitializeAdF_BMP085_U_I2CDevice();
            await InitializeAdF_L3GD20_U_I2CDevice();
            await InitializeAdF_LSM303_Accel_I2CDevice();
            await InitializeAdF_LSM303_Mag_I2CDevice();
        }
        public int timerMS = 10;

        private async void Start()
        {
            if (!started)
            {
                started = true;
                if (!bmp.begin())
                {
                    /* There was a problem detecting the BMP085 ... check your connections */
                    Debug.WriteLine("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
                    while (true) ;
                }
                Pressevent = bmp.getSensorEventObj();

                if (!gyro.begin())
                {
                    /* There was a problem detecting the BMP085 ... check your connections */
                    Debug.WriteLine("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
                    while (true) ;
                }
                Gyroevent = gyro.getSensorEventObj();

                if (!accel.begin())
                {
                    /* There was a problem detecting the BMP085 ... check your connections */
                    Debug.WriteLine("Ooops, no LSM303 detected ... Check your wiring or I2C ADDR!");
                    while (true) ;
                }
                Accelevent = accel.getSensorEventObj();

                //if (!mag.begin())
                //{
                //    /* There was a problem detecting the BMP085 ... check your connections */
                //    Debug.WriteLine("Ooops, no LSM303 detected ... Check your wiring or I2C ADDR!");
                //    while (true) ;
                //}
                //Magevent = mag.getSensorEventObj();

                // Start Timer every 1 seconds
                //ReadSensorTimer = new DispatcherTimer();
                //ReadSensorTimer.Interval = TimeSpan.FromMilliseconds(timerMS);
                //ReadSensorTimer.Tick += Timer_Tick;
                //ReadSensorTimer.Start();

                Stopwatch loopDelay = Stopwatch.StartNew();
                Stopwatch sendDelay = Stopwatch.StartNew();
                string calcXStr, calcZStr, calcYStr;                
                try
                {
                    while (true)
                    {
                        accel.getEvent(ref Accelevent);
                        gyro.getEvent(ref Gyroevent);

                        //calcX = XkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.y, Accelevent.acceleration.z) * Pi180), Gyroevent.gyro.x, elapsed);
                        //calcY = YkalmanCalculate((float)(Math.Atan2(-Accelevent.acceleration.x, Math.Sqrt(Accelevent.acceleration.y * Accelevent.acceleration.y + Accelevent.acceleration.x * Accelevent.acceleration.z)) * Pi180), Gyroevent.gyro.y, elapsed);
                        //calcZ = ZkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.z, Accelevent.acceleration.y) * Pi180), Gyroevent.gyro.z, elapsed);
                        //COULD USE THE CALIBRATE FOR SQUARE ON THE LOAD THEN USE THE YAW (Z) TO MEASURE HOOK SPIN


                        calcXStr = String.Format("{0:0.0}", XkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.y, Accelevent.acceleration.z) * Pi180), Gyroevent.gyro.x, (int)loopDelay.ElapsedMilliseconds));
                        calcYStr = String.Format("{0:0.0}", YkalmanCalculate((float)(-Math.Atan2(Accelevent.acceleration.x, Math.Sqrt(Accelevent.acceleration.y * Accelevent.acceleration.y + Accelevent.acceleration.z * Accelevent.acceleration.z)) * Pi180), Gyroevent.gyro.y, (int)loopDelay.ElapsedMilliseconds));
                        //calcZStr = String.Format("{0:0.0}", ZkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.z, Accelevent.acceleration.y) * (180 / Math.PI)), Gyroevent.gyro.z, (int)loopDelay.ElapsedMilliseconds));
                        loopDelay.Restart();
                        if (sendDelay.ElapsedMilliseconds > 200)
                        {
                            sendDelay.Restart();
                            bmp.getEvent(ref Pressevent);
                            //bmp.getTemperature(ref temperature);
                            //float altitude = bmp.pressureToAltitude(Adafruit_BMP085_Unified.SENSORS_PRESSURE_SEALEVELHPA, Pressevent.pressure);
                            mySocket.Send(calcXStr + ":" + calcYStr + ":" + 0);
                            Status.Text = loopDelay.ElapsedMilliseconds.ToString();
                            AccelXaxisC.Text = calcXStr;
                            AccelYaxisC.Text = calcYStr;
                        }
                        await Task.Delay(10);
                    }
                }
                catch (Exception ex)
                {
                    var tt = ex;
                    throw;
                }

            }
        }

        private void MainPage_Unloaded(object sender, object args)
        {
            // Cleanup
            I2CDevbmp.Dispose();
            I2CDevgyro.Dispose();
            I2CDevmag.Dispose();
            I2CDevaccel.Dispose();
        }

        private async Task InitializeAdF_LSM303_Mag_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_LSM303_Mag_Unified.LSM303_ADDRESS_MAG);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevmag = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                mag = new Adafruit_LSM303_Mag_Unified(ref I2CDevmag, 30302);
                magSensor = mag.getSensorObj();
                mag.getSensor(ref magSensor);
                Debug.WriteLine("------------- MAGNETOMETER -----------");
                Debug.WriteLine("Sensor:       " + magSensor.name);
                Debug.WriteLine("Driver Ver:   " + magSensor.version);
                Debug.WriteLine("Unique ID:    " + magSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + magSensor.max_value + " uT");
                Debug.WriteLine("Min Value:    " + magSensor.min_value + " uT");
                Debug.WriteLine("Resolution:   " + magSensor.resolution + " uT");
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }
        }

        private async Task InitializeAdF_LSM303_Accel_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_LSM303_Accel_Unified.LSM303_ADDRESS_ACCEL);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevaccel = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                accel = new Adafruit_LSM303_Accel_Unified(ref I2CDevaccel, 30301);
                accelSensor = accel.getSensorObj();
                accel.getSensor(ref accelSensor);
                Debug.WriteLine("------------- ACCELEROMETER -----------");
                Debug.WriteLine("Sensor:       " + accelSensor.name);
                Debug.WriteLine("Driver Ver:   " + accelSensor.version);
                Debug.WriteLine("Unique ID:    " + accelSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + accelSensor.max_value + " m/s^2");
                Debug.WriteLine("Min Value:    " + accelSensor.min_value + " m/s^2");
                Debug.WriteLine("Resolution:   " + accelSensor.resolution + " m/s^2");
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }
        }

        private async Task InitializeAdF_L3GD20_U_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_L3GD20_Unified.L3GD20_ADDRESS);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevgyro = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                gyro = new Adafruit_L3GD20_Unified(ref I2CDevgyro, 20);
                gyroSensor = gyro.getSensorObj();
                gyro.getSensor(ref gyroSensor);
                Debug.WriteLine("------------- GYROSCOPE -----------");
                Debug.WriteLine("Sensor:       " + gyroSensor.name);
                Debug.WriteLine("Driver Ver:   " + gyroSensor.version);
                Debug.WriteLine("Unique ID:    " + gyroSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + gyroSensor.max_value + " rad/s");
                Debug.WriteLine("Min Value:    " + gyroSensor.min_value + " rad/s");
                Debug.WriteLine("Resolution:   " + gyroSensor.resolution + " rad/s");
            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }
        }

        private async Task InitializeAdF_BMP085_U_I2CDevice()
        {
            try
            {
                // Initialize I2C device
                var settings = new I2cConnectionSettings(Adafruit_BMP085_Unified.BMP085_ADDRESS);

                settings.BusSpeed = I2cBusSpeed.FastMode;
                settings.SharingMode = I2cSharingMode.Shared;

                string aqs = I2cDevice.GetDeviceSelector("I2C1");  /* Find the selector string for the I2C bus controller                   */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller device with our selector string           */

                I2CDevbmp = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings */

                bmp = new Adafruit_BMP085_Unified(ref I2CDevbmp, 18001);
                bmpSensor = bmp.getSensorObj();
                bmp.getSensor(ref bmpSensor);
                Debug.WriteLine("-------- PRESSURE/ALTITUDE ---------");
                Debug.WriteLine("Sensor:       " + bmpSensor.name);
                Debug.WriteLine("Driver Ver:   " + bmpSensor.version);
                Debug.WriteLine("Unique ID:    " + bmpSensor.sensor_id);
                Debug.WriteLine("Max Value:    " + bmpSensor.max_value + " hPa");
                Debug.WriteLine("Min Value:    " + bmpSensor.min_value + " hpa");
                Debug.WriteLine("Resolution:   " + bmpSensor.resolution + " hPa");

            }
            catch (Exception e)
            {
                Debug.WriteLine(e.ToString());

                return;
            }
        }

        public int timerCnt = 1;
        public float gyroXangle = 0;
        public float gyroYangle = 0;
        public float gyroZangle = 0;

        public float calcX = 0;
        public float calcY = 0;
        public float calcZ = 0;
        Stopwatch tt = Stopwatch.StartNew();


        // Timer loop
        //NOT USED **********************************************************
        private void Timer_Tick(object sender, object e)
        {
            var xx = Stopwatch.StartNew();
            /* Display the results (acceleration is measured in m/s^2) */
            accel.getEvent(ref Accelevent);

            //Debug.WriteLine("ACCEL  ");
            //Debug.WriteLine("X: " + Accelevent.acceleration.x + "  " + "Y: " + Accelevent.acceleration.y + "  " + "Z: " + Accelevent.acceleration.z + "  m/s^2");

            /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
            //mag.getEvent(ref Magevent);
            //Debug.WriteLine("MAG  ");
            //Debug.WriteLine("X: " + Magevent.magnetic.x + "  " + "Y: " + Magevent.magnetic.y + "  " + "Z: " + Magevent.magnetic.z + "  uT");
            //MagXaxis.Text = Magevent.magnetic.x.ToString();
            //MagYaxis.Text = Magevent.magnetic.y.ToString();
            //MagZaxis.Text = Magevent.magnetic.z.ToString();

            /* Display the results (gyrocope values in rad/s) */
            gyro.getEvent(ref Gyroevent);
            //gyroXangle += (Gyroevent.gyro.x * timerMS / 1000);
            //gyroYangle += (Gyroevent.gyro.y * timerMS / 1000);
            //gyroZangle += (Gyroevent.gyro.z * timerMS / 1000);
            //Debug.WriteLine("GYRO  ");
            //Debug.WriteLine("X: " + Gyroevent.gyro.x + "  " + "Y: " + Gyroevent.gyro.y + "  " + "Z: " + Gyroevent.gyro.z + "  rad/s");
            int elapsed = (int)tt.ElapsedMilliseconds;
            tt.Restart();
            calcX = XkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.y, Accelevent.acceleration.z) * Pi180), Gyroevent.gyro.x, elapsed);
            calcY = YkalmanCalculate((float)(Math.Atan2(-Accelevent.acceleration.x, Math.Sqrt(Accelevent.acceleration.y * Accelevent.acceleration.y + Accelevent.acceleration.x * Accelevent.acceleration.z)) * Pi180), Gyroevent.gyro.y, elapsed);
            //calcZ = ZkalmanCalculate((float)(Math.Atan2(Accelevent.acceleration.z, Accelevent.acceleration.y) * Pi180), Gyroevent.gyro.z, elapsed);
            //COULD USE THE CALIBRATE FOR SQUARE ON THE LOAD THEN USE THE YAW (Z) TO MEASURE HOOK SPIN

            calcZ = 0;
            //calcX = 0.98F * (calcX + Gyroevent.gyro.z * timerMS * SCALE_GYR_ANGLE / 1000F) + 0.02F * (float)(90 - Math.Atan2(Accelevent.acceleration.y, Accelevent.acceleration.x) * (180 / Math.PI));
            //calcY = 0;
            //calcZ = 0.98F * (calcZ + Gyroevent.gyro.z * timerMS * SCALE_GYR_ANGLE / 1000F) + 0.02F * (float)(Math.Atan2(Accelevent.acceleration.z, Accelevent.acceleration.y) * (180 / Math.PI));

            if (timerCnt == 5)
            {
                //float AccX = (AccXAvg / 4) * SCALE_ACC_ANGLE;
                //float AccY = (AccYAvg / 4) * SCALE_ACC_ANGLE;
                //float AccZ = (AccZAvg / 4) * SCALE_ACC_ANGLE;

                //var AccX = 90 - Math.Atan2(AccYAvg / 4, AccXAvg / 4) * (180 / Math.PI);
                //var AccZ = Math.Atan2(AccZAvg / 4, AccYAvg / 4) * (180 / Math.PI);
                //var AccY = 0;

                //AccelXaxis.Text = String.Format("{0:0.0}", AccX);
                //AccelYaxis.Text = String.Format("{0:0.0}", AccY);
                //AccelZaxis.Text = String.Format("{0:0.0}", AccZ);

                AccelXaxisC.Text = String.Format("{0:0.0}", calcX);
                AccelYaxisC.Text = String.Format("{0:0.0}", calcY);
                AccelZaxisC.Text = String.Format("{0:0.0}", calcZ);

                //GyroXaxis.Text = String.Format("{0:0.0}", gyroXangle);
                //GyroYaxis.Text = String.Format("{0:0.0}", gyroYangle);
                //GyroZaxis.Text = String.Format("{0:0.0}", gyroZangle);
                //GyroXaxis.Text = Gyroevent.gyro.x.ToString();
                //GyroYaxis.Text = Gyroevent.gyro.y.ToString();
                //GyroZaxis.Text = Gyroevent.gyro.z.ToString();

                /* Display ambient temperature in C */
                float temperature = 0;
                //bmp.getTemperature(ref temperature);
                //Temperature.Text = temperature.ToString();

                /* Display the pressure sensor results (barometric pressure is measure in hPa) */
                //////////////////////////////bmp.getEvent(ref Pressevent);
                /* Display atmospheric pressure in hPa */
                //Debug.WriteLine("PRESS :" + Pressevent.pressure + " hPa ");
                Pressure.Text = Pressevent.pressure.ToString();
                /* Then convert the atmospheric pressure, SLP and temp to altitude    */
                /* Update this next line with the current SLP for better results      */
                float seaLevelPressure = Adafruit_BMP085_Unified.SENSORS_PRESSURE_SEALEVELHPA;
                float altitude = bmp.pressureToAltitude(seaLevelPressure, Pressevent.pressure, temperature);
                //Debug.WriteLine("ALTI :" + altitude + " m " );
                Altitude.Text = altitude.ToString();
                //DateTime.Text = System.DateTime.Now.ToString();

                mySocket.Send(calcX + ":" + calcY + ":" + calcZ + ":" + (-1 * gyroXangle).ToString() + ":" + (-1 * gyroYangle).ToString() + ":" + (-1 * gyroZangle).ToString() + ":" + altitude.ToString());

                //AccXAvg = 0;
                //AccYAvg = 0;
                //AccZAvg = 0;
                timerCnt = 0;
                Status.Text = elapsed.ToString();
            }
            timerCnt++;
        }

        float tau = 0.075F;
        float a = 0.0F;
        float x_angleC = 0;
        float y_angleC = 0;
        float z_angleC = 0;

        float XComplementary(float newAngle, float newRate, int looptime)
        {
            float dtC = (float)looptime / 1000.0F;
            a = tau / (tau + dtC);
            x_angleC = a * (x_angleC + newRate * dtC) + (1 - a) * (newAngle);
            return x_angleC;
        }

        float YComplementary(float newAngle, float newRate, int looptime)
        {
            float dtC = (float)looptime / 1000.0F;
            a = tau / (tau + dtC);
            y_angleC = a * (y_angleC + newRate * dtC) + (1 - a) * (newAngle);
            return y_angleC;
        }

        float ZComplementary(float newAngle, float newRate, int looptime)
        {
            float dtC = (float)looptime / 1000.0F;
            a = tau / (tau + dtC);
            z_angleC = a * (z_angleC + newRate * dtC) + (1 - a) * (newAngle);
            return z_angleC;
        }

        // KasBot V1 - Kalman filter module

        float Q_angle = 0.001F; //0.001   Q_angle and Q_gyro are inverse of standard deviation
        float Q_gyro = 0.003F;  //0.003
        float R_angle = 0.03F;  //0.03   R_angle is convergence factor, the smaller - the faster we converge from gyro towards accelerometer angle

        float x_bias = 0;
        float y_bias = 0;
        float z_bias = 0;
        float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
        float Xy, XS;
        float XK_0, XK_1;
        float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
        float Yy, YS;
        float YK_0, YK_1;
        float ZP_00 = 0, ZP_01 = 0, ZP_10 = 0, ZP_11 = 0;
        float Zy, ZS;
        float ZK_0, ZK_1;
        float x_angle;
        float y_angle;
        float z_angle;

        // newAngle = angle measured with atan2 using the accelerometer
        // newRate = angle measured using the gyro
        // looptime = loop time in millis()

        

        float XkalmanCalculate(float newAngle, float newRate, int looptime)
        {
            float dt = (float)looptime / 1000;
            x_angle += dt * (newRate - x_bias);
            XP_00 += -dt * (XP_10 + XP_01) + Q_angle * dt;
            XP_01 += -dt * XP_11;
            XP_10 += -dt * XP_11;
            XP_11 += +Q_gyro * dt;

            Xy = newAngle - x_angle;
            XS = XP_00 + R_angle;
            XK_0 = XP_00 / XS;
            XK_1 = XP_10 / XS;

            x_angle += XK_0 * Xy;
            x_bias += XK_1 * Xy;
            XP_00 -= XK_0 * XP_00;
            XP_01 -= XK_0 * XP_01;
            XP_10 -= XK_1 * XP_00;
            XP_11 -= XK_1 * XP_01;

            return x_angle;
        }

        float YkalmanCalculate(float newAngle, float newRate, int looptime)
        {
            float dt = (float)looptime / 1000;
            y_angle += dt * (newRate - y_bias);
            YP_00 += -dt * (YP_10 + YP_01) + Q_angle * dt;
            YP_01 += -dt * YP_11;
            YP_10 += -dt * YP_11;
            YP_11 += +Q_gyro * dt;

            Yy = newAngle - y_angle;
            YS = YP_00 + R_angle;
            YK_0 = YP_00 / YS;
            YK_1 = YP_10 / YS;

            y_angle += YK_0 * Yy;
            y_bias += YK_1 * Yy;
            YP_00 -= YK_0 * YP_00;
            YP_01 -= YK_0 * YP_01;
            YP_10 -= YK_1 * YP_00;
            YP_11 -= YK_1 * YP_01;

            return y_angle;
        }

        float ZkalmanCalculate(float newAngle, float newRate, int looptime)
        {
            float dt = (float)looptime / 1000;
            z_angle += dt * (newRate - z_bias);
            ZP_00 += -dt * (ZP_10 + ZP_01) + Q_angle * dt;
            ZP_01 += -dt * ZP_11;
            ZP_10 += -dt * ZP_11;
            ZP_11 += +Q_gyro * dt;

            Zy = newAngle - z_angle;
            ZS = ZP_00 + R_angle;
            ZK_0 = ZP_00 / ZS;
            ZK_1 = ZP_10 / ZS;

            z_angle += ZK_0 * Zy;
            z_bias += ZK_1 * Zy;
            ZP_00 -= ZK_0 * ZP_00;
            ZP_01 -= ZK_0 * ZP_01;
            ZP_10 -= ZK_1 * ZP_00;
            ZP_11 -= ZK_1 * ZP_01;

            return z_angle;
        }

        private void Exit_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Exit();
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            Start();
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            Shutdown();
        }

        private void Shutdown()
        {
            ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
        }

    }

    class WebSocketHandler : IWebSocketRequestHandler
    {

        public delegate void SignalDel();
        public event SignalDel ConnectedEvent;

        WebSocket classSocket;
        public bool inSess = false;

        public bool WillAcceptRequest(string uri, string protocol)
        {
            return (uri.Length == 0) && (protocol == "");
        }

        public void Connected(WebSocket socket)
        {
            classSocket = socket;
            socket.DataReceived += OnDataReceived;
            socket.ConnectionClosed += WSClose;
            inSess = true;
            ConnectedEvent();
        }

        public void Send(string myData)
        {
            if (inSess)
            {
                try
                {
                    classSocket.Send(myData);
                }
                catch (Exception)
                {

                    //throw;
                }
            }
        }

        // Browser closed session
        void WSClose(WebSocket socket)
        {
            inSess = false;
        }

        // Browser sends something back
        void OnDataReceived(WebSocket socket, string frame)
        {
            switch (frame)
            {
                case "Client Connected":
                    break;
                case "shutdown":
                    ShutdownManager.BeginShutdown(ShutdownKind.Shutdown, new TimeSpan(0));
                    break;
            }
        }
    }

    // web server handler
    public class myHandler : IotWeb.Common.Http.IHttpRequestHandler
    {
        public void HandleRequest(string uri, HttpRequest request, HttpResponse response, HttpContext context)
        {
            byte[] responseBytes;
            //UTF8Encoding utf8 = new UTF8Encoding();
            responseBytes = File.ReadAllBytes(@"Assets\sensor.html");
            response.Headers[HttpHeaders.ContentType] = MimeType.FromExtension(@"Assets\sensor.html");
            response.Content.Write(responseBytes, 0, responseBytes.Length);
        }
    }

}
