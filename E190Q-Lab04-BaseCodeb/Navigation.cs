﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double x, y, t;
        public double initialX, initialY, initialT;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double closeThresh = 0.01, thetaThresh = 0.05; // "close-enough" threshold   // lab 3
        public double delta_x, delta_y, rho, alpha, beta;   // lab 3
        public double desiredV, desiredW;                   // lab 3
        public double sonarRadius;  /* FIXME */
        public double IRRadius;     /* FIXME */
        public double laserMin;     /* ADDME */
        public double laserMax;     /* ADDME */

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        private double Krho = 16;//4.0;   //sim: 1         // Krho > 0 (Conditions for stability)
        private double Kalpha = 18;//4.5; //sim: 3.0       // Kalpha - Krho > 0
        private double Kbeta = -18;//-4.5; //sim: -2       // Kbeta < 0
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public double K_P = 14;//15; // lab 3
        public double K_I = 5;//0;   // lab 3
        public double K_D = 12;//3;   // lab 3

        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        // public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        const double maxTickSpeed = 802;
        // max speed 2.36 [m/s]*(1/(wheelRadius[m]*2*pi))*190[ticks/rev]
        const short simMaxTickSpeed = 125;   // lab 3
        // max speed 0.25 [m/s] = 85; same calculation as above
        const int realMaxTickSpeed = 30000;

        // PF Variables
        public Map map;
        public Particle[] particles;
        public Particle[] propagatedParticles;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;
        public double sigma = 500;  // estimated std for laser data (amped up. Should really be like: 0.02)

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {

            }
        }

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            propagatedParticles = new Particle[numParticles];

            this.Initialize();

            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {

            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                propagatedParticles[i] = new Particle();
            }

            // Initialize state estimates
            x = 1;//initialX;
            y = 1;//initialY;
            t = 1;//initialT;

            // Initialize state estimates
            x_est = 1;//initialX;
            y_est = 1;//initialY;
            t_est = 1;//initialT;

            initialX = x;   // Start locations for particles (when initial state is KNOWN)
            initialY = y;
            initialT = t;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            // jaguarControl.startMode = jaguarControl.UNKNOWN;   // Note: GUI checkbox initializes to KNOWN
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            // Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                // only if the robot has moved!
                if (diffEncoderPulseL != 0 || diffEncoderPulseR != 0)
                {
                    LocalizeEstWithParticleFilter();
                }

                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    // TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // making control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recent encoder measurements
                lastEncoderPulseL = currentEncoderPulseL;
                lastEncoderPulseR = currentEncoderPulseR;
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    //for (int i = 0; i < LaserData.Length; i=i+laserStepSize) // wat?
                    for (int i = 0; i < LaserData.Length; i = i + 1)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recent encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    lastEncoderPulseL = currentEncoderPulseL;
                    lastEncoderPulseR = currentEncoderPulseR;
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            // Cap off maximum motor signals if input speed exceeds 0.25 [m/s]:
            if ((Math.Abs(desiredRotRateL) > simMaxTickSpeed) && (Math.Abs(desiredRotRateL) > Math.Abs(desiredRotRateR)))
            {
                desiredRotRateR = (short)(Math.Sign(desiredRotRateR) * Math.Abs(((double)desiredRotRateR / (double)desiredRotRateL)) * simMaxTickSpeed);
                desiredRotRateL = (short)(Math.Sign(desiredRotRateL) * simMaxTickSpeed);
            }
            else if ((Math.Abs(desiredRotRateR) > simMaxTickSpeed) && (Math.Abs(desiredRotRateR) >= Math.Abs(desiredRotRateL)))
            {
                desiredRotRateL = (short)(Math.Sign(desiredRotRateL) * Math.Abs(((double)desiredRotRateL / (double)desiredRotRateR)) * simMaxTickSpeed);
                desiredRotRateR = (short)(Math.Sign(desiredRotRateR) * simMaxTickSpeed);
            }

            // apply motor signals:    
            motorSignalL = (short)(-desiredRotRateL);// was negative by JL
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // We will use the desiredRotRateRs to set our PWM signals
            int cur_e_R = (int)((double)desiredRotRateR - (diffEncoderPulseR / deltaT));   // [pulses/second]
            int cur_e_L = (int)((double)desiredRotRateL - (diffEncoderPulseL / deltaT));
            int e_dir_R = (int)((cur_e_R - e_R) / deltaT);
            int e_dir_L = (int)((cur_e_L - e_L) / deltaT);
            e_R = cur_e_R;
            e_L = cur_e_L;

            int maxErr = (int)(3000 / deltaT);

            Krho = 1.0;             // Krho > 0 (Conditions for stability)
            Kalpha = 3;//4          // Kalpha - Krho > 0
            Kbeta = -2;//-1.0;    // Kbeta < 0

            u_R = K_P * e_R + K_I * e_sum_R + K_D * e_dir_R;
            u_L = K_P * e_L + K_I * e_sum_L + K_D * e_dir_L;

            motorSignalL = (short)(zeroOutput - u_L); //* 300);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R); //* 100);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            e_sum_R = Math.Max(-maxErr, Math.Min(0.90 * e_sum_R + e_R * deltaT, maxErr));  // clever.
            e_sum_L = Math.Max(-maxErr, Math.Min(0.90 * e_sum_L + e_L * deltaT, maxErr));

        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Month.ToString() + "_"+ DateTime.Now.Day.ToString() + "_" + DateTime.Now.Year.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                 String newData = x.ToString() + ", " + y.ToString() + ", " + t.ToString() +
                    ", " + x.ToString() + ", " + y.ToString() + ", " + t.ToString() + ", "+ time.ToString();
                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

                double angVelL, angVelR;

                delta_x = desiredX - x_est;
                delta_y = desiredY - y_est;

                rho = Math.Sqrt((Math.Pow(delta_x, 2) + Math.Pow(delta_y, 2)));
                alpha = -t + Math.Atan2(delta_y, delta_x);

                desiredV = Krho * rho;

                // constrain calculated alpha:
                alpha = normalizeAngle(alpha);

                // If we need to turn more than 180 degrees, turn backwards!
                if (Math.Abs(alpha) > (Math.PI / 2))
                {
                    // recalculate alpha and desiredV to drive backwards:
                    alpha = -t + Math.Atan2(-delta_y, -delta_x);
                    // constrain calculated alpha:
                    alpha = normalizeAngle(alpha);

                    desiredV = (-Krho) * rho;
                }

                // state estimation eqn 1:
                beta = -t - alpha + desiredT;

                // constraint calculated beta:
                // alpha = normalizeAngle(alpha);
                beta = normalizeAngle(beta);

                //if (rho < closeThresh)
                //    beta = 0;

                // state estimation eqn 2:
                desiredW = (Kalpha * alpha) + (Kbeta * beta);

                angVelR = (desiredV / (2 * robotRadius)) + (desiredW / 2);
                angVelL = (-desiredV / (2 * robotRadius)) + (desiredW / 2);
                desiredRotRateR = (short)((((angVelR * 2 * robotRadius) / wheelRadius) / (2 * Math.PI)) * 190);    // [pulses/second]
                desiredRotRateL = (short)((((angVelL * 2 * robotRadius) / wheelRadius) / (2 * Math.PI)) * 190);

            // Kill motor signals if robot is "close enough" to target location:
            //if ((rho < closeThresh) && (alpha < thetaThresh))
            //{
            //    desiredRotRateR = 0;
            //    desiredRotRateL = 0;
            //}

            // ****************** Additional Student Code: End   ************
        }



        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()
        {
            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.

            // Calculate Encoder Differences:
            diffEncoderPulseR = currentEncoderPulseR - lastEncoderPulseR;
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;     

            // Check for Overflow and take the "inverted" difference if overflow occurred.

            if (diffEncoderPulseR < (-1 * maxTickSpeed))
                diffEncoderPulseR = currentEncoderPulseR + (encoderMax - lastEncoderPulseR);
            if (diffEncoderPulseR > maxTickSpeed)
                diffEncoderPulseR = -1 * (lastEncoderPulseR + (encoderMax - currentEncoderPulseR));

            if (diffEncoderPulseL < (-1 * maxTickSpeed))
                diffEncoderPulseL = currentEncoderPulseL + (encoderMax - lastEncoderPulseL);
            if (diffEncoderPulseL > maxTickSpeed)
                diffEncoderPulseL = -1 * (lastEncoderPulseL + (encoderMax - currentEncoderPulseL));

            // Calculate Linear wheel Distance travelled (in one DeltaT): [Lecture 3, Slide 16]
            wheelDistanceL = diffEncoderPulseL / pulsesPerRotation * 2 * Math.PI * wheelRadius;
            wheelDistanceR = -diffEncoderPulseR / pulsesPerRotation * 2 * Math.PI * wheelRadius;

            // Calculate angle traveled (in one DeltaT):
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2.0;
        }


        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.

        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x += distanceTravelled * Math.Cos(t + (angleTravelled / 2));
            y += distanceTravelled * Math.Sin(t + (angleTravelled / 2));

            t += angleTravelled;    // add angular displacement.

            // fit to range of -PI to +PI:
            t = normalizeAngle(t);

            // ****************** Additional Student Code: End   ************
        }




        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab

            double netParticleWeight = 0;
            // double littleRandomNoise = .1;
            double[] particleMaxGaussian =  new double[numParticles];
            double currentMax = 0;
            double currentEncoderPulseL_noise, currentEncoderPulseR_noise;

            double pDiffEncoderPulseR = 0;
            double pDiffEncoderPulseL = 0;
            double pWheelDistanceL, pWheelDistanceR;
            double pDistanceTravelled, pAngleTravelled;
           
            for (int i = 0; i < numParticles; i++)
            {
// add some noise to most recent encoder values:
                //currentEncoderPulseL_noise = currentEncoderPulseL + diffEncoderPulseL * (random.NextDouble() - 0.5);
                //currentEncoderPulseR_noise = currentEncoderPulseR + diffEncoderPulseR * (random.NextDouble() - 0.5);
                currentEncoderPulseL_noise = currentEncoderPulseL +  RandomGaussian() * K_wheelRandomness;
                currentEncoderPulseR_noise = currentEncoderPulseR +  RandomGaussian() * K_wheelRandomness;


// perform motion prediction on that particle:
                // Calculate Encoder Differences:
                pDiffEncoderPulseR = currentEncoderPulseR_noise - lastEncoderPulseR;
                pDiffEncoderPulseL = currentEncoderPulseL_noise - lastEncoderPulseL;

                // Check for Overflow and take the "inverted" difference if overflow occurred.
                if (pDiffEncoderPulseR < (-1 * maxTickSpeed))
                    pDiffEncoderPulseR = currentEncoderPulseR_noise + (encoderMax - lastEncoderPulseR);
                if (pDiffEncoderPulseR > maxTickSpeed)
                    pDiffEncoderPulseR = -1 * (lastEncoderPulseR + (encoderMax - currentEncoderPulseR_noise));

                if (pDiffEncoderPulseL < (-1 * maxTickSpeed))
                    pDiffEncoderPulseL = currentEncoderPulseL_noise + (encoderMax - lastEncoderPulseL);
                if (pDiffEncoderPulseL > maxTickSpeed)
                    pDiffEncoderPulseL = -1 * (lastEncoderPulseL + (encoderMax - currentEncoderPulseL_noise));

                // Calculate Linear wheel Distance travelled (in one DeltaT): [Lecture 3, Slide 16]
                pWheelDistanceL = pDiffEncoderPulseL / pulsesPerRotation * 2 * Math.PI * wheelRadius;
                pWheelDistanceR = -pDiffEncoderPulseR / pulsesPerRotation * 2 * Math.PI * wheelRadius;

                pWheelDistanceL += pWheelDistanceL* RandomGaussian() * K_wheelRandomness;
                pWheelDistanceR += pWheelDistanceR * RandomGaussian() * K_wheelRandomness;

                // Calculate angle traveled (in one DeltaT):
                pAngleTravelled = (pWheelDistanceR - pWheelDistanceL) / (2 * robotRadius);
                pDistanceTravelled = (pWheelDistanceL + pWheelDistanceR) / 2.0;

                // end addition

// update that particle's x,y,t:
                propagatedParticles[i].x = particles[i].x + pDistanceTravelled * Math.Cos(particles[i].t + (pAngleTravelled / 2));
                propagatedParticles[i].y = particles[i].y + pDistanceTravelled * Math.Sin(particles[i].t + (pAngleTravelled / 2));
                propagatedParticles[i].t = particles[i].t + pAngleTravelled;    // add angular displacement.

                // fit to range of -PI to +PI:
                propagatedParticles[i].t = normalizeAngle(propagatedParticles[i].t);

// Weight that particle:
                CalculateWeight(i);
                netParticleWeight += propagatedParticles[i].w;
            }

            for (int i = 0; i < numParticles; i++)
            {
                propagatedParticles[i].w /= netParticleWeight;    // normalize particle weight
                currentMax += propagatedParticles[i].w;
                particleMaxGaussian[i] = currentMax;
            }

            // Accounts for rounding error
            particleMaxGaussian[numParticles - 1] = 1;

            // Weighted random sampling
            for (int j = 0; j < numParticles; j++)
            {
                int index = 0;
                double uniformDist = random.NextDouble();

            // Find which range that Gauss falls within the propagated particle Gaussian:
                while (particleMaxGaussian[index] < uniformDist)
                {   index++;}

            // l new particle:
                particles[j].x = propagatedParticles[index].x;
                particles[j].y = propagatedParticles[index].y;
                particles[j].t = propagatedParticles[index].t;
            }

            // Part 8:
            // Now, compute the estimated pose as the average of all poses:
            x_est = 0; y_est = 0; t_est = 0;

            for (int k = 0; k < numParticles; k++)
            {
                x_est += particles[k].x;
                y_est += particles[k].y;
                t_est += particles[k].t;
            }

            x_est /= numParticles; y_est /= numParticles; t_est /= numParticles;


        }



        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.
        void CalculateWeight(int p)
        {
            double laserWeight, mu;
            propagatedParticles[p].w = 1;
            double currentAngle = 0;

            for (int i = 0; i < LaserData.Length; i = i + 30)       // LaserData.Length = 227 [long]
            {
                // mu is a single laser ray extending out at the x,y location of the robot:
                mu = map.GetClosestWallDistance(propagatedParticles[p].x, 
                                                propagatedParticles[p].y, 
                                                (propagatedParticles[p].t - 1.57 + laserAngles[i]));
                
                // because laserData[i] is calculated by map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i])

                // weight that laser ray value with a normal distribution:
                laserWeight = Math.Exp(-Math.Pow((((double)LaserData[i] / 1000.0) - mu), 2) / (2 * sigma * sigma));

                // overall probability is the product of each independent laser probability:
                propagatedParticles[p].w *= laserWeight;
            }
            /*
            mu = map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t);
            laserWeight = Math.Exp(-Math.Pow((((double)LaserData[113] / 1000.0) - mu), 2) / (2 * sigma * sigma));

            // overall probability is the product of each laser probability
            propagatedParticles[p].w *= laserWeight;
            */
        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos
        void InitializeParticles() {
	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.
        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)
            //do { particles[p].x = (RandomGaussian() * map.maxX); }
            particles[p].x = map.minX + (random.NextDouble() * (map.maxX - map.minX));
            particles[p].y = map.minY + (random.NextDouble() * (map.maxY - map.minY));
            
            particles[p].t = (random.NextDouble() * (2*Math.PI));
            particles[p].t -= Math.PI; // normalize to a range from -PI to PI

            propagatedParticles[p].x = particles[p].x;
            propagatedParticles[p].y = particles[p].y;
            propagatedParticles[p].t = particles[p].t;
            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random gaussian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }


/**********************************************************************
* Additional Functions:
* *******************************************************************/
/*
* function: normalizeAngle(double angleInput)
* returns: a value constrained from -PI to PI
*/
        public static double normalizeAngle(double angleInput)
        {
            double normalizedAngle = angleInput;

            // handle negative overshoot:
            if (normalizedAngle >= Math.PI)
            {
                while (normalizedAngle >= (Math.PI))
                    normalizedAngle -= (2 * Math.PI);
                return normalizedAngle;
            }

            // handle positive overshoot:
            else if (normalizedAngle <= (-Math.PI))
            {
                while (normalizedAngle <= (-Math.PI))
                    normalizedAngle += (2 * Math.PI);
                return normalizedAngle;
            }

            // handle no overshoot:
            else
                return normalizedAngle;
        }

        #endregion

    }
}
