/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The TalonSRX class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is a TimedRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Controls:
 * Button 1: When held, streams and fires the MP.  When released, contorl is back to PercentOutput Mode.
 * Button 2: Prints MP status to the console when held.
 * Left Joystick Y-Axis: Throttle Talon SRX forward and reverse when not running MP.
 * Gains for Motion Profile may need to be adjusted in Constants.java
 * 
 * Steps:
 * Drive the robot normally and confirm:
 *  - Open SmartDash (or similar) to see signal values.
 *  - Talon LEDs are green when robot moves straight forward (both sides)
 *  - sen_pos_drv moves in a positive direction when robot drives straight forward.  Or use Right Talon self-test (watch PID0 sensor pos)
 *  - sen_pos_turn moves in a positive direction when robot turns left.  Or use Right Talon self-test (watch PID1 aux sensor pos)
 *  - Update constants if not using Pigeon/CTRE Mag encoders.
 *
 * Supported Versions:
 * 	- Talon SRX: 4.X
 * 	- Victor SPX: 4.X
 * 	- Pigeon IMU: 4.13 (required for ribbon-cable usecase)
 * 	- CANifier: 4.X
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.ctre.phoenix.sensors.PigeonIMU;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Arrays;

import com.ctre.phoenix.motion.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;;

public class Robot extends TimedRobot {

    /** very simple state machine to prevent calling set() while firing MP. */
    int _state = 0;

    /** a master talon, add followers if need be. */
    WPI_TalonSRX _rightMaster = new WPI_TalonSRX(0);
    WPI_TalonSRX _leftMaster = new WPI_TalonSRX(1);

    /** gamepad for control */
    Joystick _joy = new Joystick(0);
    double[] arr = new double[3];

    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStreamLeft1 = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream _bufferedStreamRight1 = new BufferedTrajectoryPointStream();
    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStreamLeft2 = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream _bufferedStreamRight2 = new BufferedTrajectoryPointStream();
    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStreamLeft3 = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream _bufferedStreamRight3 = new BufferedTrajectoryPointStream();
    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStreamLeft4 = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream _bufferedStreamRight4 = new BufferedTrajectoryPointStream();
    /** new class type in 2019 for holding MP buffer. */
    BufferedTrajectoryPointStream _bufferedStreamLeft5 = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream _bufferedStreamRight5 = new BufferedTrajectoryPointStream();

    /* talon _config. */
    TalonSRXConfiguration _config = new TalonSRXConfiguration(); // factory default settings

    /* quick and dirty plotter to smartdash */
    PlotThread _plotThread = new PlotThread(_rightMaster);

    private DifferentialDrive _Drive = new DifferentialDrive(_leftMaster, _rightMaster);

    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    public void robotInit() {

        /*
         * fill our buffer object with the excel points, lets do a 90 deg turn while
         * using the profile for the robot drive
         */
        try {
            initBuffer(_bufferedStreamLeft1, MotionProfile.reader("1_left.csv"), MotionProfile.count("1_right.csv"),
                    0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }

        try {
            initBuffer(_bufferedStreamRight1, MotionProfile.reader("1_right.csv"),
                    MotionProfile.count("1_right.csv"), 0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }


        try {
            initBuffer(_bufferedStreamLeft2, MotionProfile.reader("2neg_left.csv"), MotionProfile.count("2neg_right.csv"),
                    0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }

        try {
            initBuffer(_bufferedStreamRight2, MotionProfile.reader("2neg_right.csv"),
                    MotionProfile.count("2neg_right.csv"), 0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }



        try {
            initBuffer(_bufferedStreamLeft3, MotionProfile.reader("3_left.csv"), MotionProfile.count("3_right.csv"),
                    0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }
        try {
            initBuffer(_bufferedStreamRight3, MotionProfile.reader("3_right.csv"),
                    MotionProfile.count("3_right.csv"), 0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }


        try {
            initBuffer(_bufferedStreamLeft4, MotionProfile.reader("4neg_left.csv"), MotionProfile.count("4neg_right.csv"),
                    0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }

        try {
            initBuffer(_bufferedStreamRight4, MotionProfile.reader("4neg_right.csv"),
                    MotionProfile.count("4neg_right.csv"), 0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }


        try {
            initBuffer(_bufferedStreamLeft5, MotionProfile.reader("5_left.csv"), MotionProfile.count("5_right.csv"),
                    0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }

        try {
            initBuffer(_bufferedStreamRight5, MotionProfile.reader("5_right.csv"),
                    MotionProfile.count("5_right.csv"), 0);
        } catch (IOException e) {
            System.out.println("initBuffer failed :(. Is your file in deploy?");
            e.printStackTrace();
        }

        /* -------------- config the master specific settings ----------------- */

        _config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        /* rest of the configs */
        _config.neutralDeadband = Constants.kNeutralDeadband; /* 0.1 % super small for best low-speed control */
        _config.slot0.kF = Constants.kGains_MotProf.kF;
        _config.slot0.kP = Constants.kGains_MotProf.kP;
        _config.slot0.kI = Constants.kGains_MotProf.kI;
        _config.slot0.kD = Constants.kGains_MotProf.kD;
        _config.slot0.integralZone = (int) Constants.kGains_MotProf.kIzone;
        _config.slot0.closedLoopPeakOutput = Constants.kGains_MotProf.kPeakOutput;
        _rightMaster.configAllSettings(_config);

        /* -------------- _config the left ----------------- */
        _leftMaster.configAllSettings(_config); /* no special configs */

        /* pick the sensor phase and desired direction */
        _rightMaster.setSensorPhase(false);
        _leftMaster.setSensorPhase(false);
        _rightMaster.setInverted(true); /* right side has to apply +V to M-, to go forward */

        /* speed up the target polling for PID[0] and PID-aux[1] */
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,
                20); /* plotthread is polling aux-pid-sensor-pos */
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        _rightMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);
        _leftMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1,
                20); /* plotthread is polling aux-pid-sensor-pos */
        _leftMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        _leftMaster.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 20);

    }

    public void robotPeriodic() {
        /* get joystick button and stick */
        Update_Limelight_Tracking();
        boolean bPrintValues = _joy.getRawButton(2);
        boolean bFireMp = _joy.getRawButton(1);
        double axis = -1.0 * _joy.getRawAxis(1); /* forward stick should be positive */
        double turn = -1.0 * _joy.getRawAxis(2); /* turn stick should be positive for turning left */

        /* if button is up, just drive the motor in PercentOutput */
        if (bFireMp == false) {
            _state = 0;
        }

        switch (_state) {
        /* drive master talon normally */
        case 0:
            /*
             * some clever use of the arbitratry feedforward to add the turn, there are many
             * alternative ways to do this
             */
            _Drive.arcadeDrive(axis, turn);
            if (bFireMp == true) {
                /* go to MP logic */
                _state = 1;
            }
            break;

        /* fire the MP, and stop calling set() since that will cancel the MP */
        case 1:
            ZeroAllSensors();
            _leftMaster.startMotionProfile(_bufferedStreamLeft1, 10, ControlMode.MotionProfile);
            _rightMaster.startMotionProfile(_bufferedStreamRight1, 10, ControlMode.MotionProfile);
            _state = 2;
            Instrum.printLine("MP started");
            break;

        /* wait for MP to finish */
        case 2:
            // if (_rightMaster.isMotionProfileFinished()) {
            if (_rightMaster.isMotionProfileFinished() && _leftMaster.isMotionProfileFinished()) {
                Instrum.printLine("MP finished");
                _state = 3;
            }
            break;

        case 3:
            ZeroAllSensors();
            _leftMaster.startMotionProfile(_bufferedStreamLeft2, 10, ControlMode.MotionProfile);
            _rightMaster.startMotionProfile(_bufferedStreamRight2, 10, ControlMode.MotionProfile);
            _state = 4;
            Instrum.printLine("MP started");
            break;

        /* wait for MP to finish */
        case 4:
            
            // if (_rightMaster.isMotionProfileFinished()) {
                if (_rightMaster.isMotionProfileFinished() && _leftMaster.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 5;
                }
            break;

        case 5:
            ZeroAllSensors();
            _leftMaster.startMotionProfile(_bufferedStreamLeft3, 10, ControlMode.MotionProfile);
            _rightMaster.startMotionProfile(_bufferedStreamRight3, 10, ControlMode.MotionProfile);
            _state = 6;
            Instrum.printLine("MP started");
            break;

        /* wait for MP to finish */
        case 6:
            // if (_rightMaster.isMotionProfileFinished()) {
            
            // if (_rightMaster.isMotionProfileFinished()) {
                if (_rightMaster.isMotionProfileFinished() && _leftMaster.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 7;
                }
            break;

        case 7:
            ZeroAllSensors();
            _leftMaster.startMotionProfile(_bufferedStreamLeft4, 10, ControlMode.MotionProfile);
            _rightMaster.startMotionProfile(_bufferedStreamRight4, 10, ControlMode.MotionProfile);
            _state = 8;
            Instrum.printLine("MP started");
            break;

        /* wait for MP to finish */
        case 8:
            // if (_rightMaster.isMotionProfileFinished()) {
            
            // if (_rightMaster.isMotionProfileFinished()) {
                if (_rightMaster.isMotionProfileFinished() && _leftMaster.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 9;
                }
            break;

        case 9:
            ZeroAllSensors();
            _leftMaster.startMotionProfile(_bufferedStreamLeft5, 10, ControlMode.MotionProfile);
            _rightMaster.startMotionProfile(_bufferedStreamRight5, 10, ControlMode.MotionProfile);
            _state = 10;
            Instrum.printLine("MP started");
            break;

        /* wait for MP to finish */
        case 10:
            // if (_rightMaster.isMotionProfileFinished()) {
            
            // if (_rightMaster.isMotionProfileFinished()) {
                if (_rightMaster.isMotionProfileFinished() && _leftMaster.isMotionProfileFinished()) {
                    Instrum.printLine("MP finished");
                    _state = 11;
                }
            break;

        /* MP is finished, nothing to do */
        case 11:
            break;
        }

        /* print MP values */
        Instrum.loop(bPrintValues, _rightMaster);
    }

    public void Update_Limelight_Tracking() {
        // These numbers must be tuned for your Robot! Be careful!
        final double STEER_K = 0.03; // how hard to turn toward the target
        final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0) {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE) {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
    }

    /**
     * Fill _bufferedStream with points from csv/generated-table.
     *
     * @param profile  generated array from excel
     * @param totalCnt num points in profile
     */
    private void initBuffer(BufferedTrajectoryPointStream bufferedStream, double[][] profile, int totalCnt,
            double finalTurnDeg) {

        boolean forward = true; // set to false to drive in opposite direction of profile (not really needed
                                // since you can use negative numbers in profile).

        TrajectoryPoint point = new TrajectoryPoint(); // temp for for loop, since unused params are initialized
                                                       // automatically, you can alloc just one

        /* clear the buffer, in case it was used elsewhere */
        bufferedStream.Clear();

        /* Insert every point into buffer, no limit on size */
        for (int i = 0; i < totalCnt; ++i) {

            double direction = forward ? +1 : -1;
            /* use the generated profile to figure out the forward arc path (translation) */
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            int durationMilliseconds = (int) profile[i][2];

            /*
             * to get the turn target, lets just scale from 0 deg to caller's final deg
             * linearizly
             */
            // double targetTurnDeg = finalTurnDeg * (i + 1) / totalCnt;

            /* for each point, fill our structure and pass it to API */
            point.timeDur = durationMilliseconds;

            /* drive part */
            point.position = direction * positionRot * Constants.kSensorUnitsPerRot; // Rotations => sensor units
            point.velocity = direction * velocityRPM * Constants.kSensorUnitsPerRot / 600.0; // RPM => units per 100ms
            point.arbFeedFwd = 0; // good place for kS, kV, kA, etc...

            /* turn part */
            // point.auxiliaryPos = targetTurnDeg * Constants.kTurnUnitsPerDeg; // Convert
            // deg to remote sensor units
            // point.auxiliaryVel = 0; // advanced teams can also provide the target
            // velocity
            // point.auxiliaryArbFeedFwd = 0; // good place for kS, kV, kA, etc...

            point.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            // point.profileSlotSelect1 = Constants.kAuxPIDSlot; /* auxiliary PID [0,1],
            // leave zero */
            point.zeroPos = false; /* don't reset sensor, this is done elsewhere since we have multiple sensors */
            point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
            point.useAuxPID = false; /* tell MPB that we are using both pids */

            bufferedStream.Write(point);
        }
    }

    void ZeroAllSensors() {
        /* individuall clear the quad register of each side */
        _leftMaster.getSensorCollection().setQuadraturePosition(0, 100);
        _rightMaster.getSensorCollection().setQuadraturePosition(0, 100);

    }
}
