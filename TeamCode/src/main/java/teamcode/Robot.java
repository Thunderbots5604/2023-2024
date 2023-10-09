/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcFtcLib.ftclib.FtcDashboard;
import TrcFtcLib.ftclib.FtcDcMotor;
import TrcFtcLib.ftclib.FtcMatchInfo;
import TrcFtcLib.ftclib.FtcMotorActuator;
import TrcFtcLib.ftclib.FtcOpMode;
import TrcFtcLib.ftclib.FtcRobotBattery;
import teamcode.drivebases.MecanumDrive;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.subsystems.Grabber;
import teamcode.vision.Vision;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    //
    // Global objects.
    //
    public FtcOpMode opMode;
    public FtcDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public FtcDcMotor elevator;
    public FtcDcMotor arm;
    public Grabber grabber = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        dashboard = FtcDashboard.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        checkRobotSupport();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if (RobotParams.Preferences.tuneColorBlobVision ||
            RobotParams.Preferences.useAprilTagVision ||
            RobotParams.Preferences.useColorBlobVision ||
            RobotParams.Preferences.useTensorFlowVision)
        {
            vision = new Vision(this, null);
        }
        //
        // If noRobot is true, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (!RobotParams.Preferences.noRobot)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new BlinkinLEDs(RobotParams.HWNAME_BLINKIN);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize RobotDrive.
            //
            robotDrive = RobotParams.Preferences.swerveRobot? new SwerveDrive(): new MecanumDrive();
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useElevator)
                {
                    FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams()
                        .setMotorInverted(RobotParams.ELEVATOR_MOTOR_INVERTED)
                        .setLowerLimitSwitchEnabled(RobotParams.ELEVATOR_HAS_LOWER_LIMIT_SWITCH,
                                                    RobotParams.ELEVATOR_LOWER_LIMIT_INVERTED)
                        .setUpperLimitSwitchEnabled(RobotParams.ELEVATOR_HAS_UPPER_LIMIT_SWITCH,
                                                    RobotParams.ELEVATOR_UPPER_LIMIT_INVERTED)
                        .setPositionScaleAndOffset(RobotParams.ELEVATOR_INCHES_PER_COUNT, RobotParams.ELEVATOR_OFFSET)
                        .setPositionPresets(RobotParams.ELEVATOR_PRESET_TOLERANCE, RobotParams.ELEVATOR_PRESETS);
                    elevator = new FtcMotorActuator(
                        RobotParams.HWNAME_ELEVATOR, motorParams, globalTracer, false).getMotor();
                }

                if (RobotParams.Preferences.useArm)
                {
                    FtcMotorActuator.MotorParams motorParams = new FtcMotorActuator.MotorParams()
                        .setMotorInverted(RobotParams.ARM_MOTOR_INVERTED)
                        .setLowerLimitSwitchEnabled(RobotParams.ARM_HAS_LOWER_LIMIT_SWITCH,
                                                    RobotParams.ARM_LOWER_LIMIT_INVERTED)
                        .setUpperLimitSwitchEnabled(RobotParams.ARM_HAS_UPPER_LIMIT_SWITCH,
                                                    RobotParams.ARM_UPPER_LIMIT_INVERTED)
                        .setPositionScaleAndOffset(RobotParams.ARM_DEG_PER_COUNT, RobotParams.ARM_OFFSET)
                        .setPositionPresets(RobotParams.ARM_PRESET_TOLERANCE, RobotParams.ARM_PRESETS);
                    arm = new FtcMotorActuator(
                        RobotParams.HWNAME_ARM, motorParams, globalTracer, false).getMotor();
                    dashboard.displayPrintf(5, "Arm: PID=%s", arm.getMotorPositionPidCoefficients());
                }

                if (RobotParams.Preferences.useGrabber)
                {
                    grabber = new Grabber(RobotParams.HWNAME_GRABBER, globalTracer);
                }
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    }   //toString

    private void checkRobotSupport()
    {
        if (RobotParams.Preferences.swerveRobot)
        {
            RobotParams.Preferences.useBlinkin = false;
            RobotParams.Preferences.useExternalOdometry = false;
            RobotParams.Preferences.useSubsystems = false;
        }
        else if (RobotParams.Preferences.powerPlayRobot)
        {
            RobotParams.Preferences.useBlinkin = true;
            RobotParams.Preferences.useExternalOdometry = true;
            RobotParams.Preferences.useSubsystems = true;
        }
        else
        {
        }
    }   //checkRobotSupport

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "startMode";

        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode != TrcRobot.RunMode.AUTO_MODE)
            {
                // In TeleOp or Test mode. If we are not running a competition match, autonomous may not have run
                // prior to this. Therefore, we cannot inherit the robot position from previous autonomous mode.
                // In this case, we will just assume previous robot start position.
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(funcName, "Restore saved RobotPose=%s", endOfAutoRobotPose);
                }
                else
                {
                    // There was no saved robotPose, use previous autonomous start position. In case we didn't even
                    // have a previous autonomous run (e.g. just powering up the robot and go into TeleOp), then we
                    // will default to RED_LEFT starting position which is the AutoChoices default.
                    robotDrive.setAutoStartPosition(FtcAuto.autoChoices);
                    globalTracer.traceInfo(
                        funcName, "No saved RobotPose, use autoChoiceStartPos=%s",
                        robotDrive.driveBase.getFieldPosition());
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        //
        // The following are performance counters, could be disabled for competition if you want.
        // But it might give you some insight if somehow autonomous wasn't performing as expected.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.setElapsedTimerEnabled(true);
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopMode";
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.purplePixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling PurplePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.PurplePixel, false);
            }

            if (vision.greenPixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling GreenPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.GreenPixel, false);
            }

            if (vision.yellowPixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling YellowPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.YellowPixel, false);
            }

            if (vision.whitePixelVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling WhitePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.WhitePixel, false);
            }

            if (vision.redConeVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling RedConeVision.");
                vision.setRedConeVisionEnabled(false);
            }

            if (vision.blueConeVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling BlueConeVision.");
                vision.setBlueConeVisionEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(funcName, "Disabling TensorFlowVision.");
                vision.setTensorFlowVisionEnabled(false);
            }
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(funcName, "Saved robot pose=%s", endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (elevator != null)
        {
            elevator.zeroCalibrate(owner, RobotParams.ELEVATOR_CAL_POWER);
        }

        if (arm != null)
        {
            arm.zeroCalibrate(owner, RobotParams.ARM_CAL_POWER);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
