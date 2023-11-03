/*This just does TeleOp for now, actually write things later*/


package org.firstinspires.ftc.teamcode.mecanumdrive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

public class Drive {
    //Motors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    //power calculations
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    //power multipliers
    private final double flPowerMult;
    private final double frPowerMult;
    private final double blPowerMult;
    private final double brPowerMult;
    //tick multipliers
    private final double flTickMult;
    private final double frTickMult;
    private final double blTickMult;
    private final double brTickMult;
    //store current motor ticks
    private int flTickZero;
    private int frTickZero;
    private int blTickZero;
    private int brTickZero;

    //Constructor
    public Drive(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor, boolean zeroPower,
                 double flPowerMult, double frPowerMult, double blPowerMult, double brPowerMult,
                 double flTickMult, double frTickMult, double blTickMult, double brTickMult) {
        //get motors set up
        frontLeftMotor = map.get(DcMotor.class, flMotor);
        frontRightMotor = map.get(DcMotor.class, frMotor);
        backLeftMotor = map.get(DcMotor.class, blMotor);
        backRightMotor = map.get(DcMotor.class, brMotor);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //zeroPower set to true means coast, false means break
        if (zeroPower) {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        

        //multipliers
        this.flPowerMult = flPowerMult;
        this.frPowerMult = frPowerMult;
        this.blPowerMult = blPowerMult;
        this.brPowerMult = brPowerMult;
        this.flTickMult = flTickMult;
        this.frTickMult = frTickMult;
        this.blTickMult = blTickMult;
        this.brTickMult = brTickMult;
    }

    //set multipliers to 1
    public Drive(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor, boolean zeroPower) {
        this(map, flMotor, frMotor, blMotor, brMotor, zeroPower, 1, 1, 1, 1, 1, 1, 1, 1);
    }

    //default zeroPower to break
    public Drive(HardwareMap map, String flMotor, String frMotor, String blMotor, String brMotor) {
        this(map, flMotor, frMotor, blMotor, brMotor, false, 1, 1, 1, 1, 1, 1, 1, 1);
    }

    //stops all motors
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    //set the powers to zero
    public void powersToZero() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
    }

    //linear movement calculations given a unit vector and power
    public void linearPowerCalculations(TwoDimensionalVector direction, double power) {
        //get the  x and y values
        double x = direction.getX();
        double y = direction.getY();
        //max sum of x and y
        double max = Math.abs(x) + Math.abs(y);

        //add the linear motion components to their powers
        frontLeftPower += power * (y - x) / max;
        backRightPower += power * (y - x) / max;
        frontRightPower += power * (y + x) / max;
        backLeftPower += power * (y + x) / max;
    }

    //linear movement calculations given only a vector (magnitude shouldn't be greater than 1 because otherwise things don't work
    public void linearPowerCalculations(TwoDimensionalVector velocity) {
        linearPowerCalculations(velocity.unitVector(), velocity.norm());
    }

    //radial power calculation (- is left, + is right), power should be at most 1
    public void radialPowerCalculation(double power) {
        frontRightPower += power;
        backRightPower += power;
        frontLeftPower -= power;
        backLeftPower -= power;
    }

    //actually push out the powers
    public void pushPowers() {
        //multiply the powers by their multipliers
        frontLeftPower *= flPowerMult;
        frontRightPower *= frPowerMult;
        backLeftPower *= blPowerMult;
        backRightPower *= brPowerMult;
        //first make sure none of them are above 1
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    //sets the current tick values as the new zero values for ticks
    public void setZeroTicks() {
        flTickZero = frontLeftMotor.getCurrentPosition();
        frTickZero = frontRightMotor.getCurrentPosition();
        blTickZero = backLeftMotor.getCurrentPosition();
        brTickZero = backRightMotor.getCurrentPosition();
    }

    //actually reset the encoders - Warning: stops the robot briefly
    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setZeroTicks();
    }

    //get the current ticks for each motor
    public double getFrontLeftTicks() {
        return flTickMult * (frontLeftMotor.getCurrentPosition() - flTickZero);
    }

    public double getFrontRightTicks() {
        return frTickMult * (frontRightMotor.getCurrentPosition() - frTickZero);
    }

    public double getBackLeftTicks() {
        return blTickMult * (backLeftMotor.getCurrentPosition() - blTickZero);
    }

    public double getBackRightTicks() {
        return brTickMult * (backRightMotor.getCurrentPosition() - brTickZero);
    }

    //getters
    public double getFrontLeftPower() {
        return frontLeftPower;
    }

    public double getFrontRightPower() {
        return frontRightPower;
    }

    public double getBackLeftPower() {
        return backLeftPower;
    }

    public double getBackRightPower() {
        return backRightPower;
    }
}
