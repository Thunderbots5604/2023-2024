package org.firstinspires.ftc.teamcode.mecanumdrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;
import org.firstinspires.ftc.teamcode.pathing.RobotPosition;
import org.firstinspires.ftc.teamcode.pathing.RobotPositionSystem;


public class Autonomous {
    //variables
    private Drive driveTrain;
    private final double X_TICK_SCALING = 1;
    private final double Y_TICK_SCALING = 1;
    private final double ROTATION_TICK_SCALING = 0.03211;
    private RobotPositionSystem positionSystem;
    private final double minRunPower = 0.3;
    private final double maxRunPower = 0.6;
    private final double linearTolerance;
    private final double maxLinearDistance;
    private final double rotationalTolerance;
    private final double maxRotationalDistance;
    private RobotPosition powerRecommendations;
    

    //constructor
    public Autonomous(HardwareMap map, RobotPosition startingRobotPosition, double linearTolerance,
                      double maxLinearDistance, double rotationalTolerance, double maxRotationalDistance) {


        driveTrain = new Drive(map, "lmf", "rmf", "lmb", "rmb");
        driveTrain.resetEncoders();

        positionSystem = new RobotPositionSystem(startingRobotPosition);

        this.linearTolerance = linearTolerance;
        this.rotationalTolerance = rotationalTolerance;
        this.maxLinearDistance = maxLinearDistance;
        this.maxRotationalDistance = maxRotationalDistance;
    }

    
    public boolean moveTo(RobotPosition target) {
        updatePosition();
        if(positionSystem.getRobotPosition().roughlyEquals(target, linearTolerance, rotationalTolerance)) {
            return true;
        }
        powerRecommendations = recommendPowers(target);
        //movement loop
        driveTrain.powersToZero();
        if (!powerRecommendations.getLocation().equals(TwoDimensionalVector.zeroVector())) {
            driveTrain.linearPowerCalculations(powerRecommendations.getLocation());
        }
        driveTrain.radialPowerCalculation(powerRecommendations.getAngle());
        driveTrain.pushPowers();
        return false;
    }

    //move to a target position
    public void targetingTest(RobotPosition target, Telemetry tele) {
        RobotPosition powerRecommendations;
        RobotPosition currentPosition = positionSystem.getRobotPosition();
        updatePosition();

        while(!currentPosition.roughlyEquals(target, linearTolerance, rotationalTolerance)) {
            powerRecommendations = recommendPowers(target);
            //movement loop
            driveTrain.powersToZero();
            if (!powerRecommendations.getLocation().equals(TwoDimensionalVector.zeroVector())) {
                driveTrain.linearPowerCalculations(powerRecommendations.getLocation());
            }
            driveTrain.radialPowerCalculation(powerRecommendations.getAngle());
            driveTrain.pushPowers();
            updatePosition();
            currentPosition = positionSystem.getRobotPosition();

            tele.addData("Recommended X", powerRecommendations.getLocation().getX());
            tele.addData("Recommended Y", powerRecommendations.getLocation().getY());
            tele.addData("Recommended R", powerRecommendations.getAngle());

            tele.addData("flPower", driveTrain.getFrontLeftPower());
            tele.addData("frPower", driveTrain.getFrontRightPower());
            tele.addData("blPower", driveTrain.getBackLeftPower());
            tele.addData("brPower", driveTrain.getBackRightPower());

            tele.addData("target X", target.getLocation().getX());
            tele.addData("target Y", target.getLocation().getY());
            tele.addData("target R", target.getAngle());
            tele.addData("current X", currentPosition.getLocation().getX());
            tele.addData("current Y", currentPosition.getLocation().getY());
            tele.addData("current R", currentPosition.getAngle());
            tele.update();
            
        }
        driveTrain.stop();
    }
    
    public void stop() {
        driveTrain.stop();
    }

    //recommend powers to use in linear and rotational motion
    public RobotPosition recommendPowers(RobotPosition target) {
        RobotPosition difference = positionSystem.relativeToRobot(target);
        double xDistance = difference.getLocation().getX();
        double yDistance = difference.getLocation().getY();
        double rDistance = difference.getAngle();
        double xPower;
        double yPower;
        double rPower;

        if(Math.abs(xDistance) > linearTolerance) {
            if (Math.abs(xDistance) <= maxLinearDistance) {
                xPower = minRunPower + (maxRunPower - minRunPower) * (Math.abs(xDistance) - linearTolerance) / (maxLinearDistance - linearTolerance);
            }
            else {
                xPower = maxRunPower;
            }
        }
        else {
            xPower = 0;
        }

        if(Math.abs(yDistance) > linearTolerance) {
            if (Math.abs(yDistance) <= maxLinearDistance) {
                yPower = minRunPower + (maxRunPower - minRunPower) * (Math.abs(yDistance) - linearTolerance) / (maxLinearDistance - linearTolerance);
            }
            else {
                yPower = maxRunPower;
            }
        }
        else {
            yPower = 0;
        }

        if(Math.abs(rDistance) > rotationalTolerance) {
            if (Math.abs(rDistance) <= maxRotationalDistance) {
                rPower = minRunPower + (maxRunPower - minRunPower) * (Math.abs(rDistance) - rotationalTolerance) / (maxRotationalDistance - rotationalTolerance);
            }
            else {
                rPower = maxRunPower;
            }
        }
        else {
            rPower = 0;
        }
        
        //grr needed to set the signs of the powers
        if(xDistance < 0) {
            xPower *= -1;
        }
        
        if(yDistance < 0) {
            yPower *= -1;
        }
        
        if(rDistance < 0) {
            rPower *= -1;
        }

        return new RobotPosition(new TwoDimensionalVector(xPower, yPower), rPower);
    }

    //updates the robot's current position from either encoder values
    public void updatePosition() {
        double flTickChange = driveTrain.getFrontLeftTicks();
        double frTickChange = driveTrain.getFrontRightTicks();
        double blTickChange = driveTrain.getBackLeftTicks();
        double brTickChange = driveTrain.getBackRightTicks();
        
        double deltaX = (-flTickChange + frTickChange + blTickChange - brTickChange) * X_TICK_SCALING;
        double deltaY = (frTickChange + flTickChange + brTickChange + blTickChange) * Y_TICK_SCALING;
        double deltaR = (frTickChange - flTickChange + brTickChange - blTickChange) * ROTATION_TICK_SCALING;

        positionSystem.addRelativePosition(new RobotPosition(new TwoDimensionalVector(deltaX, deltaY), deltaR));

        driveTrain.setZeroTicks();
    }
}
