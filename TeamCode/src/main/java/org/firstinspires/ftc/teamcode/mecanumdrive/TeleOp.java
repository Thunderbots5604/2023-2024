package org.firstinspires.ftc.teamcode.mecanumdrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoByTwoMatrix;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

public class TeleOp {
    private boolean halfSpeed = false;
    private boolean reverse = false;
    private Drive driveTrain;
    private double powerMultiplier;
    private int orientation = 0;
    private double flZero = 0;
    private double frZero = 0;
    private double blZero = 0;
    private double brZero = 0;

    //constructor
    public TeleOp(HardwareMap map, double powerMultiplier) {
        driveTrain = new Drive(map, "lmf", "lmb", "rmf", "rmb", false, 1, 1, 1, 1, 1, 1, 1, 1);
        this.powerMultiplier = powerMultiplier;
    }

    // toggles halfSpeed and reverse on and off
    public void toggleHalfSpeed(){
        halfSpeed = !halfSpeed;
    }
    public void toggleReverse(){
        reverse = !reverse;
    }

    // moves and sets power to appropriate values, xvel is x-velocity, yvel is y-velocity, w is angular velocity
    public void move(double xvel, double yvel, double w) {
        //calculate multiplier
        double multiplier = powerMultiplier;
        if(reverse) {
            multiplier *= -1;
        }
        if(halfSpeed) {
            multiplier *= .5;
        }
        driveTrain.powersToZero();
        driveTrain.radialPowerCalculation(w);
        if(xvel != 0 || yvel != 0) {
            driveTrain.linearPowerCalculations(TwoByTwoMatrix.rotationMatrix(90 * orientation).multiplyVector(new TwoDimensionalVector(yvel * multiplier, xvel * multiplier)));
        }
        driveTrain.pushPowers();
    }

    public double[] getMotorPowers() {
        return new double[] {driveTrain.getFrontLeftPower(), driveTrain.getFrontRightPower(), driveTrain.getBackLeftPower(), driveTrain.getBackRightPower()};
    }

    public void setCurrentZeros() {
        flZero = driveTrain.getFrontLeftTicks();
        frZero = driveTrain.getFrontRightTicks();
        blZero = driveTrain.getBackLeftTicks();
        brZero = driveTrain.getBackRightTicks();
    }

    public double[] getChangeInPositions() {
        return new double[] {driveTrain.getFrontLeftTicks() - flZero, driveTrain.getFrontRightTicks() - frZero,
                driveTrain.getBackLeftTicks() - blZero, driveTrain.getBackRightTicks() - brZero};
    }

    //stop
    public void stop() {
        driveTrain.stop();
    }

    //set the orientation
    public void orient(int orientation) {
        this.orientation = orientation;
    }
}
