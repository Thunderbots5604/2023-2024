package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.math.MathUtilities;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

public class RobotPosition implements Cloneable{
    //robot angle and position
    private final double angle;
    private final TwoDimensionalVector location;

    //constructor
    public RobotPosition(TwoDimensionalVector location, double angle) {
        this.location = location.clone();
        this.angle = angle;
    }

    //add a change in position to it
    public RobotPosition plus(RobotPosition otherRobotPosition) {
        return new RobotPosition(location.plus(otherRobotPosition.getLocation()), MathUtilities.angleSum(angle, otherRobotPosition.getAngle()));
    }

    //get the difference, that is the values needed to go between the two positions
    public RobotPosition minus(RobotPosition otherRobotPosition) {
        return new RobotPosition(location.plus(otherRobotPosition.getLocation().negative()), MathUtilities.angleDifference(angle, otherRobotPosition.getAngle()));
    }

    //equality and rough equality
    public boolean equals(RobotPosition otherRobotPosition) {
        return location.equals(otherRobotPosition.getLocation()) && angle == otherRobotPosition.getAngle();
    }

    public boolean roughlyEquals(RobotPosition otherRobotPosition, double locationTolerance, double angleTolerance) {
        return MathUtilities.within(locationTolerance, location.getX(), otherRobotPosition.getLocation().getX()) &&
                MathUtilities.within(locationTolerance, location.getY(), otherRobotPosition.getLocation().getY()) &&
                MathUtilities.within(angleTolerance, angle, otherRobotPosition.getAngle());
    }

    public RobotPosition clone() {
        return new RobotPosition(this.location, this.angle);
    }

    //getters and setters
    public double getAngle() {
        return angle;
    }

    public TwoDimensionalVector getLocation() {
        return location.clone();
    }
}