package org.firstinspires.ftc.teamcode.pathing;

import org.firstinspires.ftc.teamcode.math.MathUtilities;
import org.firstinspires.ftc.teamcode.math.linearalgebra.DeterminantOfZeroException;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoByTwoMatrix;
import org.firstinspires.ftc.teamcode.math.linearalgebra.TwoDimensionalVector;

public class RobotPositionSystem {
    //current absolute position
    private RobotPosition robotPosition;

    //defaults to 0, 0 at angle 0
    public RobotPositionSystem() {
        this.robotPosition = new RobotPosition(TwoDimensionalVector.zeroVector(), 0);
    }

    //if a specific starting position is set
    public RobotPositionSystem(RobotPosition currentRobotPosition) {
        this.robotPosition = currentRobotPosition.clone();
    }

    //add a position relative to the robot rather than absolutely
    //NOTE: This is somewhat problematic, since it can't get the change in position accurately due to the change in angles
    public void addRelativePosition(RobotPosition relativeChange) {
        //get the estimation angle
        double estimationAngle = relativeChange.getAngle() / 2;

        //now, get the change in the location
        TwoDimensionalVector absoluteChangeVector = TwoByTwoMatrix.rotationMatrix(estimationAngle).multiplyVector(relativeChange.getLocation());
        addAbsolutePosition(new RobotPosition(absoluteChangeVector, estimationAngle * 2));
    }

    //adjust the current absolute position by a bit
    public void addAbsolutePosition(RobotPosition change) {
        robotPosition = robotPosition.plus(change);
    }

    //rotate a direction vector to where it points relative to where the robot points
    public TwoDimensionalVector rotateToRobot(TwoDimensionalVector absoluteDirection) {
        return TwoByTwoMatrix.rotationMatrix(robotPosition.getAngle()).multiplyVector(absoluteDirection);
    }

    //rotate a direction vector to where it points absolutely from its relative position to the robot
    public TwoDimensionalVector rotateFromRobot(TwoDimensionalVector relativeDirection) throws DeterminantOfZeroException {
        return TwoByTwoMatrix.rotationMatrix(robotPosition.getAngle()).inverse().multiplyVector(relativeDirection.clone());
    }

    //describe a robot position relative to the current robot position
    public RobotPosition relativeToRobot(RobotPosition absoluteRobotPosition) {
        double relativeAngle = MathUtilities.angleDifference(absoluteRobotPosition.getAngle(), robotPosition.getAngle());
        TwoDimensionalVector relativeLocation = rotateToRobot(absoluteRobotPosition.getLocation().plus(robotPosition.getLocation().negative()));
        return new RobotPosition(relativeLocation, relativeAngle);
    }

    //describe a position vector relative to the robot based on absolute position
    public RobotPosition absolutePosition(RobotPosition relativeToRobot) throws DeterminantOfZeroException {
        double absoluteAngle = MathUtilities.angleSum(relativeToRobot.getAngle(), robotPosition.getAngle());
        TwoDimensionalVector absoluteLocation = rotateFromRobot(relativeToRobot.getLocation()).plus(robotPosition.getLocation());
        return new RobotPosition(absoluteLocation, absoluteAngle);
    }

    public TwoDimensionalVector getRobotLocation() {
        return robotPosition.getLocation();
    }

    public double getRobotAngle() {
        return robotPosition.getAngle();
    }

//    public void setRobotLocation(TwoDimensionalVector location) {
//        this.robotPosition.setLocation(location);
//    }
//
//    public void setRobotAngle(double angle) {
//        this.robotPosition.setAngle(angle);
//    }

    public RobotPosition getRobotPosition() {
        return robotPosition.clone();
    }

    public void setRobotPosition(RobotPosition robotPosition) {
        this.robotPosition = robotPosition.clone();
    }
}
