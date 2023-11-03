/* This class can be used to draw a line between two points
 * Note that it functions a bit like a vector in that it is directional
 * since it has a startpoint and an endpoint. Also, technically a line
 * segment because it's not infinite.*/

package org.firstinspires.ftc.teamcode.math;

public class Line implements Cloneable{
    //variables
    private Point startPoint;
    private Point endPoint;
    private double changeInX;
    private double changeInY;

    //methods
    //constructor - inputs two points and makes line between them
    public Line(Point startPoint, Point endPoint) {
        this.startPoint = startPoint.clone();
        this.endPoint = endPoint.clone();
        updateChanges();
    }

    //constructor - from 4 numbers
    public Line(double x1, double y1, double x2, double y2) {
        this(new Point(x1, y1), new Point(x2, y2));
    }

    //updateChanges is used to update the change in x and change in y
    //when another coordinate changes
    private void updateChanges() {
        changeInX = endPoint.getX() - startPoint.getX();
        changeInY = endPoint.getY() - startPoint.getY();
    }

    //tools for changing endpoint and startpoint
    //The lack of scaling is intentional as it would be ambiguous.
    //If you would want to scale, you'd have to decide where to scale from,
    //so this is left to whoever would call that method.
    //whole new startpoint
    public void setStartPoint(Point newStartPoint) {
        startPoint = newStartPoint.clone();
        updateChanges();
    }

    //whole new endpoint
    public void setEndPoint(Point newEndPoint) {
        endPoint = newEndPoint.clone();
        updateChanges();
    }

    //translate startpoint by some amount
    public void translateStartPoint(Point translateBy) {
        this.startPoint = this.getStartPoint().plus(translateBy);
        updateChanges();
    }

    //translate endpoint by some amount
    public void translateEndPoint(Point translateBy) {
        this.endPoint = this.getEndPoint().plus(translateBy);
        updateChanges();
    }

    //translate the whole line by some amount
    public void translateWholeLine(Point translateBy) {
        this.translateEndPoint(translateBy);
        this.translateStartPoint(translateBy);
    }

    //translate a version of the line with the start at the origin
    public Line startAtOrigin() {
        //make a copy of this line
        Line originLine =  this.clone();
        //calculate how much you need to translate by to get to the origin
        Point translationFactor = Point.origin().minus(originLine.getStartPoint());
        //translate the originLine by that amount
        originLine.translateWholeLine(translationFactor);
        //return the new line
        return originLine;
    }

    //flip the line, so end point becomes start point and start point becomes end point
    public Line flip() {
        return new Line(this.getEndPoint(), this.getStartPoint());
    }

    //clone - just manually make a copy
    @Override
    public Line clone() {
        //note that the start point and end are cloned, so this does not
        //reference the original points at all
        return new Line(this.getStartPoint(), this.getEndPoint());
    }

    //check for equality
    //BY DEFAULT, EQUALS DOES NOT CHECK THE FLIPPED VERSIONS!
    public boolean equals(Line otherLine) {
        return (this.getStartPoint().equals(otherLine.getStartPoint()) &&
                this.getEndPoint().equals(otherLine.getEndPoint()));
    }

    //check for equality WITH FLIPPED VERSIONS BEING CHECKED
    public boolean nonDirectionalEquals(Line otherLine) {
        //if the lines are the same forwards OR backwards, return true
        return (this.equals(otherLine) || this.flip().equals(otherLine));
    }

    //find the length of the line
    public double getLength() {
        return Math.sqrt(getChangeInX() * getChangeInX() + getChangeInY() * getChangeInY());
    }

    //find the angle of the line from the positive x axis
    public double getAngle() {
        return Math.atan2(changeInY, changeInX);
    }

    public Point getStartPoint() {
        //return a clone of it because otherwise bad things might happen
        return startPoint.clone();
    }

    public Point getEndPoint() {
        //return a clone of it because otherwise bad things might happen
        return endPoint.clone();
    }

    public double getChangeInX() {
        return changeInX;
    }

    public double getChangeInY() {
        return changeInY;
    }
}