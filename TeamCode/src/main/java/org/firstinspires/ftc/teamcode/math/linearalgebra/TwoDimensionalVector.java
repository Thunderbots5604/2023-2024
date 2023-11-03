package org.firstinspires.ftc.teamcode.math.linearalgebra;

public class TwoDimensionalVector implements  Cloneable{
    private final double x;
    private final double y;

    //Constructor just takes the coordinates
    public TwoDimensionalVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    //zero vector
    public static TwoDimensionalVector zeroVector() {
        return new TwoDimensionalVector(0,0);
    }

    //unit vector version
    public TwoDimensionalVector unitVector() {
        double length = norm();
        return new TwoDimensionalVector(getX() / length, getY() / length);
    }

    //Dot product of two vectors
    public double dot(TwoDimensionalVector otherVector) {
        return x * otherVector.getX() + y * otherVector.getY();
    }

    //scale the vector
    public TwoDimensionalVector scale(double scaleFactor) {
        return new TwoDimensionalVector(this.x * scaleFactor, this.y * scaleFactor);
    }

    //get the length (norm) of the vector
    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    //get the vector as an array
    public double[] asArray() {
        return new double[]{x, y};
    }

    //flip the direction
    public TwoDimensionalVector negative() {
        return new TwoDimensionalVector(-x, -y);
    }

    //vector addition
    public TwoDimensionalVector plus(TwoDimensionalVector otherVector) {
        return new TwoDimensionalVector(x + otherVector.getX(), y + otherVector.getY());
    }

    //clone
    public TwoDimensionalVector clone() {
        return new TwoDimensionalVector(this.x, this.y);
    }
    
    //equality
    public boolean equals(TwoDimensionalVector otherVector) {
        return this.getX() == otherVector.getX() && this.getY() == otherVector.getY();
    }

    //Getters and Setters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
