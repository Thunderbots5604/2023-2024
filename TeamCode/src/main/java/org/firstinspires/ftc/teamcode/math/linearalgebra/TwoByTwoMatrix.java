package org.firstinspires.ftc.teamcode.math.linearalgebra;

public class TwoByTwoMatrix implements Cloneable{
    //We're looking at this as two vectors dont @ me
    private TwoDimensionalVector topRow;
    private TwoDimensionalVector bottomRow;

    //Constructor uses vectors as inputs
    public TwoByTwoMatrix(TwoDimensionalVector topRow, TwoDimensionalVector bottomRow) {
        this.topRow = topRow.clone();
        this.bottomRow = bottomRow.clone();
    }

    //generate a rotation matrix. Note it rotates the coordinate plane by this angle, it doesn't rotate this angle to the coordinate plane
    public static TwoByTwoMatrix rotationMatrix(double theta) {
        TwoDimensionalVector xRotator = new TwoDimensionalVector(Math.sin(Math.toRadians(theta)), Math.cos(Math.toRadians(theta)));
        TwoDimensionalVector yRotator = new TwoDimensionalVector(Math.cos(Math.toRadians(theta)), -Math.sin(Math.toRadians(theta)));
        return new TwoByTwoMatrix(yRotator, xRotator);
    }

    //get the determinant
    public double determinant() {
        return topRow.getX() * bottomRow.getY() - topRow.getY() * bottomRow.getX();
    }

    //get the inverse matrix
    public TwoByTwoMatrix inverse() throws DeterminantOfZeroException {
        double determinant = this.determinant();
        //throw exception if determinant is 0
        if(determinant == 0) {
            throw new DeterminantOfZeroException(this, "This matrix has a determinant of zero, so the inverse does not exist.");
        }

        //1/determinant
        double scaleFactor = 1 / determinant;
        //flip the values as necessary
        TwoDimensionalVector topInverse = new TwoDimensionalVector(this.bottomRow.getY(), -this.topRow.getY());
        TwoDimensionalVector bottomInverse = new TwoDimensionalVector(-this.bottomRow.getX(), this.topRow.getX());
        //generate the inverse
        TwoByTwoMatrix inverted = new TwoByTwoMatrix(topInverse, bottomInverse);
        return inverted.scale(scaleFactor);
    }

    //scale the matrix by a scalar
    public TwoByTwoMatrix scale(double scaleFactor) {
        return new TwoByTwoMatrix(this.topRow.scale(scaleFactor), this.bottomRow.scale(scaleFactor));
    }

    //Multiply it by a vector
    public TwoDimensionalVector multiplyVector(TwoDimensionalVector vector) {
        return new TwoDimensionalVector(topRow.dot(vector), bottomRow.dot(vector));
    }

    public TwoByTwoMatrix clone() {
        return new TwoByTwoMatrix(this.topRow, this.bottomRow);
    }

    public TwoDimensionalVector getTopRow() {
        return topRow.clone();
    }

    public void setTopRow(TwoDimensionalVector topRow) {
        this.topRow = topRow;
    }

    public TwoDimensionalVector getBottomRow() {
        return bottomRow.clone();
    }

    public void setBottomRow(TwoDimensionalVector bottomRow) {
        this.bottomRow = bottomRow;
    }
}
