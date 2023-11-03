package org.firstinspires.ftc.teamcode.math.linearalgebra;

public class AffineFunction2to2 {
    //variables
    private TwoByTwoMatrix linearTransformation;
    private TwoDimensionalVector translationVector;

    //constructor
    public AffineFunction2to2(TwoByTwoMatrix linearTransformation, TwoDimensionalVector translationVector) {
        this.linearTransformation = linearTransformation;
        this.translationVector = translationVector;
    }

    //apply the function
    public TwoDimensionalVector applyFunction(TwoDimensionalVector input) {
        return this.linearTransformation.multiplyVector(input).plus(translationVector);
    }

    //
}
