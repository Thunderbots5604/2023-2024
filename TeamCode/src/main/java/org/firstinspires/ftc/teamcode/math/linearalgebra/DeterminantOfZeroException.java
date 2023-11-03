package org.firstinspires.ftc.teamcode.math.linearalgebra;

public class DeterminantOfZeroException extends Exception {
    private TwoByTwoMatrix matrix;

    public DeterminantOfZeroException(TwoByTwoMatrix matrix, String message) {
        super(message);
        this.setMatrix(matrix);
    }

    public DeterminantOfZeroException(TwoByTwoMatrix matrix, String message, Throwable cause) {
        super(message, cause);
        this.setMatrix(matrix);
    }

    public TwoByTwoMatrix getMatrix() {
        return matrix;
    }

    public void setMatrix(TwoByTwoMatrix matrix) {
        this.matrix = matrix;
    }
}