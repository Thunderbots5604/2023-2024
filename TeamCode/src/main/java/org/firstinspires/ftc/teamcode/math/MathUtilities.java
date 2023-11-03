/*A set of various utilities used for things in our math package
 * everything here should be static*/

package org.firstinspires.ftc.teamcode.math;

public class MathUtilities {
    //variables
    //very small number used for comparisons
    public static final double SMALL_NUMBER = 1e-6;

    //methods
    //figure out if the two values are close to equal
    //use if slightly imprecise values might be created
    public static boolean closeEnough(double firstNumber, double secondNumber) {
        //check for being close
        //otherwise return false
        return Math.abs(firstNumber - secondNumber) <= SMALL_NUMBER;
    }

    //check if a number is within a certain range
    public static boolean within(double range, double firstNumber, double secondNumber) {
        return Math.abs(firstNumber - secondNumber) <= range;
    }

    //angle addition
    public static double angleSum(double angle1, double angle2) {
        double sum = (((angle1 + angle2) % 360));
        if (sum >= 180) {
            return (sum - 360);
        }
        else if (sum < -180) {
            return (sum + 360);
        }
        else {
            return sum;
        }
    }

    //get the difference between two angles, making sure that the value is between -180 and 180
    public static double angleDifference(double angle1, double angle2) {
        //get the difference
        double difference = (((angle1 - angle2) % 360));
        if (difference >= 180) {
            return (difference - 360);
        }
        else if (difference < -180) {
            return (difference + 360);
        }
        else {
            return difference;
        }
    }
}