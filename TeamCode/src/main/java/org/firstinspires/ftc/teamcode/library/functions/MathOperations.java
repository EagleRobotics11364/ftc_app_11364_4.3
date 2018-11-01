package org.firstinspires.ftc.teamcode.library.functions;

/**
 * Created by Kk4jr on 9/28/2017.
 */

public class MathOperations {
    /**
     *
     * @param numberToClip This is the number to be range clipped
     * @param lowerBound Lower Bound
     * @param upperBound Upper Bound
     */
    public static double rangeClip(double numberToClip, double lowerBound, double upperBound){
        if(numberToClip >= upperBound)
            return upperBound;
        if(numberToClip <= lowerBound)
            return lowerBound;
        return numberToClip;
    }
    public static float rangeClip(float numberToClip, float lowerBound, float upperBound){
        if(numberToClip >= upperBound)
            return upperBound;
        if(numberToClip <= lowerBound)
            return lowerBound;
        return numberToClip;
    }

    public static double pow(double x, int n){
        double temp = x;
        for(int i = 1; i < n; i++){
            temp *= x;
        }
        return temp;
    }
}