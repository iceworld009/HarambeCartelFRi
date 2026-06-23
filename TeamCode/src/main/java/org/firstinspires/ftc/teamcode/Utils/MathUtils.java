package org.firstinspires.ftc.teamcode.Utils;

public enum MathUtils {
    ;
    public static double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }
}
