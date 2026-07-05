package org.firstinspires.ftc.teamcode.Utils;

public enum GeneralUtils {

    ;
    public static void sleep(int ms){
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public double getRPM(double d) {
        double r = 0.012 * d * d - 1.9 * d + 2070;
        r = Math.max(Math.min(r,3000),100);
        return r;
    }
    public double getHood(double d) {
        if (d < 130)
            return (d - 100) * (9.0 / 30.0);
        else if (d < 170)
            return 9 - (d - 130) * (1.0 / 40.0);
        else
            return 6 + (d - 170) * (5.0 / 150.0);
    }

        public static double convertToNewRange(double value,
                                               double oldMin, double oldMax,
                                               double newMin, double newMax) {
            return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
        }

}
