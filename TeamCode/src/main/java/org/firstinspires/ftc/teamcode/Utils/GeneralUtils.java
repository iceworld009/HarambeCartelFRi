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

    public static int getRPM(double d) {
        double r;
        if (d <= 220)
            r = 2860;
        else if (d <= 250)
            r = 2875 + (d - 220) * (2869 - 2820) / (250 - 220);
        else if (d <= 330)
            r = 3000 + (d - 250) * (3733 - 2869) / (330 - 250);
        else if (d <= 356)
            r = 3925 + (d - 330) * (3800 - 3733) / (356 - 330);
        else
            r = 4050 + (d - 356) * (3800 - 3733) / (356 - 330);

        return (int) Math.max(1000, Math.min(r, 4000));
    }

    public static int  getRPMPurple(double d) {
        double r = -0.0023 * Math.pow(d, 2) + 2.77 * d + 2200;

        return (int) Math.max(1000, Math.min(r, 4000));
    }

    public static double  getHood(double d) {
        if (d < 130) {
            return (d - 100) * (7.0 / 30.0);
        }
        else if (d < 170) {
            return 5 - (d - 130) * (2.5 / 40.0);
        }
        else if (d < 220) {
            return 3 + (d - 170) * (1.3 / 50.0);
        }
        else {
            return 4.6 + (d - 220) * (2.6 / 130.0);
        }
    }

        public static double convertToNewRange(double value,
                                               double oldMin, double oldMax,
                                               double newMin, double newMax) {
            return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
        }

}
