package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public enum Color {
    PURPLE,
    GREEN,
    INVALID;

    public static Color getColor(double red,double green,double blue,double alpha) {

        if(alpha<10 && (green*blue*red)==0)
            return INVALID;
        if (alpha > 350) {
            if (green+100 < (blue + red) / 2)
                return PURPLE;
            else if (green > red && green > blue && green > 1000)
                return GREEN;
        }
        return INVALID;
    }

    public static Color getColor2C(double red,double green, double blue,double alpha,double red2,double green2, double blue2, double alpha2){
        red+=red2;
        green+=green2;
        blue+=blue2;
        alpha+=alpha2;
        if(alpha<20 && (green*blue*red)==0)
            return INVALID;
        if (alpha > 420) {
            if (green+100 < (blue + red) / 2)
                return PURPLE;
            else if (green > red && green > blue && green > 1000)
                return GREEN;
        }
        return INVALID;
    }


    public static boolean isBetween(ArrayList<Double> colorMin, ArrayList<Double> colorMax, ArrayList<Double> color) {
        double current;
        for(int i = 0; i < 3; i++) {
            current = color.get(i);

            if(current > colorMax.get(i) || current < colorMin.get(i)) {
                return false;
            }
        }

        return true;
    }
}