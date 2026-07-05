package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.HardwareClass;

public enum Geometry {
    ;
    public double getDistanceOD(double x, double y, int color){ //Formula automata pentru cele doua cosuri
        double distance = 0;
        if(color == 0){//red
            distance = Math.sqrt(Math.pow(HardwareClass.redX-x,2)+Math.pow(HardwareClass.redY-y,2));
        }
        else
            distance = Math.sqrt(Math.pow(HardwareClass.blueX-x,2)+Math.pow(HardwareClass.blueY-y,2));

        return distance * 2.54;
    }

    public double getDistanceODMan(double x, double y, double targetX, double targetY){
        return Math.sqrt(Math.pow(targetX-x,2)+Math.pow(targetY-y,2)) * 2.54;
    }

    public double getRobotAngle(double x, double y, int target){
        if(target == 0)
            return Math.atan2(Math.abs(HardwareClass.redY-y),Math.abs(HardwareClass.redX-x));
        return Math.atan2(Math.abs(HardwareClass.blueY-y),Math.abs(HardwareClass.blueY-x));
    }
}
