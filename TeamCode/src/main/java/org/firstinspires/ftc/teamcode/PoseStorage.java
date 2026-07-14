package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class PoseStorage {
    public static Pose autoPose = null;
    public static Pose redPose = new Pose(127,79.5,Math.toRadians(0));
    public static Pose bluePose = new Pose(16.25,80.75,Math.toRadians(180));
    public static Pose resetPose = new Pose(72,-72,0);//public static double startX = 62.5 ,startY = 0, startAngle=180;
}
