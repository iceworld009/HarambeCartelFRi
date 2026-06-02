package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class PoseStorage {
    public static Pose autoPose = new Pose(HardwareClass.startX, HardwareClass.startY, HardwareClass.startAngle);
    public static Pose redPose = new Pose(-7.6,54,1.55);
    public static Pose bluePose = new Pose(-9.7,-56.2,4.7123);
    public static Pose resetPose = new Pose(64,0,0);//public static double startX = 62.5 ,startY = 0, startAngle=180;
}
