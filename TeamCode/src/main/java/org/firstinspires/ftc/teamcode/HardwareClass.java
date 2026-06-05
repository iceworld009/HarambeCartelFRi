package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class HardwareClass {


    //Declarations
    public DcMotor FR, FL , BR , BL;

    public DcMotorEx ramp, ramp2, turret,intakeMotor;

    private static HardwareClass hardwareClass = null;


    public Servo angle;
    public Servo selectTop, selectBotR, selectBotL;
    public CRServo rise1, rise2;
    public RevColorSensorV3 colorTop, colorBotR, colorBotL; //selection sensors
    public RevColorSensorV3 colorTop2, colorBotR2, colorBotL2;
    public static double startX = 64 ,startY = 0, startAngle=0;//Inch
    public static double redX = -71, redY = 70;
    public static double redScoreX = -71, redScoreY = 70;
    public static double autoRedScorePoseX = 4, autoRedScorePoseY = 139;
    public static double autoBlueScorePoseX= 4, autoBlueScorePoseY = 5;

    public static double blueX = -71, blueY = -70;
    public static double blueScoreX = -71, blueScoreY = -70;
    public static double tagPosX = -72, tagPosY = 0;

    public static double selectTopLOW = 0.89, selectBotLLOW = 0.13,selectBotRLOW =0.87;
    public static double selectTopHIGH = 0.48, selectBotLHIGH = 0.6,selectBotRHIGH = 0.45;
    public static int bratDelay = 135; // 110
    public static int bratBetween = 65; // 65
    public static int turret_min=-715, turret_max=0;
    public static double hood_LOW = 0.21, hood_HIGH = 0.46;



    public HardwareClass(HardwareMap hardwareMap){

        /*Chassis*/

        this.FR = hardwareMap.get(DcMotor.class , "FR");
        this.FL = hardwareMap.get(DcMotor.class , "FL");
        this.BR = hardwareMap.get(DcMotor.class , "BR");
        this.BL = hardwareMap.get(DcMotor.class , "BL");

        /*Subsystems*/

        this.ramp = hardwareMap.get(DcMotorEx.class, "Ramp");
        this.ramp2 = hardwareMap.get(DcMotorEx.class,"Ramp2");
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "IM");
        this.turret = hardwareMap.get(DcMotorEx.class , "TR");


        /*Servos*/
        this.angle = hardwareMap.get(Servo.class, "AG");
        this.selectTop = hardwareMap.get(Servo.class, "ST");
        this.selectBotR = hardwareMap.get(Servo.class, "SR");
        this.selectBotL = hardwareMap.get(Servo.class, "SL");
        this.rise1 = hardwareMap.get(CRServo.class,"R1");
        this.rise2 = hardwareMap.get(CRServo.class,"R2");
        /* Sensors */
        this.colorTop = hardwareMap.get(RevColorSensorV3.class, "CT");
        this.colorBotL = hardwareMap.get(RevColorSensorV3.class, "CL");
        this.colorBotR = hardwareMap.get(RevColorSensorV3.class, "CR");
        this.colorTop2 = hardwareMap.get(RevColorSensorV3.class, "CT2");
        this.colorBotL2 = hardwareMap.get(RevColorSensorV3.class, "CL2");
        this.colorBotR2 = hardwareMap.get(RevColorSensorV3.class, "CR2");
    }

    public static synchronized HardwareClass getInstance(HardwareMap hardwareMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hardwareMap);
        return hardwareClass;
    }
}