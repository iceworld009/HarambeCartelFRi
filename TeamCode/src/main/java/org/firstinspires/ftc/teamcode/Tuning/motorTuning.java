package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class motorTuning extends LinearOpMode {

    public DcMotor motor = null;
    public static String motorName = "";
    public static double Power = 0;
        public void runOpMode(){
            this.motor = hardwareMap.get(DcMotor.class, motorName);
            waitForStart();
            while(opModeIsActive()){
                motor.setPower(Power);
            }
        }
}