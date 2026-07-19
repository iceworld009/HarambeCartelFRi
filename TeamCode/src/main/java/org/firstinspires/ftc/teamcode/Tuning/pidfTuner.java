package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config
public class pidfTuner extends LinearOpMode {

    public DcMotorEx motor = null, motor2 = null;
    public static String motorName = "";
    public static String motorName2 = "";
    public static double p,f;
    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
    TelemetryManager TelemetryM;
    public static double Speed = 0;
    public void runOpMode(){
        TelemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.motor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){

            motor.setVelocity(Speed);
            motor2.setVelocity(Speed);
            TelemetryM.addData("Speed:",(motor.getVelocity()*60)/28);
            TelemetryM.update();
        }
    }
}