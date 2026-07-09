package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.SubSys.Turret;

@TeleOp(name="Reset Turret", group = "Solo")
@Config
public class TurretCheck extends OpMode {

    public DcMotorEx motor = null;
    public DcMotorEx FR = null;
    public static String motorName = "Ramp2";
    public static String FR_Name = "FR";

    public static double kp = 0.01, ki = 0, kd = 0.001;
    public static double target = 0;

    Turret turret = null;
    public static double Position = 0;
    public static double adjust = 0;

    @Override
    public void init() {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret = new Turret(new HardwareClass(hardwareMap),telemetry);
        //turret.setup();
        turret.resetMotor();
    }

    @Override
    public void loop() {
        //motor.setPower(adjust);
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.update();
    }
}