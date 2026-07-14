package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.SubSys.Turret;

@TeleOp(name="Reset Turret", group = "Solo")
@Config
public class TurretCheck extends OpMode {

    Turret turret = null;
    public static double adjust = 0;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        turret = new Turret(new HardwareClass(hardwareMap),telemetry);
        turret.setup();
        turret.resetTurret();
        turret.powerOn();

    }
    public void start(){
        timer.reset();
        turret.goToPosition(adjust);
    }
    @Override
    public void loop() {
        if(Math.abs(turret.getPosition()-adjust)<1.5) {
            telemetry.addData("Time:", timer.milliseconds());
            telemetry.update();
            while(true){}
        }
        telemetry.addData("Position:",turret.getPosition());
        telemetry.addData("Error:",Math.abs(turret.getPosition()-adjust)<1.5);
        telemetry.update();
    }
}