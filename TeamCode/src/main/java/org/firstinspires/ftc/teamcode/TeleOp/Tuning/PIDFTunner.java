package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Motors;
//requires motor class!!!!!
//I will change the tuner in future update!!

@Config
@TeleOp(name="PIDF Tunner", group = "Solo")
public class PIDFTunner extends LinearOpMode {
    Servos servos = null;
    HardwareClass hardwareClass = null;
    Selectioner selectioner = null;
    Motors motors = null;

    private TelemetryManager telemetryM;
    @Override
    public void runOpMode()  {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        servos = Servos.getInstance(hardwareMap , telemetry);
        motors = Motors.getInstance(hardwareMap);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.ramp.setDirection(DcMotorSimple.Direction.FORWARD);
        selectioner = Selectioner.getInstance(hardwareClass,telemetry);
        waitForStart();
        motors.setRampVelocityC(1000);
         ElapsedTime delay = new ElapsedTime();
        double coefs[] = { 32, 0 , 0 , 0.5 };
        int target = 0;
        double size = 0.1;
        double targetVelocity = 2700;
        int currentAngle = 10;
        boolean shooting = false;
        hardwareClass.ramp.setDirection(DcMotorSimple.Direction.FORWARD);
        while(opModeIsActive()) {

            double velocity = motors.getVelocity();
            double error = targetVelocity - velocity;
            selectioner.hoodMove(currentAngle);

            if (gamepad1.ps) {
                telemetry.clear();
                motors.setCoefsMan(coefs[0],coefs[1],coefs[2],coefs[3]);
                telemetry.addLine("Coefs set!");
                telemetry.update();
                sleep(500);
            }

            if(gamepad1.right_bumper) {
                motors.setCoefsMan(coefs[0], coefs[1], coefs[2], coefs[3]);
                motors.setRampVelocityC((int)targetVelocity);
            }

            if(Math.abs(motors.getRampError(targetVelocity))<250) {
                selectioner.unloadBalls();
                sleep(200);
                motors.setCoefsMan(20,0,0,1);
                motors.setRampVelocityC(1000);
            }

            if (gamepad1.left_bumper) {
                motors.rampStop();
            }

            if (gamepad1.dpad_right) {
                if (target < coefs.length - 1 && delay.milliseconds() > 300){
                    target++;
                    delay.reset();
                }
            }

            if (gamepad1.dpad_left) {
                if (target > 0 && delay.milliseconds()>300) {
                    target--;
                    delay.reset();
                }
            }

            if (gamepad1.dpad_up) {
                if (delay.milliseconds() > 200) {
                    coefs[target] += size;
                    delay.reset();
                }
            }

            if (gamepad1.dpad_down) {
                if (delay.milliseconds() > 200) {
                    coefs[target] -= size;
                    delay.reset();
                }
            }

            if (gamepad1.right_trigger > 0.3) {
                if (delay.milliseconds() > 200) {
                    targetVelocity += 50;
                    delay.reset();
                }
            }

            if (gamepad1.left_trigger > 0.3) {
                if (delay.milliseconds() > 200) {
                    targetVelocity -= 50;
                    delay.reset();
                }
            }

            if(gamepad1.options){
                if(delay.milliseconds()>200){
                    currentAngle+=1;
                    delay.reset();
                }
            }

            if(gamepad1.share){
                if(delay.milliseconds()>200) {
                    currentAngle -= 1;
                    delay.reset();
                }
            }



            telemetry.clear();
            telemetry.addData("P: ", coefs[0]);
            telemetry.addData("I: ", coefs[1]);
            telemetry.addData("D: ", coefs[2]);
            telemetry.addData("F: ", coefs[3]);
            telemetry.addData("Target:", target);
            telemetry.addData("Error:", error);
            telemetry.addData("Angle ",currentAngle);
            telemetry.update();
            telemetryM.addData("Velocity:",velocity);
            telemetryM.update();
        }
    }

}