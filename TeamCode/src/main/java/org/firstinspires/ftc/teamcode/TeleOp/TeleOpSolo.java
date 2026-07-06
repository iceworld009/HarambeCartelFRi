package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.GeneralUtils.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.SubSys.Limelight;
import org.firstinspires.ftc.teamcode.SubSys.Motors;
import org.firstinspires.ftc.teamcode.SubSys.Selectioner;
import org.firstinspires.ftc.teamcode.SubSys.Servos;
import org.firstinspires.ftc.teamcode.SubSys.Turret;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="TeleOp Solo", group = "Solo")
public class TeleOpSolo extends LinearOpMode {

    HardwareClass  hardwareClass;
    Motors motors;
    Servos servos;
    Selectioner selectioner;
    Holonomic holonomic;
    Turret turret;
    Limelight limelight;
    Follower follower;
    TelemetryManager telemetryM;
    Pose BotPose;
    Thread updateTurret;

    double targetVelocity, distance, error;
    double x,y;
    double target, targetPosition;


    public void runOpMode(){
        //Phase 1
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        servos = Servos.getInstance(hardwareMap , telemetry);
        motors = Motors.getInstance(hardwareMap);
        turret = new Turret(hardwareClass,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        holonomic = Holonomic.getInstance(hardwareMap , gamepad1, gamepad2);
        limelight = Limelight.getInstance(hardwareMap,telemetry);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        selectioner.checkColors();
        motors.setRampCoefs(hardwareMap.voltageSensor.iterator().next().getVoltage());

        //Phase 2
        turret.setup();
        telemetry.addLine("Ready! ");
        telemetry.update();
        waitForStart();
        if(!holonomic.getStatus()){
            holonomic.start();
        }
        selectioner.resetServos();
        turret.resetTurret();
        startUpdate();

        try {
            while (opModeIsActive()) {

                if(gamepad1.right_trigger > 0.3){
                    motors.intakeOn();
                } else if(gamepad1.left_trigger > 0.3 ){
                    motors.intakeReverse();
                } else motors.intakeOff();

                    


            }
        } finally {
            if (turret != null) {
                turret.stop();
            }

            if(holonomic.getStatus()){
                holonomic.stop();
            }
        }

    }

    public void startUpdate() {
        boolean running = true;
        if (updateTurret == null || !updateTurret.isAlive()) {
            updateTurret = new Thread(() -> {
                while (running) {
                    updatePosition();
                    if(target == 0 || target == 1 || target == 10) {
                        updateTurret();
                    }
                    sleep(10); // foloseste prea mult cpu !!!
                }
            });
            updateTurret.start();
        }
    }
    void updateTurret() {

        double dx = 0, dy = 0;

        if(target == 1) {
            dx = HardwareClass.blueX - x;
            dy = HardwareClass.blueY - y;
        }
        else if(target == 0) {
            dx = HardwareClass.redX - x;
            dy = HardwareClass.redY - y;
        }

        double goalAngle = Math.atan2(dy, dx);
        double robotHeading = BotPose.getHeading();

        double relativeAngle = goalAngle - robotHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        targetPosition = convertToNewRange(
                relativeAngle,
                -2 * Math.PI / 3, 2 * Math.PI / 3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        targetPosition = Math.max(
                HardwareClass.turret_min+10,
                Math.min(HardwareClass.turret_max-10, targetPosition)
        );
        turret.goToPosition(targetPosition);
    }

    double getTurretError(){
        return turret.getPosition()-targetPosition;
    }

    void updatePosition(){
        follower.update();
        BotPose = follower.getPose();
        x = BotPose.getX();
        y = BotPose.getY();
    }

    void updateTelemetry(){

        telemetry.clearAll();
        telemetry.addData("x" , x);
        telemetry.addData("y" , y);
        telemetry.addData("Distance: " , distance);
        telemetry.addData("Velocity",motors.getVelocity());
        telemetry.addData("Turret error",getTurretError());
        telemetry.addData("Velocity error",error);
        telemetry.update();
    }

}
