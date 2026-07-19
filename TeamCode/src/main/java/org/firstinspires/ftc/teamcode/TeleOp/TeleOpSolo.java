package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utils.GeneralUtils.*;
import static org.firstinspires.ftc.teamcode.Utils.Geometry.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.SubSys.Limelight;
import org.firstinspires.ftc.teamcode.SubSys.Motors;
import org.firstinspires.ftc.teamcode.SubSys.Selectioner;
import org.firstinspires.ftc.teamcode.SubSys.Servos;
import org.firstinspires.ftc.teamcode.SubSys.Turret;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp Solo", group = "Solo")
public class TeleOpSolo extends LinearOpMode {

    private static final int TURRET_THREAD_PERIOD_MS = 10;
    private static final int SHOT_RAMP_OVERSHOOT_RPM = 300;
    private static final int SHOT_KICK_SLEEP_MS = 400;
    private static final double SHOOT_DONE_RAMP_ERROR = -50;
    private static final int POST_SHOT_SLEEP_MS = 100;
    private static final double IDLE_RPM = 1600;
    private static final double IDLE_RAMP_ERROR_CEILING = 1000;

    HardwareClass hardwareClass;
    Motors motors;
    Servos servos;
    Selectioner selectioner;
    Holonomic holonomic;
    Turret turret;
    Limelight limelight;
    Follower follower;
    TelemetryManager telemetryM;
    volatile Pose BotPose;
    Thread updateTurret;
    private volatile boolean turretThreadRunning = false;

    double targetVelocity, distance, error, targetPosition;
    volatile double x, y;
    volatile int target;

    boolean isShooting = false, canIdle = false;
    ElapsedTime check = new ElapsedTime();
    ElapsedTime temp = new ElapsedTime();
    int tempVar = 0;

    boolean prevRightBumper, prevLeftBumper, prevX, prevDpadUp, prevDpadLeft, prevDpadRight,prevDpadDown;

    public void runOpMode() {
        // Phase 1
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        if (PoseStorage.autoPose != null)
            follower.setStartingPose(PoseStorage.autoPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        servos = Servos.getInstance(hardwareMap, telemetry);
        motors = Motors.getInstance(hardwareMap);
        turret = new Turret(hardwareClass, telemetry);
        holonomic = Holonomic.getInstance(hardwareMap, gamepad1, gamepad2);
        limelight = Limelight.getInstance(hardwareMap, telemetry);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        selectioner.checkColors();
        motors.setRampCoefs(hardwareMap.voltageSensor.iterator().next().getVoltage());
        motors.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Phase 2
        turret.setup();
        telemetry.addLine("Ready! ");
        telemetry.update();
        waitForStart();
        if (!holonomic.getStatus()) {
            holonomic.start();
        }
        selectioner.resetServos();
        turret.resetTurret();
        startUpdate();

        try {
            while (opModeIsActive()) {

                distance = getDistanceOD(x, y, target);
                error = motors.getRampError();
                targetVelocity = getRPM(distance);

                if(distance > 300)
                    servos.hoodSetPos(0.55);
                else if(y<0) servos.hoodSetPos(0.2);
                else servos.hoodSetPos(0.3);

                if (check.milliseconds() > 60) {
                    updateTelemetry();
                    check.reset();
                }

//                if(temp.milliseconds()>300){
//                    if(gamepad1.a){
//                        tempVar -= 30;
//                    }
//                    if(gamepad1.y){
//                        tempVar += 30;
//                    }
//                    temp.reset();
//                }

                if (motors.getRampError() > SHOOT_DONE_RAMP_ERROR && isShooting) {
                    selectioner.unloadBalls();
                    sleep(POST_SHOT_SLEEP_MS);
                    motors.rampStop();
                    isShooting = false;
                }

                if (canIdle && !isShooting) {
                    if (motors.getRampError() < IDLE_RAMP_ERROR_CEILING)
                        motors.setRampVelocityC((int) IDLE_RPM);
                }

                if (gamepad1.right_trigger > 0.3) {
                    motors.intakeOn();
                } else if (gamepad1.left_trigger > 0.3) {
                    motors.intakeReverse();
                } else motors.intakeOff();

                if (gamepad1.x && !prevX) {
                    selectioner.unloadBallsSlow();
                }

                if (gamepad1.right_bumper && !prevRightBumper) {
                    isShooting = true;
                    canIdle = true;
                    motors.setRampVelocityC((getRPM(distance) + SHOT_RAMP_OVERSHOOT_RPM + tempVar));
                    sleep(SHOT_KICK_SLEEP_MS);
                    motors.setRampVelocityC( getRPM(distance) + tempVar);
                }

                if (gamepad1.left_bumper && !prevLeftBumper) {
                    isShooting = false;
                    canIdle = false;
                    motors.rampStop();
                }

                if (gamepad1.right_stick_button) {
                    target = 1;
                } else if (gamepad1.left_stick_button) {
                    target = 0;
                } else if (gamepad1.ps) {
                    target = 3;
                }

                if (gamepad1.dpad_up && !prevDpadUp) {
                    target = -1;
                    turret.resetTurret();
                    target = 3;
                }
                if (gamepad1.dpad_left && !prevDpadLeft)
                    follower.setPose(PoseStorage.bluePose);

                if (gamepad1.dpad_right && !prevDpadRight)
                    follower.setPose(PoseStorage.redPose);

                if(gamepad1.dpad_down && !prevDpadDown)
                    follower.setPose(PoseStorage.resetPose);

                prevX = gamepad1.x;
                prevRightBumper = gamepad1.right_bumper;
                prevLeftBumper = gamepad1.left_bumper;
                prevDpadUp = gamepad1.dpad_up;
                prevDpadLeft = gamepad1.dpad_left;
                prevDpadRight = gamepad1.dpad_right;
                prevDpadDown = gamepad1.dpad_down;
            }
        } finally {
            stopUpdate();

            if (turret != null) {
                turret.stop();
            }

            if (holonomic.getStatus()) {
                holonomic.stop();
            }
        }
    }

    public void startUpdate() {
        if (updateTurret == null || !updateTurret.isAlive()) {
            turretThreadRunning = true;
            updateTurret = new Thread(() -> {
                while (turretThreadRunning) {
                    updatePosition();
                    if (target == 0 || target == 1 || target == 3) {
                        updateTurret();
                    }
                    sleep(TURRET_THREAD_PERIOD_MS);
                }
            });
            updateTurret.start();
        }
    }

    private void stopUpdate() {
        turretThreadRunning = false;
        if (updateTurret != null) {
            updateTurret.interrupt();
            try {
                updateTurret.join(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    void updateTurret() {

        double dx = 0, dy = 0;

        if (target == 1) {
            dx = HardwareClass.blueX - x;
            dy = HardwareClass.blueY - y;
        } else if (target == 0) {
            dx = HardwareClass.redX - x;
            dy = HardwareClass.redY - y;
        } else if (target == 3) {
            dx = HardwareClass.autoArtilleryScorePoseX - x;
            dy = HardwareClass.autoArtilleryScorePoseY - y;
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
                HardwareClass.turret_min + 10,
                Math.min(HardwareClass.turret_max - 10, targetPosition)
        );
        turret.goToPosition(targetPosition);
    }
    double getTurretError() {
        return turret.getPosition() - targetPosition;
    }

    void updatePosition() {
        follower.update();
        BotPose = follower.getPose();
        x = BotPose.getX();
        y = BotPose.getY();

    }

    double getRandomSpeed(){
        double valRand = Math.random();
        return valRand % 100 / 100.0;
        /* TODO: If you have time, its fun ;;;))) */
    }

    void updateTelemetry() {

        telemetry.clearAll();
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Velocity", motors.getVelocity());
        telemetry.addData("TargetVelocity:", targetVelocity + tempVar);
        telemetry.addData("Turret error", getTurretError());
        telemetry.addData("Velocity error", error);
        telemetryM.addData("VelocityTarget", targetVelocity + tempVar);
        telemetryM.addData("Velocity:", motors.getVelocity());
        telemetryM.update();
        telemetry.update();
    }

}