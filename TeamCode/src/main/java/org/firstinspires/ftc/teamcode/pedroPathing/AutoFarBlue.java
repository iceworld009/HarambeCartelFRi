package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceOD;
import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceODMan;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.SubSys.Limelight;
import org.firstinspires.ftc.teamcode.SubSys.Motors;
import org.firstinspires.ftc.teamcode.SubSys.Selectioner;
import org.firstinspires.ftc.teamcode.SubSys.Servos;
import org.firstinspires.ftc.teamcode.SubSys.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.PoseStorage;

@Autonomous(name = "Auto Far Blue", group = "Test")
public class AutoFarBlue extends OpMode {

    private static final int REP = 9000;                    // max poll iterations while ramping flywheel
    private static final int TELEMETRY_UPDATE_INTERVAL = 25;
    private static final double DEFAULT_TARGET_VELOCITY = 3850;
    private static final double RAMP_THRESHOLD = -60;

    private static final double START_DELAY_SECONDS = 22;
    private static final double PARK_MAX_POWER = 0.8;
    private static final int PRE_PARK_SLEEP_MS = 2000;
    private static final double PARK_BRAKING_START = 3;
    private static final double PARK_BRAKING_STRENGTH = 0.4;

    private static final double VISION_OFFSET_DEADBAND = 2;
    private static final double VISION_OFFSET_GAIN = 0.18;
    private static final double VISION_OFFSET_DECAY = 0.8;
    private static final double AIM_Y_OFFSET = 1; // small vertical aim correction

    private final Pose startPose = new Pose(133.6, 86.95, Math.toRadians(180));
    private final Pose parkPose = new Pose(100, 89, Math.toRadians(180));

    // Hardware / subsystem references
    private DcMotor FR, FL, BR, BL;
    private Follower follower;
    private Servos servos;
    private Motors motors;
    private Limelight limelight;
    private Selectioner selectioner;
    private HardwareClass hardwareClass;
    private Turret turret;
    private TelemetryManager telemetryM;

    private Timer pathTimer, opmodeTimer;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime sleepyyeah = new ElapsedTime();

    // State
    private int pathState;
    private int target = 0;
    private int possibleGreenPos = -1;
    private double shoots = 0;

    private double targetAngle, targetPosition, distance;
    private double visionOffset;
    private double x, y;
    private Pose botPose;
    private double error;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    private PathChain park;

    public void buildPaths() {
        park = follower.pathBuilder()
                .addPath(new BezierLine(startPose, parkPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration()
                .setBrakingStart(PARK_BRAKING_START)
                .setBrakingStrength(PARK_BRAKING_STRENGTH)
                .build();
    }

    private void hold(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(-power);
    }

    private void waitForRampThenStop(double rampThreshold) {
        for (int i = 0; i < REP; i++) {
            if (i % TELEMETRY_UPDATE_INTERVAL == 0) {
                errorTelemetry();
            }
            if (motors.getRampError() > rampThreshold) {
                break;
            }
        }
        stopPresiune();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (sleepyyeah.seconds() > START_DELAY_SECONDS) {
                    startPresiune();
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    waitForRampThenStop(RAMP_THRESHOLD);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(PARK_MAX_POWER);
                    sleep(PRE_PARK_SLEEP_MS);
                    follower.followPath(park, true);
                    setPathState(100);
                }
                break;

            case 100:
                if (!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        updatePosition();

        distance = getDistanceODMan(x, y, HardwareClass.autoRedScorePoseX, HardwareClass.autoRedScorePoseY);

        error = motors.getRampError();

        if (target != -1) {
            updateTurretFusion();
        }

        autonomousPathUpdate();


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM", motors.getVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Distance to target", distance);
        telemetry.addData("Timer", timer.milliseconds());
        telemetry.addData("PreAutoSeek", possibleGreenPos);
        telemetryM.addData("Rpm", motors.getVelocity());
        telemetry.update();
    }

    public void errorTelemetry() {
        telemetry.clear();
        telemetry.addData("Error:", motors.getRampError());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        hardwareClass = HardwareClass.getInstance(hardwareMap);
        servos = Servos.getInstance(hardwareMap, telemetry);
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap, telemetry);
        limelight.setup();
        limelight.setPipeline(1);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        FL = hardwareClass.FL;
        FR = hardwareClass.FR;
        BL = hardwareClass.BL;
        BR = hardwareClass.BR;


        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();

        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        distance = getDistanceOD(follower.getPose().getX(), follower.getPose().getY(), target);
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        selectioner.setTagPos(possibleGreenPos);
        limelight.stop();
        turret.powerOn();
        setPathState(0);
        sleepyyeah.reset();
    }

    private void sleep(int delayMs) {
        try {
            Thread.sleep(delayMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }

    public void startPresiune() {
        hold(0.18);
        selectioner.resetServos();
        motors.intakeOn();
        motors.setCoefsMan(12, 0, 0, 3.5);
        motors.setRampVelocityC((int) targetVelocity);
    }

    public void stopPresiune() {
        PoseStorage.autoPose = follower.getPose();
        selectioner.unloadBallsSlow();
        motors.setRampVelocityC((int) (0.33 * targetVelocity));
        sleep(100);
        motors.intakeOff();
        hold(0);
    }

    private void resetTurret() {
        target = -1;
        turret.goToPosition(1400);
        sleep(500);
        turret.resetMotor();
        turret.goToPosition((HardwareClass.turret_max + HardwareClass.turret_min) / 2.0);
        target = 1;
        sleep(500);
        turret.powerOFF();
    }

    private void updatePosition() {
        botPose = follower.getPose();
        x = botPose.getX();
        y = botPose.getY();
    }

    private void updateTurretFusion() {
        limelight.setPipeline(0);
        double dx = HardwareClass.autoBlueScorePoseX - x;
        double dy = HardwareClass.autoBlueScorePoseY - y + AIM_Y_OFFSET;

        double goalAngle = Math.atan2(dy, dx);
        double thetaR = botPose.getHeading();

        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

        targetPosition = convertToNewRange(
                targetAngle,
                -2 * Math.PI / 3, 2 * Math.PI / 3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        if (limelight.checkResults()) {
            double tx = limelight.getXPos();
            if (Math.abs(tx) > VISION_OFFSET_DEADBAND) {
                visionOffset -= tx * VISION_OFFSET_GAIN;
            }
        } else {
            visionOffset *= VISION_OFFSET_DECAY;
        }
        targetPosition += visionOffset;

        targetPosition = Math.min(Math.max(targetPosition, HardwareClass.turret_min), HardwareClass.turret_max);
        turret.goToPosition(targetPosition);
    }

    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    @Override
    public void stop() {
        motors.rampStop();
        turret.stop();
    }
}