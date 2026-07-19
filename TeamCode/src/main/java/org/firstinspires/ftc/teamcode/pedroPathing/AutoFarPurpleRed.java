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
import org.firstinspires.ftc.teamcode.PoseStorage;

@Autonomous(name = "Auto Far Purple Red", group = "Test")
public class AutoFarPurpleRed extends OpMode {

    private static final int REP = 4500;                    // max poll iterations while ramping flywheel
    private static final int TELEMETRY_UPDATE_INTERVAL = 35; // only push telemetry every N iterations
    /** needs adjustment !!!
     *  try to rise and lower the rpm in magnitude of 25 rpm +-
     */
    private static final double DEFAULT_TARGET_VELOCITY = 3675;  //3900 +-


    private static final double PRELOAD_RAMP_THRESHOLD = -60;
    private static final double STANDARD_RAMP_THRESHOLD = -80;

    private static final int PIPELINE_PRESEEK = 1;
    private static final int PIPELINE_AIM = -4;

    private static final double AIM_X_OFFSET = -2;

    private static final int POST_SHOT_SLEEP_MS = 80;
    private static final int TURRET_THREAD_PERIOD_MS = 10;



    private final Pose startPose = new Pose(83.56, 0.4, Math.toRadians(0));
    private final Pose scorePose = new Pose(87, -17, Math.toRadians(0));
    private final Pose pickupScore1 = new Pose(101, 34, Math.toRadians(0));
    private final Pose pickupScore1_3 = new Pose(125, 34, Math.toRadians(0));
    private final Pose pickupPose2_2 = new Pose(125, 10, Math.toRadians(0));
    private final Pose pickupPose2_1 = new Pose(125, 0, Math.toRadians(0));
    private final Pose parkPose = new Pose(22,0,Math.toRadians(0));

    // Hardware
    private DcMotor FR, FL, BR, BL;
    private Follower follower;
    private Servos servos;
    private Motors motors;
    private Limelight limelight;
    private Selectioner selectioner;
    private HardwareClass hardwareClass;
    private Turret turret;
    private TelemetryManager telemetryM;

    private Thread turretThread = null;
    private volatile boolean turretThreadRunning = false;

    private Timer pathTimer, opmodeTimer;
    private final ElapsedTime timer = new ElapsedTime();

    // State
    private int pathState;
    private int target = 1;
    private int possibleGreenPos = -1;
    private double shoots = 0;

    private double targetAngle, targetPosition, distance;
    private double x, y;
    private Pose botPose;
    private double error;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    private PathChain scorePreload, grabFirst, grabHuman,grabHuman2, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupScore1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupScore1.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickupScore1, pickupScore1_3))
                .setLinearHeadingInterpolation(pickupScore1.getHeading(), pickupScore1_3.getHeading())
                .addPath(new BezierLine(pickupScore1_3, scorePose))
                .setLinearHeadingInterpolation(pickupScore1_3.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, this::startPresiune)
                .addParametricCallback(0.35, () -> motors.intakeReverse())
                .setGlobalDeceleration(0.4)
                .build();

        grabHuman = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose2_2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose2_2.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickupPose2_2, scorePose))
                .setLinearHeadingInterpolation(pickupPose2_2.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, this::startPresiune2)
                .addParametricCallback(0.4, () -> motors.intakeReverse())
                .addParametricCallback(0.5, () -> motors.intakeOn())
                .addParametricCallback(0.6, () -> motors.intakeReverse())
                .addParametricCallback(0.95, () -> {
                    motors.intakeOff();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        grabHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupPose2_1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupPose2_1.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickupPose2_1, scorePose))
                .setLinearHeadingInterpolation(pickupPose2_1.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, this::startPresiune2)
                .addParametricCallback(0.4, () -> motors.intakeReverse())
                .addParametricCallback(0.5, () -> motors.intakeOn())
                .addParametricCallback(0.6, () -> motors.intakeReverse())
                .addParametricCallback(0.95, () -> {
                    motors.intakeOff();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.5)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupScore1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupScore1.getHeading())
                .setGlobalDeceleration(0.5)
                .build();
    }

    private void hold(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(-power);
    }

    /**
     * Spins up the flywheel and polls the ramp error until it settles (or REP iterations pass),
     */
    private void waitForRampThenStop(double rampThreshold) {
        motors.intakeOn();
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
                setPathState(1);
                follower.followPath(scorePreload, true);
                startPresiune();
                break;

            case 1:
                if (!follower.isBusy()) {
                    waitForRampThenStop(PRELOAD_RAMP_THRESHOLD);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(grabFirst, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(grabHuman2, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(grabHuman, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(grabHuman2, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grabHuman, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    follower.followPath(park, true);
                    setPathState(100);
                }
                break;

            case 100:
                if (!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    setPathState(-1);
                    hold(0);
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
        //follower.update();
        //updatePosition();

        distance = getDistanceODMan(x, y, HardwareClass.autoRedScorePoseX, HardwareClass.autoRedScorePoseY);
        error = motors.getRampError();
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
        limelight.setPipeline(PIPELINE_PRESEEK);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        motors.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        FL = hardwareClass.FL;
        FR = hardwareClass.FR;
        BL = hardwareClass.BL;
        BR = hardwareClass.BR;


        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetTurret();

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
        startUpdate();
        setPathState(0);
        servos.hoodSetPos(0.5);
    }

    private void sleep(int delayMs) {
        try {
            Thread.sleep(delayMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new RuntimeException(e);
        }
    }

    public void startUpdate() {
        if (turretThread == null || !turretThread.isAlive()) {
            turretThreadRunning = true;
            turretThread = new Thread(() -> {
                while (turretThreadRunning) {
                    updatePosition();
                    follower.update();
                    if (target == 0 || target == 1 || target == 10) {
                        updateTurret();
                    }
                    try {
                        Thread.sleep(TURRET_THREAD_PERIOD_MS);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }
            });
            turretThread.start();
        }
    }

    private void stopUpdate() {
        turretThreadRunning = false;
        if (turretThread != null) {
            turretThread.interrupt();
            try {
                turretThread.join(200);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public void startPresiune() {
        hold(0.25);
        selectioner.resetServos();
        motors.intakeOn();
        motors.setCoefsMan(12, 0, 0, 3.5, hardwareMap.voltageSensor.iterator().next().getVoltage());
        motors.setRampVelocityC((int) targetVelocity);
    }

    public void startPresiune2() {
        hold(0.25);
        selectioner.resetServos();
        motors.intakeOn();
        motors.setCoefsMan(12, 0, 0, 3.5, hardwareMap.voltageSensor.iterator().next().getVoltage());
        motors.setRampVelocityC((int) targetVelocity-60);
    }

    public void stopPresiune() {
        PoseStorage.autoPose = follower.getPose();
        selectioner.unloadBallsSlow();
        motors.setRampVelocityC((int) (0.33 * targetVelocity));
        sleep(POST_SHOT_SLEEP_MS);
        motors.intakeOff();
        hold(0);
    }

    private void updatePosition() {
        botPose = follower.getPose();
        x = botPose.getX();
        y = botPose.getY();
    }

    /** Odometry-driven turret aim */
    private void updateTurret() {
        limelight.setPipeline(PIPELINE_AIM);
        double dx = HardwareClass.autoArtilleryScorePoseX - x + AIM_X_OFFSET;
        double dy = HardwareClass.autoArtilleryScorePoseY - y;

        double goalAngle = Math.atan2(dy, dx);
        double thetaR = botPose.getHeading();

        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

        targetPosition = convertToNewRange(
                targetAngle,
                -2 * Math.PI / 3, 2 * Math.PI / 3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        targetPosition = Math.min(Math.max(targetPosition, HardwareClass.turret_min), HardwareClass.turret_max);
        turret.goToPosition(targetPosition);
    }

    public double convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    @Override
    public void stop() {
        stopUpdate();
        motors.rampStop();
        turret.stop();
        hold(0);
    }
}
