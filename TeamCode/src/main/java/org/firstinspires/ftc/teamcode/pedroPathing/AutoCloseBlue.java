package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceOD;
import static org.firstinspires.ftc.teamcode.Utils.Geometry.getDistanceODMan;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Auto Close Blue", group = "Test")
public class AutoCloseBlue extends OpMode {
    private static final int REP = 3000;
    private static final int TELEMETRY_UPDATE_INTERVAL = 50;
    private static final double DEFAULT_TARGET_VELOCITY = 2650;
    private static final double PRELOAD_RAMP_THRESHOLD = -110;
    private static final double STANDARD_RAMP_THRESHOLD = -50;
    private static final double FINAL_RAMP_THRESHOLD = -65;

    private static final long UNLOAD_WAIT_STAGE1_MS = 1100;
    private static final long UNLOAD_WAIT_STAGE2_MS = 1100;
    private static final long UNLOAD_WAIT_STAGE3_MS = 1000;
    private static final int POST_UNLOAD_SLEEP_MS = 800;

    // Cutoff so the last shot doesn't run out of auto time
    private static final double OPMODE_TIME_LIMIT_FINAL_S = 28.8;
    private final Pose startPose = new Pose(27.7, 130.85, Math.toRadians(138));
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(60, 82 , Math.toRadians(180)); // scorePose used only by turret
    private final Pose pickup1_3Pose = new Pose(25, 82, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(42, 61.5, Math.toRadians(180));
    private final Pose pickup2_3Pose = new Pose(17.5, 61.5, Math.toRadians(180));
    private final Pose aux_2 = new Pose(60, 60, Math.toRadians(180));

    private final Pose parkPose = new Pose(36, 84, Math.toRadians(180));
    private final Pose unloadPose = new Pose(14.25, 61.5, Math.toRadians(153)); //14 61.75 153
    private final Pose unloadPose2 = new Pose(14.25, 61.5, Math.toRadians(153));
    private final Pose aux = new Pose(60, 60, Math.toRadians(180));

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
    private ElapsedTime unloadTimer = new ElapsedTime();
    Thread updateTurret = null;
    // State
    private int pathState;
    private int target = 0;
    private int possibleGreenPos = -1;

    private double targetAngle, targetPosition, distance;
    private double x, y;
    private Pose botPose;
    private double error;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    private PathChain scorePreloadChain, grabFirst, grabFromRack, scoreFromRack, parkBot;
    private PathChain grabSecond, grabFromRack2, scoreFromRack2, scoreFromRack3;

    public void buildPaths() {
        scorePreloadChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0, () -> {
                    selectioner.resetServos();
                    startPresiune();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        grabFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.5, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                .addPath(new BezierCurve(pickup2_3Pose, aux_2,scorePose1))
                .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.5, () -> motors.intakeReverse())
                .addParametricCallback(0.1, this::startPresiune)
                .addParametricCallback(0.95, () -> {
                    motors.intakeOff();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        grabSecond = follower.pathBuilder()
                .setGlobalDeceleration(0.8)
                .addPath(new BezierLine(scorePose1, pickup1_3Pose))
                .setBrakingStrength(1.2)
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1_3Pose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup1_3Pose, scorePose1))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.1, this::startPresiune)
                .addParametricCallback(0.65, () -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        grabFromRack = follower.pathBuilder()
                .setGlobalDeceleration(1)
                .addPath(new BezierCurve(scorePose1, aux_2, unloadPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), unloadPose.getHeading(), 0.5)
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addParametricCallback(1, () -> {
                    unloadTimer.reset();
                    hold(0.6);
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        grabFromRack2 = follower.pathBuilder()
                .setGlobalDeceleration(1)
                .addPath(new BezierCurve(scorePose1, aux,unloadPose2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), unloadPose2.getHeading(),0.5)
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addParametricCallback(0.9, () -> {
                    unloadTimer.reset();
                    hold(0.6);
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        scoreFromRack = follower.pathBuilder()
                .addPath(new BezierCurve(unloadPose, aux_2, scorePose1))
                .setLinearHeadingInterpolation(unloadPose.getHeading(), aux_2.getHeading())
                .addParametricCallback(0, () -> motors.intakeReverse())
                .setLinearHeadingInterpolation(aux_2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0, this::startPresiune)
                .addParametricCallback(0.45, () -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        scoreFromRack2 = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose2, scorePose1))
                .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeReverse();
                    startPresiune();
                })
                .addParametricCallback(0.3, () -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();

        scoreFromRack3 = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose2, scorePose1))
                .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeReverse();
                    startPresiune();
                })
                .addParametricCallback(0.3, () -> {
                    motors.intakeReverse();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.6)
                .build();
        parkBot = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .setGlobalDeceleration(0.8)
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
     * then stops/unloads.
     * Telemetry is throttled to avoid hammering telemetry.update() up to 3000x per call.
     */
    private void waitForRampThenStop(double rampThreshold) {
        waitForRampThenStop(rampThreshold, Double.MAX_VALUE);
    }

    private void waitForRampThenStop(double rampThreshold, double opmodeTimeLimitSeconds) {
        motors.intakeOn();
        for (int i = 0; i < REP && opmodeTimer.getElapsedTimeSeconds() < opmodeTimeLimitSeconds; i++) {
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
                follower.followPath(scorePreloadChain, true);
                setPathState(1);
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
                    follower.followPath(grabFromRack, 0.9,true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && unloadTimer.milliseconds() > UNLOAD_WAIT_STAGE1_MS) {
                    sleep(POST_UNLOAD_SLEEP_MS);
                    hold(0);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(scoreFromRack, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(grabSecond, 0.9, true);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(11); // jumps to 16 !!!!!!!
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(grabFromRack2, 0.9,true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && unloadTimer.milliseconds() > UNLOAD_WAIT_STAGE2_MS) {
                    hold(0);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(scoreFromRack2, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(grabFromRack2,0.9, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy() && unloadTimer.milliseconds() > UNLOAD_WAIT_STAGE3_MS) {
                    setPathState(18);
                    hold(0);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(scoreFromRack3, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    PoseStorage.autoPose = follower.getPose();
                    waitForRampThenStop(FINAL_RAMP_THRESHOLD, OPMODE_TIME_LIMIT_FINAL_S);
                    follower.followPath(parkBot,0.9,true);
                    setPathState(20);
                }
                break;

            case 20:
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

        distance = getDistanceODMan(x, y, HardwareClass.autoRedScorePoseX, HardwareClass.autoRedScorePoseY);
        error = motors.getRampError();

        if (target != -1) {
            updateTurret();
        }

        autonomousPathUpdate();

        distance = getDistanceOD(follower.getPose().getX(), follower.getPose().getY(), target);

        telemetry.addData("path state", pathState);
        telemetry.addData("RPM", motors.getVelocity());
        telemetry.addData("Error:",error);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Distance to target", distance);
        telemetry.addData("Timer", timer.milliseconds());
        telemetry.update();
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        FL = hardwareClass.FL;
        FR = hardwareClass.FR;
        BL = hardwareClass.BL;
        BR = hardwareClass.BR;
        motors.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        unloadTimer = new ElapsedTime();

        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();

        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        distance =getDistanceOD(follower.getPose().getX(), follower.getPose().getY(), target);
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
        hold(0.3);
        motors.intakeOn();
        motors.setCoefsMan(12, 0, 0, 3.5);
        motors.setRampVelocityC((int) targetVelocity);
    }

    public void stopPresiune() {
        PoseStorage.autoPose = follower.getPose();
        selectioner.unloadBalls();
        sleep(50);
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
        turret.goToPosition(turret.getPosition());
        target = 1;
        sleep(100);
        turret.powerOFF();
    }

    private void updatePosition() {
        botPose = follower.getPose();
        x = botPose.getX();
        y = botPose.getY();
    }

    private void updateTurret() {
        limelight.setPipeline(0);
        double dx = HardwareClass.autoBlueScorePoseX - x;
        double dy = HardwareClass.autoBlueScorePoseY - y;

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
        motors.rampStop();
        turret.stop();
    }
}