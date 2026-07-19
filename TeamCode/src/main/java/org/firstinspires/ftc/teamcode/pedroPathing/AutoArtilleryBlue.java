package org.firstinspires.ftc.teamcode.pedroPathing;

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

@Autonomous(name = "Auto artillery Blue", group = "Test")
public class AutoArtilleryBlue extends OpMode {
    private static final int REP = 3000;                    // max poll iterations while ramping flywheel
    private static final int TELEMETRY_UPDATE_INTERVAL = 50; // only push telemetry every N iterations
    private static final double DEFAULT_TARGET_VELOCITY = 2915; // TODO: retune for this game

    private static final double PRELOAD_RAMP_THRESHOLD = -150; // TODO: retune
    private static final double STANDARD_RAMP_THRESHOLD = -50; // TODO: retune
    private static final int TURRET_THREAD_PERIOD_MS = 10;

    private static final int MIN_ARTILLERY_CYCLES = 3;
    private static final int MAX_ARTILLERY_CYCLES = 4;
    private static final double ARTILLERY_TIME_CUTOFF_S = 27;
    private static final double OFFSET_X = -7.75;
    private final Pose startPose = new Pose(57.38, -117.35, Math.toRadians(215.79));
    private final Pose shootGeneric = new Pose(49, -72, Math.toRadians(180));
    private final Pose line1Pose = new Pose(42, -82.5, Math.toRadians(180));       // prima linie
    private final Pose line1_3Pose = new Pose(19.7, -82.5, Math.toRadians(180));       // prima linie
    private final Pose line2Pose = new Pose(42, -59, Math.toRadians(180));       // a doua linie
    private final Pose line2_3Pose = new Pose(19.7, -59, Math.toRadians(180));       // a doua linie
    private final Pose line3Pose = new Pose(42, -36, Math.toRadians(180));       // a treia linie
    private final Pose line3_3Pose = new Pose(15, -36, Math.toRadians(180));
    private final Pose auxOnePose = new Pose(50,-82,Math.toRadians(180));
    private final Pose auxTwoPose = new Pose(50, -60, Math.toRadians(180));      // aux_doi
    private final Pose auxThreePose = new Pose(50, -60, Math.toRadians(180));    // aux_trei
    private final Pose controlPoint = new Pose(19,-83.5,0);
    private final Pose shootArtilleryPose = new Pose(37, -84, Math.toRadians(270));
    private final Pose pickupPose = new Pose(24, -126.3, Math.toRadians(270));
    private final Pose pickupPose2 = new Pose(9, -126.3, Math.toRadians(255));
    private final Pose parkPose = new Pose(22, -120, Math.toRadians(270));
    private DcMotor FR, FL, BR, BL;
    private Follower follower;
    private Motors motors;
    private Servos servos;
    private Limelight limelight;
    private Selectioner selectioner;
    private HardwareClass hardwareClass;
    private Turret turret;
    private TelemetryManager telemetryM;

    private Timer pathTimer, opmodeTimer;
    private final ElapsedTime timer = new ElapsedTime();
    private Thread turretThread = null;
    private volatile boolean turretThreadRunning = false;

    // State
    private int pathState;
    private int target = 0;
    private int possibleGreenPos = -1;
    private int artilleryCycle = 0;

    private double targetAngle, targetPosition;
    private double x, y;
    private Pose botPose;
    private double error;
    private double targetVelocity = DEFAULT_TARGET_VELOCITY;

    private PathChain scorePreload;
    private PathChain grabLine1, scoreLine1;
    private PathChain grabLine2, shootLine2;
    private PathChain grabLine3, shootLine3;
    private PathChain grabPickup, shootPickup;
    private PathChain shootPickup2, grabPickup2;
    private PathChain parkRobot;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootGeneric))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootGeneric.getHeading())
                .addParametricCallback(0.1, () ->
                {
                    selectioner.resetServos();
                    startPresiune();
                    PoseStorage.autoPose = follower.getPose();
                })
                .setGlobalDeceleration(0.45)
                .build();

        grabLine1 = follower.pathBuilder()
                .addPath(new BezierLine(shootGeneric, line1Pose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), line1Pose.getHeading(),0.6)
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(line1Pose,line1_3Pose))
                .setLinearHeadingInterpolation(line1Pose.getHeading(),line1_3Pose.getHeading())
                .setGlobalDeceleration(0.5)
                .build();

        scoreLine1 = follower.pathBuilder()
                .addPath(new BezierCurve(line1_3Pose, auxOnePose, shootGeneric))
                .setLinearHeadingInterpolation(line1_3Pose.getHeading(),shootGeneric.getHeading())
                .addParametricCallback(0,() -> {
                    startPresiune();
                    motors.intakeReverse();
                })
                .addParametricCallback(0.9, () -> {
                    motors.intakeOff();
                })
                .setGlobalDeceleration(0.45)
                .build();

        grabLine2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootGeneric,auxTwoPose, line2Pose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), line2Pose.getHeading(),0.8)
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(line2Pose,line2_3Pose))
                .setLinearHeadingInterpolation(line2Pose.getHeading(),line2_3Pose.getHeading())
                .setGlobalDeceleration(0.5)
                .build();

        shootLine2 = follower.pathBuilder()
                .addPath(new BezierCurve(line2_3Pose, auxTwoPose, shootGeneric))
                .setLinearHeadingInterpolation(line2_3Pose.getHeading(),shootGeneric.getHeading())
                .addParametricCallback(0.25,() -> {
                    motors.intakeReverse();
                    startPresiune();
                })
                .setGlobalDeceleration(0.45)
                .build();

        grabLine3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootGeneric, auxThreePose, line3Pose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), line3Pose.getHeading(),0.8)
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(line3Pose,line3_3Pose))
                .setLinearHeadingInterpolation(line3Pose.getHeading(),line3_3Pose.getHeading())
                .setGlobalDeceleration(0.5)
                .build();

        shootLine3 = follower.pathBuilder()
                .addPath(new BezierCurve(line3_3Pose, auxThreePose, shootArtilleryPose))
                .setLinearHeadingInterpolation(line3_3Pose.getHeading(), shootArtilleryPose.getHeading())
                .addParametricCallback(0.2, () -> motors.intakeReverse())
                .setGlobalDeceleration(0.45)
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootArtilleryPose, pickupPose))
                .setLinearHeadingInterpolation(shootArtilleryPose.getHeading(),pickupPose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.3)
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootArtilleryPose, controlPoint, pickupPose2))
                .setLinearHeadingInterpolation(shootArtilleryPose.getHeading(),pickupPose2.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.3)
                .build();

        shootPickup = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootArtilleryPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootArtilleryPose.getHeading())
                .addParametricCallback(0.15, () -> startPresiune())
                .addParametricCallback(0.2, () -> motors.intakeReverse())
                .addParametricCallback(0.75, () -> motors.intakeOff())
                .setGlobalDeceleration(0.45)
                .build();

        shootPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, shootArtilleryPose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), shootArtilleryPose.getHeading())
                .addParametricCallback(0.15, () -> startPresiune())
                .addParametricCallback(0.2, () -> motors.intakeReverse())
                .addParametricCallback(0.75, () -> motors.intakeOff())
                .setGlobalDeceleration(0.45)
                .build();

        parkRobot = follower.pathBuilder()
                .addPath(new BezierLine(shootArtilleryPose, parkPose))
                .setLinearHeadingInterpolation(shootArtilleryPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration(0.45)
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
     * then stops/unloads. Telemetry is throttled to avoid hammering telemetry.update() every iteration.
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
                follower.followPath(scorePreload, true);
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
                    follower.followPath(grabLine1, true);
                    setPathState(3);
                    break;
                }

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scoreLine1, true);
                    setPathState(4);
                    break;
                }

            case 4:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(grabLine2, true);
                    setPathState(7);
                    break;
                }

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(shootLine2, true);
                    setPathState(8);
                    break;
                }

            case 8:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(13); // skips to 13
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(grabLine3, true);
                    setPathState(11);
                    break;
                }

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(shootLine3, true);
                    setPathState(12);
                    break;
                }

            case 12:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    artilleryCycle = 0;
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    servos.hoodSetPos(0.36);
                    follower.followPath(grabPickup2, true);
                    setPathState(15);
                    break;
                }
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup, true);
                    setPathState(16);
                    break;
                }

            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(shootPickup2, true);
                    setPathState(17);
                    break;
                }

            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(shootPickup, true);
                    setPathState(17);
                    break;
                }

            case 17:
                if (!follower.isBusy()) {
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    artilleryCycle++;

                    boolean timeAllowsAnotherCycle = opmodeTimer.getElapsedTimeSeconds() < ARTILLERY_TIME_CUTOFF_S;
                    boolean shouldLoopAgain = artilleryCycle < MIN_ARTILLERY_CYCLES
                            || (artilleryCycle < MAX_ARTILLERY_CYCLES && timeAllowsAnotherCycle);

                    if(shouldLoopAgain){
                        if(artilleryCycle %2 == 1 )
                            setPathState(13);
                        if(artilleryCycle %2 == 0)
                            setPathState(14);
                    } else setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(parkRobot, true);
                    setPathState(19);
                    break;
                }

            case 19:
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
        error = motors.getRampError();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("artillery cycle", artilleryCycle);
        telemetry.addData("RPM", motors.getVelocity());
        telemetry.addData("Error:", error);
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Timer", timer.milliseconds());
        telemetryM.addData("Rpm", motors.getVelocity());
        telemetry.update();
    }

    public void startUpdate() {
        if (turretThread == null || !turretThread.isAlive()) {
            turretThreadRunning = true;
            turretThread = new Thread(() -> {
                while (turretThreadRunning) {
                    follower.update();
                    updatePosition();
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

    /** Signals the background turret thread to exit and waits briefly for it to actually stop. */
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
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap, telemetry);
        limelight.setup();
        limelight.setPipeline(1);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        servos = Servos.getInstance(hardwareMap,telemetry);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        FL = hardwareClass.FL;
        FR = hardwareClass.FR;
        BL = hardwareClass.BL;
        BR = hardwareClass.BR;
        motors.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();

        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        servos.hoodSetPos(0.4);
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
        hold(0.32);
        motors.intakeReverse();
        motors.setCoefsMan(12.5, 0, 0, 4.5, hardwareMap.voltageSensor.iterator().next().getVoltage());
        motors.setRampVelocityC((int) targetVelocity);
    }

    public void stopPresiune() {
        PoseStorage.autoPose = follower.getPose();
        selectioner.unloadBalls();
        motors.intakeOff();
        motors.setRampVelocityC((int) (0.33 * targetVelocity));
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
        double dx = HardwareClass.autoArtilleryScorePoseX - x + OFFSET_X;
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