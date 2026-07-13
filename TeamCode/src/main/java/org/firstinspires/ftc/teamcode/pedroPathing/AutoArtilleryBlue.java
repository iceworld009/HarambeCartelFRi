package org.firstinspires.ftc.teamcode.pedroPathing;

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

@Autonomous(name = "Auto artilery blue", group = "Test")
public class AutoArtilleryBlue extends OpMode {

    private static final int REP = 3000;                    // max poll iterations while ramping flywheel
    private static final int TELEMETRY_UPDATE_INTERVAL = 50; // only push telemetry every N iterations
    private static final double DEFAULT_TARGET_VELOCITY = 2600; // TODO: retune for this game

    private static final double PRELOAD_RAMP_THRESHOLD = -110; // TODO: retune
    private static final double STANDARD_RAMP_THRESHOLD = -50; // TODO: retune
    private static final int TURRET_THREAD_PERIOD_MS = 10;

    private static final int MIN_ARTILLERY_CYCLES = 3;
    private static final int MAX_ARTILLERY_CYCLES = 5;
    private static final double ARTILLERY_TIME_CUTOFF_S = 28.0;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // TODO: heading unknown
    private final Pose shootGeneric = new Pose(0, 0, Math.toRadians(180));
    private final Pose line1Pose = new Pose(0, 0, Math.toRadians(180));       // prima linie
    private final Pose line1_3Pose = new Pose(0, 0, Math.toRadians(180));       // prima linie
    private final Pose line2Pose = new Pose(0, 0, Math.toRadians(180));       // a doua linie
    private final Pose line2_3Pose = new Pose(0, 0, Math.toRadians(180));       // a doua linie

    private final Pose line3Pose = new Pose(0, 0, Math.toRadians(180));       // a treia linie
    private final Pose auxTwoPose = new Pose(0, 0, Math.toRadians(180));      // aux_doi
    private final Pose auxThreePose = new Pose(0, 0, Math.toRadians(180));    // aux_trei
    private final Pose shootArtilleryPose = new Pose(0, 0, Math.toRadians(233));
    private final Pose pickupPose = new Pose(0, 0, Math.toRadians(233));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(233));

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

    private PathChain toShootGeneric;
    private PathChain shootGenericToLine1, line1ToShootGeneric;
    private PathChain shootGenericToAuxTwo, auxTwoToLine2, line2ToShootGeneric;
    private PathChain shootGenericToAuxThree, auxThreeToLine3, line3ToShootArtillery;
    private PathChain shootArtilleryToPickup, pickupToShootArtillery;
    private PathChain shootArtilleryToPark;

    public void buildPaths() {
        toShootGeneric = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootGeneric))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootGeneric.getHeading())
                .setGlobalDeceleration(0.6)
                .build();

        shootGenericToLine1 = follower.pathBuilder()
                .addPath(new BezierLine(shootGeneric, line1Pose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), line1Pose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.6)
                .build();

        line1ToShootGeneric = follower.pathBuilder()
                .addPath(new BezierLine(line1Pose, shootGeneric))
                .setLinearHeadingInterpolation(line1Pose.getHeading(), shootGeneric.getHeading())
                .setGlobalDeceleration(0.6)
                .build();

        shootGenericToAuxTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootGeneric, auxTwoPose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), auxTwoPose.getHeading())
                .setGlobalDeceleration(0.6)
                .build();

        auxTwoToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(auxTwoPose, line2Pose))
                .setLinearHeadingInterpolation(auxTwoPose.getHeading(), line2Pose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.6)
                .build();

        line2ToShootGeneric = follower.pathBuilder()
                .addPath(new BezierLine(line2Pose, shootGeneric))
                .setLinearHeadingInterpolation(line2Pose.getHeading(), shootGeneric.getHeading())
                .setGlobalDeceleration(0.6)
                .build();

        shootGenericToAuxThree = follower.pathBuilder()
                .addPath(new BezierLine(shootGeneric, auxThreePose))
                .setLinearHeadingInterpolation(shootGeneric.getHeading(), auxThreePose.getHeading())
                .setGlobalDeceleration(0.6)
                .build();

        auxThreeToLine3 = follower.pathBuilder()
                .addPath(new BezierLine(auxThreePose, line3Pose))
                .setLinearHeadingInterpolation(auxThreePose.getHeading(), line3Pose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.6)
                .build();

        line3ToShootArtillery = follower.pathBuilder()
                .addPath(new BezierLine(line3Pose, shootArtilleryPose))
                .setLinearHeadingInterpolation(line3Pose.getHeading(), shootArtilleryPose.getHeading())
                .addParametricCallback(0.5, () -> motors.intakeReverse())
                .setGlobalDeceleration(0.6)
                .build();

        shootArtilleryToPickup = follower.pathBuilder()
                .addPath(new BezierLine(shootArtilleryPose, pickupPose))
                .setLinearHeadingInterpolation(shootArtilleryPose.getHeading(), pickupPose.getHeading())
                .addParametricCallback(0, () -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .setGlobalDeceleration(0.6)
                .build();

        pickupToShootArtillery = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootArtilleryPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootArtilleryPose.getHeading())
                .addParametricCallback(0.5, () -> motors.intakeReverse())
                .setGlobalDeceleration(0.6)
                .build();

        shootArtilleryToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootArtilleryPose, parkPose))
                .setLinearHeadingInterpolation(shootArtilleryPose.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration(0.6)
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
                follower.followPath(toShootGeneric, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(PRELOAD_RAMP_THRESHOLD);
                    setPathState(2);
                }
                break;

            case 2:
                follower.followPath(shootGenericToLine1, true);
                setPathState(3);
                break;

            case 3:
                follower.followPath(line1ToShootGeneric, true);
                setPathState(4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(5);
                }
                break;

            case 5:
                follower.followPath(shootGenericToAuxTwo, true);
                setPathState(6);
                break;

            case 6:
                follower.followPath(auxTwoToLine2, true);
                setPathState(7);
                break;

            case 7:
                follower.followPath(line2ToShootGeneric, true);
                setPathState(8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    setPathState(9);
                }
                break;

            case 9:
                follower.followPath(shootGenericToAuxThree, true);
                setPathState(10);
                break;

            case 10:
                follower.followPath(auxThreeToLine3, true);
                setPathState(11);
                break;

            case 11:
                follower.followPath(line3ToShootArtillery, true);
                setPathState(12);
                break;

            case 12:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    artilleryCycle = 0;
                    setPathState(13);
                }
                break;

            case 13:
                follower.followPath(shootArtilleryToPickup, true);
                setPathState(14);
                break;

            case 14:
                follower.followPath(pickupToShootArtillery, true);
                setPathState(15);
                break;

            case 15:
                if (!follower.isBusy()) {
                    startPresiune();
                    waitForRampThenStop(STANDARD_RAMP_THRESHOLD);
                    artilleryCycle++;

                    boolean timeAllowsAnotherCycle = opmodeTimer.getElapsedTimeSeconds() < ARTILLERY_TIME_CUTOFF_S;
                    boolean shouldLoopAgain = artilleryCycle < MIN_ARTILLERY_CYCLES
                            || (artilleryCycle < MAX_ARTILLERY_CYCLES && timeAllowsAnotherCycle);

                    setPathState(shouldLoopAgain ? 13 : 16);
                }
                break;

            case 16:
                follower.followPath(shootArtilleryToPark, true);
                setPathState(17);
                break;

            case 17:
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

        turret = new Turret(hardwareClass, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();

        follower.setStartingPose(startPose);
        motors.setRampCoefs();
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
        motors.intakeOn();
        motors.setCoefsMan(12, 0, 0, 3.5, hardwareMap.voltageSensor.iterator().next().getVoltage());
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
        double dx = HardwareClass.autoArtilleryScorePoseX - x;
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