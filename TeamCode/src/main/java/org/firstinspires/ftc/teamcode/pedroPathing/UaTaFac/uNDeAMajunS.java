package org.firstinspires.ftc.teamcode.pedroPathing.UaTaFac;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;


@Autonomous(name = "uNDeAMajunS" , group = "Test")
public class uNDeAMajunS extends OpMode {
    int target=0;
    private final Pose startPose = new Pose(14.8, 118.7, 2.26); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 72  ,Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(66.1,105,Math.toRadians(90)); // scorePose 1 doar cu turreta
    private final Pose pickup1Pose = new Pose(58, 99, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1_3Pose = new Pose(68.1, 117, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(78, 95   , Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose aux_2 = new Pose(82, 110   , Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2_3Pose = new Pose(81,122,Math.toRadians(90));
    private final Pose pickup2_3Pose1 = new Pose(82,123,Math.toRadians(30));
    private final Pose parkPose = new Pose(40.7, 91, 2.116); // Park // Park
    private final Pose unloadPose = new Pose(81,133.7,Math.toRadians(114));
    private final Pose unloadPose2 = new Pose(82.2,132,Math.toRadians(116));
    private final Pose aux = new Pose (79.8 , 122  , Math.toRadians(130));

    DcMotor FR , FL , BR , BL;

    double hood_offset_1 = 0.1;
    double hood_offset_2 = 0.05;
    double hood_offset_3 = 0;
    double shoots = 0;
    private Follower follower;
    private final ElapsedTime delay = new ElapsedTime();
    Servos servos = null;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass= null;
    Turret turret = null;
    private TelemetryManager telemetryM;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime path = new ElapsedTime();
    double targetAngle, targetPosition,distance;
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1_3, grabPickup2, grabPickup2_3,scorePickup2,scorePickup3, preload,scorePickup1, Park,rPath1;

    private PathChain unload1,unload2;
    private PathChain ScorePreload , GrabFirst, GrabFromRack, ScoreFromRack, GrabSecond, ScoreFromRack2, Prep,GrabFromRack2;

    double Treshold = 0.82;
    int REP = 3000;
    double TargetRPM = 1830;

    int High_P = 60 , Low_P = 20;

    int delay_shoot = 200;

    ElapsedTime unload_timer = null;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setGlobalDeceleration(0.7)
                .build();


        GrabFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                .addPath(new BezierLine(pickup2_3Pose, aux_2))
                .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), aux_2.getHeading())
                .addParametricCallback(0.4,() -> {
                    motors.intakeReverse();
                    turret.goToPosition(-70);
                })
                .addPath(new BezierLine(aux_2, scorePose1))
                .setLinearHeadingInterpolation(aux_2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.9,() -> {
                    startPresiune();
                    motors.intakeOff();
                })
                .setGlobalDeceleration(0.7)
                .build();

        GrabSecond = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, pickup1_3Pose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1_3Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup1_3Pose, scorePose1))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.6,() -> {
                    motors.intakeReverse();
                    turret.goToPosition(-60);
                })
                .setGlobalDeceleration(0.7)
                .build();

        GrabFromRack = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, aux_2))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), aux_2.getHeading())
                .addPath(new BezierLine(aux_2, aux))
                .setLinearHeadingInterpolation(aux_2.getHeading(), aux.getHeading())
                .addPath(new BezierLine(aux, unloadPose))
                .setLinearHeadingInterpolation(aux.getHeading(), unloadPose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();

                })
                .addParametricCallback(1,() -> {
                    unload_timer.reset();
                    unload_timer.startTime();
                })
                .setGlobalDeceleration(0.7)
                .build();

        GrabFromRack2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, aux))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), aux.getHeading())
                .addPath(new BezierLine(aux, unloadPose2))
                .setLinearHeadingInterpolation(aux.getHeading(), unloadPose2.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();

                })
                .addParametricCallback(0.9,() -> {
                    unload_timer.reset();
                    unload_timer.startTime();
                })
                .setGlobalDeceleration(0.7)
                .build();

        Prep = follower.pathBuilder()
                .addPath(new BezierLine(aux_2, aux))
                .setLinearHeadingInterpolation(aux_2.getHeading(), aux.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .build();


        ScoreFromRack = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose, aux_2))
                .setLinearHeadingInterpolation(unloadPose.getHeading(), aux_2.getHeading())
                .addPath(new BezierLine(aux_2, scorePose1))
                .setLinearHeadingInterpolation(aux_2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.4,() -> {
                    startPresiune();
                    motors.intakeReverse();
                    turret.goToPosition(-60);
                })
                .setGlobalDeceleration(0.7)
                .build();

        ScoreFromRack2 = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose2, scorePose1))
                .setLinearHeadingInterpolation(unloadPose2.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.4,() -> {
                    startPresiune();
                    motors.intakeReverse();
                    turret.goToPosition(-50);
                })
                .setGlobalDeceleration(0.7)
                .build();


        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose.getHeading())
                .addParametricCallback(0,() -> {
                    turret.goToPosition(0);
                    motors.intakeOff();
                })
                .build();

    }

    void hold(double power){
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(-power);
        BR.setPower(-power);
    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(ScorePreload,true);
                setPathState(1);
                break;

            case 1:

                if(!follower.isBusy()) {
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
        autonomousPathUpdate();
        distance = limelight.getDistanceOD(
                follower.getPose().getX(),
                follower.getPose().getY(),
                target
        );
        //handleTurret();
        //servos.hoodMove((int)getHood(distance));
        //motors.setRampVelocityC((int)getRPM(distance));
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM: ",motors.getVelocity());
        telemetry.addData("Distance to target:",distance);
        telemetry.addData("Timer:", timer.milliseconds());
        telemetry.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        servos = Servos.getInstance(hardwareMap,telemetry);
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap,telemetry);
        limelight.setup();
        limelight.setPipeline(1);
        selectioner = Selectioner.getInstance(hardwareClass,telemetry);
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

        unload_timer = new ElapsedTime();

        turret = Turret.getInstance(hardwareMap, telemetry);
        turret.setup();
        turret.resetMotor();

        //servos.hoodMove(1);
        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        distance = limelight.getDistanceOD(follower.getPose().getX(), follower.getPose().getY(),target);
    }


    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    private void sleep(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void waitForRpm(){

        telemetry.clear();
        telemetry.addData("Velocity:",motors.getVelocity()*60/28);
        telemetry.update();

        if(delay.seconds()>1){
            if (Math.abs(motors.getRampError(2500)) < 75)
                shootBall();
            delay.reset();
        }
    }
    public void shootBall(){
        selectioner.unloadBallsSlow();
        sleep(300);
        shoots++;
    }

    public void startPresiune(){
        hold(0.18);
        motors.intakeOn();
        motors.setCoefsMan(High_P,0,0,1.2);
        motors.setRampVelocityC((int)(TargetRPM));
        //selectioner.resetServos();
    }

    public void stopPresiune(){
        shoot_short();
        motors.setCoefsMan(Low_P,0,0,1.2);
        motors.setRampVelocityC((int)(0.33 * TargetRPM));
        selectioner.resetServos();
        motors.intakeOff();
        hold(0);
    }

    void shoot_short(){
        HoodToPos(0);
        selectioner.topServoUp();
        sleep(delay_shoot);
        selectioner.leftServoUp();
        sleep(delay_shoot);
        selectioner.rightServoUp();
        HoodToPos(0);
    }

    void HoodToPos(double pos){
        hardwareClass.angle.setPosition(HardwareClass.hood_LOW - pos);
    }



    @Override
    public void stop() {
        //selectioner.stop();
    }


}