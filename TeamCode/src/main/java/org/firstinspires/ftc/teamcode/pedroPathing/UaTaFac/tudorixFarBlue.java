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


@Autonomous(name = "Tudorix Far Blue" , group = "Test")
public class tudorixFarBlue extends OpMode {
    int target = 0;
    private final Pose startPose = new Pose(133.6, 57.05, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(123.5, 57.3,Math.toRadians(240)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseError = new Pose(123.5,54,Math.toRadians(242.5)); // 116??
    private final Pose pickupScore1 = new Pose(108.42,43,Math.toRadians(270));
    private final Pose pickupScore1_3 = new Pose(108.42,14,Math.toRadians(270));
    private final Pose pickupPose2 = new Pose(131.09,11.75,Math.toRadians(270));
    private final Pose aux = new Pose(108.42,36,Math.toRadians(45));
    private final Pose parkPose = new Pose(131.4,40,Math.toRadians(270));
    private final ElapsedTime delay = new ElapsedTime();

    private Follower follower;
    double visionOffset;
    Servos servos = null;
    private TelemetryManager telemetryM;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass = null;

    Turret turret = null;

    double Treshold = 1.1;
    int REP = 3000;
    double TargetRPM = 4000;

    int High_P = 120 , Low_P = 20;

    int delay_shoot = 150;

    ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState, shoots = 0;
    private final int targetVelocity = 3000;
    int[] Pos = {0, 0, 0, 0, 0, 0};
    double targetAngle, targetPosition;

    private double power = 0.64;

    Pose BotPose;
    double x, y;
    private Path scorePreload;
    private PathChain park, Preload, Human, Pickup1;


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        Pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickupScore1))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickupScore1.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeOff();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickupScore1, pickupScore1_3))
                .setLinearHeadingInterpolation(pickupScore1.getHeading(), pickupScore1_3.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeOn();
                })
                .addPath(new BezierLine(pickupScore1_3, scorePose))
                .setLinearHeadingInterpolation(pickupScore1_3.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.3 , () -> {
                    motors.intakeReverse();
                })
                .setGlobalDeceleration(0.7)
                .build();

        Human = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, aux))
                .setLinearHeadingInterpolation(scorePose.getHeading(), aux.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeOff();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(aux, pickupPose2))
                .setLinearHeadingInterpolation(aux.getHeading(), pickupPose2.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeOn();
                })
                .addPath(new BezierLine(pickupPose2, aux))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), aux.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeReverse();
                })
                .addPath(new BezierLine(aux, pickupPose2))
                .setLinearHeadingInterpolation(aux.getHeading(), pickupPose2.getHeading())
                .addParametricCallback(0 , () -> {
                    motors.intakeOn();
                })
                .addPath(new BezierLine(pickupPose2, scorePose))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.5 , () -> {
                    motors.intakeReverse();
                })
                .setGlobalDeceleration(0.7)
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseError, parkPose))
                .setLinearHeadingInterpolation(scorePoseError.getHeading(), parkPose.getHeading())
                .setGlobalDeceleration(0.7)
                .build();

    }


    /* You could check for
               - Follower State: "if(!follower.isBusy()) {}"
               - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
               - Robot Position: "if(follower.getPose().getX() > 36) {}"
               */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        updateTurretFusion();
                        if(motors.getVelocity() >= (TargetRPM - 1000) * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(Pickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        updateTurretFusion();
                        if(motors.getVelocity() >= (TargetRPM - 1000) * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(Human);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        updateTurretFusion();
                        if(motors.getVelocity() >= (TargetRPM - 1000) * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(6);
                }
                break;
            case 6:
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


        telemetry.addData("path state", pathState);
        telemetry.addData("Ball1: ", Pos[1]);
        telemetry.addData("Ball2: ", Pos[2]);
        telemetry.addData("Ball3: ", Pos[3]);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Power: ",power);
        telemetryM.addData("RPM: ",motors.getVelocity());
        telemetry.addData("Shoots: ",shoots);
        telemetryM.addData("Shoots:", shoots);
        telemetry.update();
        updatePosition();
        updateTurretFusion();
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
        selectioner =Selectioner.getInstance(hardwareClass,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = Turret.getInstance(hardwareMap,telemetry);

        selectioner.hoodMove(1);
        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        turret = Turret.getInstance(hardwareMap, telemetry);
        turret.setup();
        turret.resetMotor();
        resetTurret();
    }

    public void startPresiune(){
        motors.intakeOn();
        turret.powerOn();
        motors.setCoefsMan(High_P,0,0,1.2);
        motors.setRampVelocityC((int)(TargetRPM));
    }

    public void stopPresiune(){
        PoseStorage.autoPose = follower.getPose();
        shoot_long();
        motors.setCoefsMan(Low_P,0,0,1.2);
        motors.setRampVelocityC((int)(1000));
        selectioner.resetServos();
        motors.intakeOff();
    }

    void shoot_long(){
        //Shoot First
        HoodToPos(0);
        selectioner.rightServoUp();
        sleep(delay_shoot);
        //Shoot Second
        //HoodToPos(0.01);
        selectioner.leftServoUp();
        sleep(delay_shoot);
        //Shoot Third
        //HoodToPos(0.02);
        selectioner.topServoUp();
        HoodToPos(0);
    }

    void HoodToPos(double pos){
        hardwareClass.angle.setPosition(HardwareClass.hood_LOW - pos);
    }





    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.setStartingPose(startPose);
        turret.powerOn();
        setPathState(0);
    }
    private void sleep(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }


    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    void updateTurretFusion() {     //Odometrie instant + corectie limelight
        double dx=0,dy=0;
        updatePosition();
        limelight.setPipeline(0);
        dx = HardwareClass.autoBlueScorePoseX - x;
        dy = HardwareClass.autoBlueScorePoseY - y;

        double goalAngle = Math.atan2(dy, dx);
        double thetaR = BotPose.getHeading();

        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));

        targetPosition = convertToNewRange(
                targetAngle,
                -2*Math.PI/3, 2*Math.PI/3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        if (limelight.checkResults()) {
            double tx = limelight.getXPos();
            if(Math.abs(tx)>1.2)
                visionOffset -= tx * 0.20;
        } else {
            visionOffset *= 0.8;
        }
        targetPosition += visionOffset;

        targetPosition = Math.min(Math.max(targetPosition,HardwareClass.turret_min),HardwareClass.turret_max);
        turret.goToPosition(targetPosition);
    }

    private void waitForRpm(){
        if(delay.seconds()>1.3){
            if (Math.abs(motors.getVelocity() * 60 / 28 - targetVelocity) < 100)
                shootBall();
            delay.reset();
        }
    }

    void updatePosition(){
        BotPose = follower.getPose();
        x = BotPose.getX();
        y = BotPose.getY();
    }

    void resetTurret() {
        target = -1;
        turret.goToPosition(1400);
        sleep(500);
        turret.resetMotor();
        turret.goToPosition(turret.getPosition());
        target = 1;
        turret.powerOFF();
    }

    public void shootBall(){
        selectioner.unloadBallsSlow();
    }
    @Override
    public void stop() {
        //selectioner.stop();
    }


}