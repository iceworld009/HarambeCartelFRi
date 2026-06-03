package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

@TeleOp(name="TeleOp Solo", group = "Solo")
public class TeleOpSolo extends LinearOpMode {
    Servos servos = null;
    HardwareClass hardwareClass = null;
    Holonomic holonomic = null;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    Turret turret = null;
    Follower follower;
    boolean rampUp = false;
    double targetVelocity;
    int target = 10;
    int greenPos = -1;
    double distance = 0, error;
    boolean turretOn = false;
    double adjust, visionOffset;
    double targetPosition, targetAngle;
    int greenOffset = 0;
    double x, y;
    Pose BotPose;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime check = new ElapsedTime();
    private TelemetryManager telemetryM;
    Thread updateTurret = null;
    @Override
    public void runOpMode()  {

        // Get Instances
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(PoseStorage.autoPose.getX()-72, PoseStorage.autoPose.getY()-72,PoseStorage.autoPose.getPose().getHeading()));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        servos = Servos.getInstance(hardwareMap , telemetry);
        motors = Motors.getInstance(hardwareMap);
        turret = Turret.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        holonomic = Holonomic.getInstance(hardwareMap , gamepad1, gamepad2);
        limelight = Limelight.getInstance(hardwareMap,telemetry);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        selectioner.checkColors();


        //Inital setup 1/2
        limelight.setup();
        limelight.start();
        telemetry.addLine("Ready");
        telemetry.addLine("Press START");
        telemetry.clear();
        telemetry.update();


        waitForStart();


        //Inital setup 2/2
        selectioner.resetServos();
        if(!turret.getStatus()){
            turret.setup();
        }
        motors.setRampCoefs();
        turret.resetMotor();
        if(!holonomic.getStatus()){
            holonomic.start();
        }
        resetTurret();
        target = 10;
        selectioner.hoodMove(2);
        startUpdate();
        while(opModeIsActive()) {


            if(updateTurret == null)
                updatePosition();
            distance = limelight.getDistanceOD(x, y,target);
            error = motors.getRampError(targetVelocity);
            targetVelocity = getRPM(distance);

            if(check.milliseconds()>150){
                updateTelemetry();
                check.reset();
            }

            if(rampUp)
            {
                motors.setRampVelocityC((int)targetVelocity);
            }
            else if(motors.getVelocity()<700 && turretOn && !rampUp && selectioner.ballsfull) {
                motors.setRampVelocityC(700);
            }


            if(error>-40) {
                selectioner.shootOnAT(greenOffset);
                motors.rampStop();
                rampUp = false;
            }

            if(gamepad1.right_stick_button){
                target=0;
                limelight.setPipeline(4);
                limelight.stop();
            }
            else
            if(gamepad1.left_stick_button){
                target=1;
                limelight.setPipeline(0);
                limelight.stop();
            }
            else if(gamepad1.ps){
                target = 10;
                greenPos = -1;
                limelight.pipeline = -1;
                limelight.start();
            }

            if (gamepad1.right_trigger > 0.1 ) {
                motors.intakeOn();
            }
            else if(gamepad1.left_trigger > 0.1) {
                motors.intakeReverse();
                time.reset();
            }
            else
                motors.intakeOff();

            if (gamepad1.right_bumper) {
                rampUp = true;
                turretOn = true;
                greenOffset = -1;
                motors.setRampVelocityC((int)targetVelocity);
            }

            if (gamepad1.dpad_left) {
                rampUp = true;
                turretOn = true;
                greenOffset = 0;
            }

            if (gamepad1.dpad_down) {
                rampUp = true;
                turretOn = true;
                greenOffset = 2;
            }

            if (gamepad1.dpad_right) {
                rampUp = true;
                turretOn = true;
                greenOffset = 1;
            }

            if (gamepad1.left_bumper) {
                motors.rampStop();
                rampUp = false;
                turretOn = false;
            }

            if(gamepad1.a && rampUp){
                selectioner.shootOnAT(greenOffset);
            }

            if(gamepad1.y) {
                selectioner.unloadBalls();
            }

            if(gamepad1.dpad_up){
                resetTurret();
            }

            if(gamepad1.x)
                follower.setPose(PoseStorage.bluePose);


            if(gamepad1.b)
                follower.setPose(PoseStorage.redPose);
        }
    }

    int getRPM(double d) {
        double r = -0.0023 * Math.pow(d, 2) + 2.77 * d + 2150;
        return (int) Math.max(1000, Math.min(r, 4000));
    }

    double getHood(double d) {
        if (d < 130) {
            return (d - 100) * (7.0 / 30.0);
        }
        else if (d < 170) {
            return 5 - (d - 130) * (2.5 / 40.0);
        }
        else if (d < 220) {
            return 3 + (d - 170) * (1.3 / 50.0);
        }
        else {
            return 4.6 + (d - 220) * (2.6 / 130.0);
        }
    }



    public void startUpdate() {
        boolean running = true;
        if (updateTurret == null || !updateTurret.isAlive()) {
            updateTurret = new Thread(() -> {
                while (running) {
                    updatePosition();
                    if(target == 0 || target == 1 || target == 10) {
                        updateTurretFusion();
                    }
                }
            });
            updateTurret.start();
        }
    }
    void updateTurretFusion() {

        double dx = 0, dy = 0;

        if(target == 1) {
            dx = HardwareClass.blueScoreX - x;
            dy = HardwareClass.blueScoreY - y;
        }
        else if(target == 0) {
            dx = HardwareClass.redScoreX - x;
            dy = HardwareClass.redScoreY - y;
        }
        else if(target == 10) {
            if(greenPos == -1) {
                greenPos = limelight.checkApriltagResults();
            }

            if(greenPos > 0 && greenPos < 4) {
                target = 0;
                limelight.setPipeline(4);
                selectioner.setTagPos(greenPos);
            }

            dx = HardwareClass.tagPosX - x;
            dy = HardwareClass.tagPosY - y;
        }

        double goalAngle = Math.atan2(dy, dx);
        double robotHeading = BotPose.getHeading();

        double relativeAngle = goalAngle - robotHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        double finalPosition = convertToNewRange(
                relativeAngle,
                -2 * Math.PI / 3, 2 * Math.PI / 3,
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        finalPosition = Math.max(
                HardwareClass.turret_min+10,
                Math.min(HardwareClass.turret_max-10, finalPosition)
        );
        targetPosition = finalPosition;
        turret.goToPosition(finalPosition);
    }


    void updateTurretPredictive() {
        Pose pose = follower.getPose();

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        double speed = follower.getVelocity().getMagnitude();

        double vx = speed * Math.cos(heading);
        double vy = speed * Math.sin(heading);

        double goalX = HardwareClass.redScoreX;     // red
        double goalY = HardwareClass.redScoreY;

        double dx = goalX - x;
        double dy = goalY - y;

        double distance = Math.hypot(dx, dy);

        double projectileSpeed = 550.0;
        double time = distance / projectileSpeed;

        double predictedX = x + vx * time;
        double predictedY = y + vy * time;

        double pdx = goalX - predictedX;
        double pdy = goalY - predictedY;

        double goalAngle = Math.atan2(pdy, pdx);

        double omega = follower.getPose().getHeading();
        double predictedHeading = heading + omega * time;

        double relativeAngle = goalAngle - predictedHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        double basePosition = convertToNewRange(
                relativeAngle,
                -Math.toRadians(115), Math.toRadians(115),
                HardwareClass.turret_min, HardwareClass.turret_max
        );

        double finalPos = basePosition;
        finalPos = Math.max(HardwareClass.turret_min, Math.min(HardwareClass.turret_max, finalPos));

        turret.goToPosition(finalPos);
    }

    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    void resetTurret() {
        target = -1;
        turret.goToPosition(1000);
        sleep(700);
        turret.goToPosition((HardwareClass.turret_min + HardwareClass.turret_max)/2.0);
        turret.resetMotor();
        target = -1;
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
        selectioner.checkColors();
        telemetry.clearAll();
        telemetry.addData("x:" , x);
        telemetry.addData("y:" , y);
        telemetry.addData("Distance: " , distance);
        telemetry.addData("Velocity:",motors.getVelocity());
        telemetry.addData("Turret error:",getTurretError());
        telemetry.addData("Velocity error:",error);
        if(target == 0){
            telemetry.addLine("Target: RED");
        }
        else if(target == 1) {
            telemetry.addLine("Target: BLUE");
        }
        else if(target == 10){
            telemetry.addLine("Target: AprilTag");
        }
        telemetry.addData("ResultTop:",selectioner.resultTop);
        telemetry.addData("ResultLeft:",selectioner.resultBotL);
        telemetry.addData("ResultRight:",selectioner.resultBotR);
        telemetry.addData("GreenPos:",this.greenPos);
        telemetry.update();
    }
}