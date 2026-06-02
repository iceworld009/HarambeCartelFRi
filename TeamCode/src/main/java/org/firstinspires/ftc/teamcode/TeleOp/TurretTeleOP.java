//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.HardwareClass;
//import org.firstinspires.ftc.teamcode.Threads.Holonomic;
//import org.firstinspires.ftc.teamcode.Threads.Servos;
//import org.opencv.core.Mat;
//
//public class TurretTeleOP extends LinearOpMode {
//
//    HardwareClass hardwareClass = null;
//    private Follower follower;
//    String Style = "Drive";
//
//    int ExtendoPosition = 0;
//
//    enum Mode {
//        DRIVER_CONTROL,
//        AUTOMATIC_CONTROL
//    }
//
//    Mode currentMode = Mode.DRIVER_CONTROL;
//
//    // The coordinates we want the bot to automatically go to when we press the A button
//    Vector2d targetAVector = new Vector2d(-60, -56);
//    // The heading we want the bot to end on for targetA
//    double targetAHeading = Math.toRadians(45);
//
//    // The location we want the bot to automatically go to when we press the B button
//    Vector2d targetBVector = new Vector2d(-15, 25);
//
//    // The angle we want to align to when we press Y
//    double targetAngle = Math.toRadians(45);
//
//    Pose2d pose = null;
//
//    int automatic = -1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hardwareClass = HardwareClass.getInstance(hardwareMap);
//
//        turret = Turret.getInstance(hardwareMap, telemetry);
//
////        Constants.setConstants(FConstants.class, LConstants.class);
////        follower = new Follower(hardwareMap);
////        follower.setStartingPose(follower.getPose());
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive() && !isStopRequested()) {
//            switch (Style){
//                case "Drive" : {
//
//                    /**
//                     * Pentru Stanga
//                     */
//
//                    if(gamepad1.x){
//                        double x = follower.getPose().getX();
//                        double y = follower.getPose().getY();
//                        double goalAngle = Math.atan(x/y);// RADIANS
//                        double thetaR = follower.getPose().getHeading(); // RADIANS
//                        double targetAngle = goalAngle - thetaR;
//
//                        double targerPosition = convertToNewRange(targetAngle , -1.5707 ,1.5707 , -500 , 500 );
//
//                        turret.goToPosition(targerPosition);
//                    }
//                    break;
//                }
//                case "Intake" :{
//                    break;
//                }
//                default: break;
//            }
//        }
//    }
//
//    public int convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
//        return (int)(newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin));
//    }
//
//
//    private void delay(int delay){
//        try {
//            Thread.sleep(delay);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//    }
//}