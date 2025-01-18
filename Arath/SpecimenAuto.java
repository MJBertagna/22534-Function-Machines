import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous (name = "SpecimenAuto")
public class SpecimenAuto extends OpMode {

    private final Pose startPose = new Pose(9, 60.08823529411764, Math.toRadians(0));

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private PathChain preloadHang, backUp, positionSample1, pushSample1, positionSample2, pushSample2,  positionSample3, pushSample3;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.529, 60.618, Point.CARTESIAN),
                                new Point(39.706, 70.412, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(39.706, 70.412, Point.CARTESIAN),
                                new Point(1.588, 1.588, Point.CARTESIAN),
                                new Point(60.618, 67.500, Point.CARTESIAN),
                                new Point(142.676, 18.000, Point.CARTESIAN),
                                new Point(63.794, 10.059, Point.CARTESIAN),
                                new Point(18.265, 22.235, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(18.265, 22.235, Point.CARTESIAN),
                                new Point(104.824, 46.853, Point.CARTESIAN),
                                new Point(103.765, 0.794, Point.CARTESIAN),
                                new Point(42.353, 8.206, Point.CARTESIAN),
                                new Point(19.059, 16.676, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(19.059, 16.676, Point.CARTESIAN),
                                new Point(43.676, 14.029, Point.CARTESIAN),
                                new Point(12.706, 15.353, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(12.706, 15.353, Point.CARTESIAN),
                                new Point(16.676, 69.618, Point.CARTESIAN),
                                new Point(39.706, 70.676, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(39.706, 70.676, Point.CARTESIAN),
                                new Point(12.706, 60.882, Point.CARTESIAN),
                                new Point(52.147, 10.853, Point.CARTESIAN),
                                new Point(12.706, 15.618, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(12.706, 15.618, Point.CARTESIAN),
                                new Point(16.676, 69.618, Point.CARTESIAN),
                                new Point(39.706, 70.676, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));


        //Line 1
        preloadHang = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(9.529, 60.618, Point.CARTESIAN),
                        new Point(39.706, 70.412, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //Line 2
        backUp = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(39.706, 70.412, Point.CARTESIAN),
                        new Point(1.588, 1.588, Point.CARTESIAN),
                        new Point(60.618, 67.500, Point.CARTESIAN),
                        new Point(142.676, 18.000, Point.CARTESIAN),
                        new Point(63.794, 10.059, Point.CARTESIAN),
                        new Point(18.265, 22.235, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
//        //Line 3
        positionSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(18.265, 22.235, Point.CARTESIAN),
                        new Point(104.824, 46.853, Point.CARTESIAN),
                        new Point(103.765, 0.794, Point.CARTESIAN),
                        new Point(42.353, 8.206, Point.CARTESIAN),
                        new Point(19.059, 16.676, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //Line 4
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(19.059, 16.676, Point.CARTESIAN),
                        new Point(43.676, 14.029, Point.CARTESIAN),
                        new Point(12.706, 15.353, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //Line 5
        positionSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(12.706, 15.353, Point.CARTESIAN),
                        new Point(16.676, 69.618, Point.CARTESIAN),
                        new Point(39.706, 70.676, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //Line 6
        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(39.706, 70.676, Point.CARTESIAN),
                        new Point(12.706, 60.882, Point.CARTESIAN),
                        new Point(52.147, 10.853, Point.CARTESIAN),
                        new Point(12.706, 15.618, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        //Line 7
        positionSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(12.706, 15.618, Point.CARTESIAN),
                        new Point(16.676, 69.618, Point.CARTESIAN),
                        new Point(39.706, 70.676, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void init() {
        HardwareMap hMap = hardwareMap;

        BotConstants.clawSubRotate = hardwareMap.get(Servo.class, "clawSubRotate");
        BotConstants.clawSub = hardwareMap.get(Servo.class, "clawSub");
        BotConstants.clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        BotConstants.clawVert = hardwareMap.get(Servo.class, "clawVert");
        BotConstants.claw = hardwareMap.get(Servo.class, "claw");

        //Elevator Variables
        BotConstants.leftElevator = hardwareMap.get(DcMotorEx.class, "leftSlide");
        BotConstants.rightElevator = hardwareMap.get(DcMotorEx.class, "rightSlide");
        BotConstants.horizontalElevator = hardwareMap.get(DcMotorEx.class, "hSlide");

        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        buildPaths();

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        ClawSubsystem.checkTimers();
        SlideSubsystem.findElevatorPower();
        SlideSubsystem.setElevatorPower(BotConstants.powerLeft, BotConstants.powerRight, BotConstants.powerH);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                ClawSubsystem.setClawPosition(3);
                follower.followPath(preloadHang);
                setPathState(1);

                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(backUp,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(positionSample1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSample1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(positionSample2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSample2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(positionSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(pushSample3,true);
                    setPathState(8);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
