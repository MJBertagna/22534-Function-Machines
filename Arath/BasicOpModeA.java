import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "crashOutA")
public class BasicOpModeA extends OpMode {

    @Override
    public void init() {

        BotConstants.dashboard = FtcDashboard.getInstance();
        telemetry = BotConstants.dashboard.getTelemetry();

        //Mecanum Motor Variables
        BotConstants.frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        BotConstants.frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        BotConstants.backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        BotConstants.backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        //Servo Variables
        BotConstants.clawSubRotate = hardwareMap.get(Servo.class, "clawSubRotate");
        BotConstants.clawSub = hardwareMap.get(Servo.class, "clawSub");
        BotConstants.clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        BotConstants.clawVert = hardwareMap.get(Servo.class, "clawVert");
        BotConstants.claw = hardwareMap.get(Servo.class, "claw");

        //Elevator Variables
        BotConstants.leftElevator = hardwareMap.get(DcMotorEx.class, "leftSlide");
        BotConstants.rightElevator = hardwareMap.get(DcMotorEx.class, "rightSlide");
        BotConstants.horizontalElevator = hardwareMap.get(DcMotorEx.class, "hSlide");

        //Reversed Motors
        BotConstants.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BotConstants.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BotConstants.rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        //Reversed servos
        BotConstants.claw.setDirection(Servo.Direction.REVERSE);

        //Set Mode for Elevator Variables
        BotConstants.leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BotConstants.rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BotConstants.leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BotConstants.rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BotConstants.horizontalElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BotConstants.horizontalElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        BotConstants.imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        BotConstants.imu.initialize(parameters);


        //Initializes initial position of robot will
        //CHANGE TO HOLD SPECIMEN
        BotConstants.claw.setPosition(BotConstants.clawOpenPos);
        BotConstants.clawRotate.setPosition(BotConstants.clawRotatePickUpPos);
        BotConstants.clawVert.setPosition(BotConstants.clawVertPickUpPos);
        try{
            long delay = 500;
            sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        BotConstants.clawSubRotate.setPosition(BotConstants.clawSubRotateUp);
        BotConstants.clawSub.setPosition(BotConstants.clawSubOpenPos);

        // <<<<<<<<<<< INITIALIZES VARIABLES TO RESET >>>>>>>>>>
        BotConstants.speedMultiplier = 1;
        SlideSubsystem.setCounter(-1);
        SlideSubsystem.setCounterH(-2);
        BotConstants.retracting = false;
        BotConstants.raising = false;

        // For claw open/close
        BotConstants.clawClosing = false;
        BotConstants.clawClosingBucket = false;
        BotConstants.clawOpeningPickup = false;

        // For raising/lowering claw
        BotConstants.clawLowering = false;
        BotConstants.clawRaisingBucket = false;
        BotConstants.clawRaisingHang = false;

        // For rotating claw
        BotConstants.clawRotatingPickUp = false;
        BotConstants.clawRotatingSpecimen = false;

        // For clawSub open/close
        BotConstants.clawSubOpening = false;
        BotConstants.clawSubClosing = false;

        // For clawSub rotating
        BotConstants.clawSubRotatingUp = false;
        BotConstants.clawSubRotatingDown = false;

        BotConstants.timesSquarePressed = 0;
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) {
            BotConstants.speedMultiplier = 1.00;
        }
        if(gamepad1.dpad_right) {
            BotConstants.speedMultiplier = 0.60;
        }
        if(gamepad1.dpad_down) {
            BotConstants.speedMultiplier = 0.35;
        }

        if(gamepad1.options){
            BotConstants.imu.resetYaw();
        }

        ////////////////////////////////////////////////////////////////////////////////////////////
        //Movement Code
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        //Accounts for stick drift on the control
        if(-BotConstants.leftStickDeadZone <= leftStickX && leftStickX <= BotConstants.leftStickDeadZone)
            leftStickX = 0;
        if(-BotConstants.leftStickDeadZone <= leftStickY && leftStickY <= BotConstants.leftStickDeadZone)
            leftStickY = 0;
        if(-BotConstants.rightStickDeadZone <= rightStickX && rightStickX <= BotConstants.rightStickDeadZone)
            rightStickX = 0;

        double x = leftStickX;
        double y = leftStickY;
        double yaw = -rightStickX;

        double botHeading = BotConstants.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        //Rotates the movement direction to counter the rotation of the robot
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        yaw = Math.pow(yaw,3);

        //Calculates the power for each wheel and makes sure that everything is proportional
        double frontLeftPower = (y - x + yaw);
        double frontRightPower = (y + x - yaw);
        double backLeftPower = (y + x + yaw);
        double backRightPower = (y - x - yaw);

        double[] appliedPowers = scaleDrivePower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        //Sets the power to each wheel
        BotConstants.frontLeftMotor.setPower(appliedPowers[0] * BotConstants.speedMultiplier);
        BotConstants.frontRightMotor.setPower(appliedPowers[1] * BotConstants.speedMultiplier);
        BotConstants.backLeftMotor.setPower(appliedPowers[2] * BotConstants.speedMultiplier);
        BotConstants.backRightMotor.setPower(appliedPowers[3] * BotConstants.speedMultiplier);



        ////////////////////////////////////////////////////////////////////////////////////////////

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CLAW CODE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        //Drops sample into bucket and goes into pick up position
        if(gamepad2.circle) {
            ClawSubsystem.setClawPosition(0);
            BotConstants.timesSquarePressed = 0;
        }

//        //Moves the top claw for specimen
//        if(gamepad2.square) {
//            if (!BotConstants.isSquarePressed) {
//                BotConstants.timesSquarePressed++;
//                if(BotConstants.timesSquarePressed == 1) {
//                    ClawSubsystem.setClawPosition(2);
//                }else if(BotConstants.timesSquarePressed == 2) {
//                    ClawSubsystem.setClawPosition(3);
//                }else if(BotConstants.timesSquarePressed == 3) {
//                    ClawSubsystem.setClawPosition(2);
//                    BotConstants.timesSquarePressed = 1;
//                }
//
//            }
//            BotConstants.isSquarePressed = true;
//        }else{
//            BotConstants.isSquarePressed = false;
//        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<< ELEVATOR CODE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        //Changes the reference of the vertical slides (RAISES)
        if(gamepad2.right_bumper) {
            if (!BotConstants.isRightBumperPressed) {
                //grabs the sample first and then
                SlideSubsystem.setCounter(1);
            }
            BotConstants.isRightBumperPressed = true;
        }else{
            BotConstants.isRightBumperPressed = false;
        }

        //Changes the reference of the vertical slide (LOWERS)
        if(gamepad2.left_bumper) {
            if (!BotConstants.isLeftBumperPressed) {
                // do thing
                SlideSubsystem.setCounter(-1);
            }
            BotConstants.isLeftBumperPressed = true;
        }else{
            BotConstants.isLeftBumperPressed = false;
        }

        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< HORIZONTAL SLIDES >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        //Allows manual control of the HORIZONTAL slides
        //Moves H slide forward with claw sub rotated up
        if(gamepad2.triangle) {
            if (!BotConstants.isTrianglePressed) {
                //grabs the sample first and then
                SlideSubsystem.setCounterH(1);
                BotConstants.clawSubRotate.setPosition(BotConstants.clawSubRotateDown);

                BotConstants.clawSubRotatingDown = true;
                BotConstants.delayTimer.reset();
            }
            BotConstants.isTrianglePressed = true;
        }else{
            BotConstants.isTrianglePressed = false;
        }

        //Moves H slide back with claw sub rotated up
        if(gamepad2.cross) {
            if (!BotConstants.isCrossPressed) {
                //grabs the sample first and then
                BotConstants.clawVert.setPosition(BotConstants.clawVertPickUpPos - 0.03);
                BotConstants.clawSub.setPosition(BotConstants.clawSubClosedPos);

                BotConstants.clawSubClosing = true;
                BotConstants.delayTimer.reset();
            }
            BotConstants.isCrossPressed = true;
        }else{
            BotConstants.isCrossPressed = false;
        }

        if(gamepad2.dpad_up) {
            ClawSubsystem.setClawPosition(1);
        }

        if(gamepad2.dpad_down) {
            BotConstants.referenceH = 0;
        }

        //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< LIMITS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.rightElevator.getCurrentPosition() <= 10) {
            SlideSubsystem.setElevatorPower(0, 0, BotConstants.powerH);
        }


        //Sets Power
        SlideSubsystem.findElevatorPower();
        SlideSubsystem.setElevatorPower(BotConstants.powerLeft, BotConstants.powerRight, BotConstants.powerH);
        ClawSubsystem.checkTimers();

        ////////////////////////////////////////////////////////////////////////////////////////////
        //Telemetry Data

        //Adds data for the driver station to show
        telemetry.addData("Counter", BotConstants.counter);
        telemetry.addData("Left Elevator Pos", BotConstants.leftElevator.getCurrentPosition());
        telemetry.addData("Right Elevator Pos", BotConstants.rightElevator.getCurrentPosition());
        telemetry.addData("H Elev Pos", BotConstants.horizontalElevator.getCurrentPosition());
        telemetry.addData("KpH", BotConstants.KpH);
        telemetry.addData("KdH", BotConstants.KdH);
        telemetry.addData("Error", BotConstants.lastError);
        telemetry.addData("Error H", BotConstants.lastErrorH);

        telemetry.update();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    public double[] scaleDrivePower(double fLeftPower, double fRightPower, double bLeftPower, double bRightPower) {

        double max = Math.max(Math.abs(fLeftPower), Math.max(Math.abs(fRightPower), Math.max(Math.abs(bLeftPower), Math.abs(bRightPower))));

        if(max > 1) {
            fLeftPower /= max;
            fRightPower /= max;
            bLeftPower /= max;
            bRightPower /= max;
        }

        double[] motorPowers = {fLeftPower, fRightPower, bLeftPower, bRightPower};
        return motorPowers;
    }



}