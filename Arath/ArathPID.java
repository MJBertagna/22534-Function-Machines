package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//0 servo is arm
//1 servo is bottom wheel
//2 servo is top wheel


@TeleOp(name = "crashOutA")
public class BasicOpModeA extends OpMode {

    public double integralSum = 0;
    public double Kp = 0.013;
    public double Ki = 0;
    public double Kd = 0;

    //Counter on 0 is full bottom - 0
    //Counter on 1 is bottom specimen bar - 1100 counts
    //counter on 2 is top specimen bar/bottom bucket - 2500 counts
    //counter on 3 is top bucket
    //counter on 4 is bottom hanging bar
    //counter on 5 is top hanging bar
    int reference = 0;
    public int counter = 0;
    public boolean isRightBumperPressed = false;
    public boolean isLeftBumperPressed = false;
    public boolean isTrianglePressed = false;
    public boolean isXPressed = false;
    //public double powerLeft = 0;
    public double powerRight = 0;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;

    public Servo armServo = null;
    public CRServo bottomWheel = null;
    public CRServo topWheel = null;

    public DcMotorEx leftElevator = null;
    public DcMotorEx rightElevator = null;

    //public Servo clawServo = null;
    //public Servo armServo = null;

    //DeadZone variables for stick drift
    public final double leftStickDeadZone = 0.15;
    public final double rightStickDeadZone = 0.15;
    public double speedMultiplier = 0.8;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "flD");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frD");
        backLeftMotor = hardwareMap.get(DcMotor.class, "blD");
        backRightMotor = hardwareMap.get(DcMotor.class, "brD");

        armServo = hardwareMap.get(Servo.class, "arm");
        bottomWheel = hardwareMap.get(CRServo.class, "bottomWheel");
        topWheel = hardwareMap.get(CRServo.class, "topWheel");


        leftElevator = hardwareMap.get(DcMotorEx.class, "lS");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rS");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up)
            speedMultiplier = 0.8;
        if(gamepad1.dpad_left)
            speedMultiplier = 0.5;
        if(gamepad1.dpad_down)
            speedMultiplier = 0.2;

        if(gamepad1.right_trigger > 0)
            speedMultiplier += 0.001;
        if(gamepad1.left_trigger > 0)
            speedMultiplier -= 0.001;

        if(speedMultiplier < 0.2)
            speedMultiplier = 0.2;
        if(speedMultiplier > 1)
            speedMultiplier = 1;

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

//        //Accounts for stick drift on the control
        if(-leftStickDeadZone <= leftStickX && leftStickX <= leftStickDeadZone)
            leftStickX = 0;
        if(-leftStickDeadZone <= leftStickY && leftStickY <= leftStickDeadZone)
            leftStickY = 0;
        if(-rightStickDeadZone <= rightStickX && rightStickX <= rightStickDeadZone)
            rightStickX = 0;

        double x = leftStickX * 1.1; // Multiplier to counteract imperfect strafing
        double y = leftStickY;
        double yaw = rightStickX;

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        double frontLeftPower = (y - x - yaw) / denominator;
        double frontRightPower = (y + x + yaw) / denominator;
        double backLeftPower = (y + x - yaw) / denominator;
        double backRightPower = (y - x + yaw) / denominator;

        frontRightMotor.setPower(frontRightPower * speedMultiplier);
        frontLeftMotor.setPower(frontLeftPower * speedMultiplier);
        backLeftMotor.setPower(backLeftPower * speedMultiplier);
        backRightMotor.setPower(backRightPower * speedMultiplier);


        if(gamepad1.y)
            armServo.setPosition(.5);

        topWheel.setPower(-gamepad1.right_trigger + gamepad1.left_trigger);//Reversed Direction
        bottomWheel.setPower(gamepad1.right_trigger - gamepad1.left_trigger);


        //raises the elevator
        if(gamepad1.right_bumper) {
            if (!isRightBumperPressed) {
                // do thing
                setCounter(counter, 1);
            }
            isRightBumperPressed = true;
        }else{
            isRightBumperPressed = false;
        }

        //lowers the elevator
        if(gamepad1.left_bumper) {
            if (!isLeftBumperPressed) {
                // do thing
                setCounter(counter, -1);
            }
            isLeftBumperPressed = true;
        }else{
            isLeftBumperPressed = false;
        }


        //Changes the PID constants
        if(gamepad1.dpad_right){
            if(gamepad1.square)
                Ki-=0.001;
            if(gamepad1.circle)
                Kd-=0.001;
        }else{
            if(gamepad1.square)
                Ki+=0.001;
            if(gamepad1.circle)
                Kd+=0.0001;
        }

        if(Kp < 0)
            Kp = 0;
        if(Ki < 0)
            Ki = 0;
        if(Kd < 0)
            Kd = 0;

        findElevatorPower(counter);
        if(rightElevator.getCurrentPosition() > reference - 100 && rightElevator.getCurrentPosition() < reference + 50){
            setElevatorPower(powerRight * 0.2, powerRight * 0.2);
        }else{
            setElevatorPower(powerRight, powerRight);
        }


        telemetry.addData("Proportional", Kp);
        telemetry.addData("Integral", Ki);
        telemetry.addData("Derivative", Kd);
        telemetry.addData("Counter", counter);
        telemetry.addData("Speed Multiplier", speedMultiplier);
        telemetry.addData("Left Elevator Pos", leftElevator.getCurrentPosition());
        telemetry.addData("Right Elevator Pos", rightElevator.getCurrentPosition());
        telemetry.addData("Left Stick DeadZone", leftStickDeadZone);

        telemetry.update();

    }

    //Positional PID
    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public void findElevatorPower(int count){
        if(count == 0){
            //powerLeft = PIDControl(reference, leftElevator.getCurrentPosition());
            powerRight = PIDControl(reference, rightElevator.getCurrentPosition());
        }else if(count == 1){
            //powerLeft = PIDControl(reference, leftElevator.getCurrentPosition());
            powerRight = PIDControl(reference, rightElevator.getCurrentPosition());
        }else if(count == 2){
            //powerLeft = PIDControl(reference, leftElevator.getCurrentPosition());
            powerRight = PIDControl(reference, rightElevator.getCurrentPosition());
        }else if(count == 3){
            //powerLeft = PIDControl(reference, leftElevator.getCurrentPosition());
            powerRight = PIDControl(reference, rightElevator.getCurrentPosition());
        }//else if(count == 4){
//            powerLeft = PIDControl(3100, leftElevator.getCurrentPosition());
//            powerRight = PIDControl(3100, rightElevator.getCurrentPosition());
//        }else if(count == 5){
//            powerLeft = PIDControl(3100, leftElevator.getCurrentPosition());
//            powerRight = PIDControl(3100, rightElevator.getCurrentPosition());
//        }
    }

    //Sets the elevators power using the values from the PID
    public void setElevatorPower(double powerLeft, double powerRight){
        leftElevator.setPower(powerLeft);
        rightElevator.setPower(powerRight);
    }

    //Sets the counter and makes sure it doesn't go over or under the limit
    public void setCounter(int tempCounter, int sign){
        if(sign == 1){
            if(tempCounter == 0){
                counter = 1;
                reference = 1100;
            }else if(tempCounter == 1){
                counter = 2;
                reference = 2500;
            }else if(tempCounter == 2){
                counter = 3;
                reference = 2900;
            } else if (tempCounter == 3) {
                counter = 3;
                reference = 2900;
            }
        }else if(sign == -1){
            if(tempCounter == 0){
                counter = 0;
                reference = 0;
            }else if(tempCounter == 1){
                counter = 0;
                reference = 0;
            }else if(tempCounter == 2){
                counter = 1;
                reference = 1100;
            }else if(tempCounter == 3){
                counter = 2;
                reference = 2500;
            }
        }
    }

}
