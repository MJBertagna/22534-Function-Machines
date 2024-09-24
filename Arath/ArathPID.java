package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "crashOutA")
public class BasicOpModeA extends OpMode {

    public double PIDSign = 1;
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
    public int counter = 0;
    int reference = 0;//Reference for the target position of the linear slide
    public boolean isRightBumperPressed = false;
    public boolean isLeftBumperPressed = false;

    public boolean isSquarePressed = false;
    public boolean isCirclePressed = false;
    public boolean intakeInToggle = false;
    public boolean intakeOutToggle = false;


    //public double powerLeft = 0;
    public double powerRight = 0;

    ElapsedTime timer = new ElapsedTime();
    public double lastError = 0;

    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;

    public Servo intakeArm = null;
    public CRServo intake = null;

    public Servo clawArm = null;
    public Servo claw = null;

    public DcMotorEx leftElevator = null;
    public DcMotorEx rightElevator = null;

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

        intakeArm = hardwareMap.get(Servo.class, "intakePivot");
        intake = hardwareMap.get(CRServo.class, "intakeSpin");

        clawArm = hardwareMap.get(Servo.class, "clawPivot");
        claw = hardwareMap.get(Servo.class, "clawOpen");

        leftElevator = hardwareMap.get(DcMotorEx.class, "lS");
        rightElevator = hardwareMap.get(DcMotorEx.class, "rS");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightElevator.setDirection(DcMotorSimple.Direction.REVERSE);

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        //////////////////////////////////////////////////////////////////////////////////////
        //Speed multiplier code
        //Controls to change the speed multiplier on the wheels
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

        ////////////////////////////////////////////////////////////////////////////////////////
        //Movement Code
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

        //Calculates the power for each wheel
        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        double frontLeftPower = (y - x - yaw) / denominator;
        double frontRightPower = (y + x + yaw) / denominator;
        double backLeftPower = (y + x - yaw) / denominator;
        double backRightPower = (y - x + yaw) / denominator;

        //Sets the power to each wheel
        frontRightMotor.setPower(frontRightPower * speedMultiplier);
        frontLeftMotor.setPower(frontLeftPower * speedMultiplier);
        backLeftMotor.setPower(backLeftPower * speedMultiplier);
        backRightMotor.setPower(backRightPower * speedMultiplier);


        ///////////////////////////////////////////////////////////////////////////////////////
        //Claw Code

        //CLAW ARM CODE
        //If the counter is set to the bottom position then set the arm servo to extend outwards
        double clawArmPosition = gamepad2.right_trigger * 0.625;
        clawArm.setPosition(1 - clawArmPosition);

        //Sets the claw position
        //Closes the claw
        if(gamepad2.triangle){
            claw.setPosition(0.3);
        }
        //Opens the claw
        if(gamepad2.cross){
            claw.setPosition(0.7);
        }


        ///////////////////////////////////////////////////////////////////////////////////////
        //Intake Code

        //INTAKE ARM CODE
        intakeArm.setPosition(gamepad2.left_trigger);


        //INTAKE CODE
        //Toggles the intake(outake) on and off
        if(gamepad2.square){
            intakeInToggle = false;
            if(!isSquarePressed){
                intakeOutToggle = !intakeOutToggle;
            }
            isSquarePressed = true;
        }else{
            isSquarePressed = false;
        }

        if(intakeOutToggle){
            intake.setPower(1);
        }


        //Toggle on and off for the intake
        if(gamepad2.circle){
            intakeOutToggle = false;
            if(!isCirclePressed){
                intakeInToggle = !intakeInToggle;
            }
            isCirclePressed = true;
        }else{
            isCirclePressed = false;
        }

        if(intakeInToggle){
            intake.setPower(-1);
        }

        //If both are false set the intake power to 0
        if(!intakeOutToggle && !intakeInToggle){
            intake.setPower(0);
        }

        /////////////////////////////////////////////////////////////////////////////////////
        //PID Constants
        if(gamepad1.square){
            PIDSign = -1;
        }else{
            PIDSign = 1;
        }
        if(gamepad1.triangle){
            Kp += 0.0001 * PIDSign;
        }
        if(gamepad1.circle){
            Ki += 0.0001 * PIDSign;
        }
        if(gamepad1.cross){
            Kd += 0.0001 * PIDSign;
        }


        if(Kp < 0)
            Kp = 0;
        if(Ki < 0)
            Ki = 0;
        if(Kd < 0)
            Kd = 0;


        /////////////////////////////////////////////////////////////////////////////////////
        //Elevator Code
        //Raises the counter by one to change the reference point for the linear slide
        if(gamepad2.right_bumper) {
            if (!isRightBumperPressed) {
                // do thing
                setCounter(counter, 1);
            }
            isRightBumperPressed = true;
        }else{
            isRightBumperPressed = false;
        }

        //Lowers the counter by one to change the reference point for the linear slide
        if(gamepad2.left_bumper) {
            if (!isLeftBumperPressed) {
                // do thing
                setCounter(counter, -1);
            }
            isLeftBumperPressed = true;
        }else{
            isLeftBumperPressed = false;
        }

        findElevatorPower(counter);
        //If the position of the linear slides is between a set range/interval
        //The elevator power from the PID will by 20% of what it found
        //Only uses the right elevator in order for both slides to maintain the same height
        if(rightElevator.getCurrentPosition() > reference - 100 && rightElevator.getCurrentPosition() < reference + 50){
            setElevatorPower(powerRight * 0.2, powerRight * 0.2);
        }else{
            //If not between the set range/interval it will use what the PID found
            setElevatorPower(powerRight, powerRight);
        }


        /////////////////////////////////////////////////////////////////////////
        //Telemetry Data

        //Adds data for the driver station to show
        telemetry.addData("Intake Arm Position", intakeArm.getPosition());
        telemetry.addData("Claw Arm Position", clawArm.getPosition());
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

    /////////////////////////////////////////////////////////////////////////////////////////

    //Positional PID for the linear slide
    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }


    //Finds the elevator power for the slides given a specific count
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
