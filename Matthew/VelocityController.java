package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left

 */

@TeleOp(name="30hrRobot", group="Linear OpMode")
public class VelocityController extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx slideExtendR = null;
    private DcMotorEx slideExtendL = null;
    private Servo clawPivot = null;
    private Servo clawOpen = null;

    double lastErrorLeft = 0; //vel probably
    double integralSumLeft = 0;
    double KpL = 0;
    double KiL = 0;
    double KdL = 0;

    double lastErrorRight = 0; //vel probably
    double integralSumRight = 0;
    double KpR = 0;
    double KiR = 0;
    double KdR = 0;
    ElapsedTime pidTimer = new ElapsedTime();
    ElapsedTime pidAdjustTimer = new ElapsedTime();
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "flD");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "blD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brD");
        slideExtendR = hardwareMap.get(DcMotorEx.class, "slideExtendR");
        slideExtendL = hardwareMap.get(DcMotorEx.class, "slideExtendL");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        clawOpen = hardwareMap.get(Servo.class, "clawOpen");


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideExtendL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideExtendR.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        pidTimer.reset();
        pidAdjustTimer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }




            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //pivot claw down
            if (gamepad2.left_stick_y < -0.25)
            {
                clawPivot.setPosition(clawPivot.getPosition() + gamepad2.left_stick_y*0.000001);
            }

            //pivot claw up
            if (gamepad2.left_stick_y > 0.25)
            {
                clawPivot.setPosition(clawPivot.getPosition() + gamepad2.left_stick_y*0.000001);
            }

            //open claw
            if (gamepad2.a)
            {
                clawOpen.setPosition(0);
            }

            //close claw
            if (gamepad2.x)
            {
                clawOpen.setPosition(0.3);
            }

            double deltaT = pidAdjustTimer.seconds();

            if (gamepad1.dpad_up)
            {
                KpL += 1*deltaT;
                KpR += 1*deltaT;
            }
            if (gamepad1.dpad_down)
            {
                KpL -= 1*deltaT;
                KpR -= 1*deltaT;
            }
            if (gamepad1.dpad_right)
            {
                KiL += 1*deltaT;
                KiR += 1*deltaT;
            }
            if (gamepad1.dpad_left)
            {
                KiL -= 1*deltaT;
                KiR -= 1*deltaT;
            }
            if (gamepad1.right_bumper)
            {
                KdL += 1*deltaT;
                KdR += 1*deltaT;
            }
            if (gamepad1.left_bumper)
            {
                KdL -= 1*deltaT;
                KdR -= 1*deltaT;
            }
            pidAdjustTimer.reset();




            //extend slide
            if (gamepad2.right_trigger > 0.25)
            {
                //slideExtendR.setPower(gamepad2.right_trigger);
                slideExtendL.setPower(gamepad2.right_trigger);
                integralSumLeft = 0;
                integralSumRight = 0;
                lastErrorLeft = -1*slideExtendL.getVelocity();
                lastErrorRight = -1*slideExtendR.getVelocity();
            }
            //retract slide
            else if (gamepad2.left_trigger > 0.25)
            {
                //slideExtendR.setPower(-gamepad2.left_trigger/3);
                slideExtendL.setPower(-gamepad2.left_trigger/3);
                integralSumLeft = 0;
                integralSumRight = 0;
                lastErrorLeft = -1*slideExtendL.getVelocity();
                lastErrorRight = -1*slideExtendR.getVelocity();
            }
            else
            {
                pidTimer.reset();
                slideExtendL.setPower(stopControllerLeft(slideExtendL.getVelocity()));
                //slideExtendR.setPower(stopControllerRight(slideExtendR.getVelocity()));
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Pivot", clawPivot.getPosition());
            //telemetry.addData("Opener", clawOpen.getPosition());
            telemetry.addData("Kp", KpL);
            telemetry.addData("Ki", KiL);
            telemetry.addData("Kd", KdL);
            telemetry.update();
        }

    }
    private double stopControllerLeft(double vel)
    {
        double error = -1*vel;
        double derivative = (error-lastErrorLeft)/pidTimer.seconds();
        double integral = integralSumLeft + (error*pidTimer.seconds());
        double out = KpL*(error) + KiL*(integral) + KdL*(derivative);
        lastErrorLeft = error;
        return out;
    }
    private double stopControllerRight(double vel)
    {
        double error = -1*vel;
        double derivative = (error-lastErrorRight)/pidTimer.seconds();
        double integral = integralSumRight + (error*pidTimer.seconds());
        double out = KpR*(error) + KiR*(integral) + KdR*(derivative);
        lastErrorRight = error;
        return out;
    }
}
