/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left

 */

@TeleOp(name="30hrRobot", group="Linear OpMode")
public class PIDattempt extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor slideExtendR = null;
    private DcMotor slideExtendL = null;
    private Servo clawPivot = null;
    private Servo clawOpen = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "flD");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "blD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "brD");
        slideExtendR = hardwareMap.get(DcMotor.class, "slideExtendR");
        slideExtendL = hardwareMap.get(DcMotor.class, "slideExtendL");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        clawOpen = hardwareMap.get(Servo.class, "clawOpen");


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        slideExtendL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideExtendR.setDirection(DcMotorSimple.Direction.FORWARD);

        ElapsedTime pidTimer = new ElapsedTime();
        double KpR = 1;
        double KpL = 1;
        double KiR = 1;
        double KiL = 1;
        double KdR = 10;
        double KdL = 10;

        double integralSumR = 0;
        double integralSumL = 0;
        double referenceR = slideExtendR.getCurrentPosition();
        double referenceL = slideExtendL.getCurrentPosition();
        double lastErrorR = 0;
        double lastErrorL = 0;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

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
                clawPivot.setPosition(clawPivot.getPosition() + gamepad2.left_stick_y*0.0001);
            }

            //pivot claw up
            if (gamepad2.left_stick_y > 0.25)
            {
                clawPivot.setPosition(clawPivot.getPosition() + gamepad2.left_stick_y*0.0001);
            }

            //open claw
            if (gamepad2.a)
            {
                clawOpen.setPosition(clawOpen.getPosition() + 0.0001);
            }

            //close claw
            if (gamepad2.x)
            {
                clawOpen.setPosition(clawOpen.getPosition() - 0.0001);
            }



            //extend slide
            if (gamepad2.right_trigger > 0.25)
            {
                referenceR = slideExtendR.getCurrentPosition();
                referenceL = slideExtendL.getCurrentPosition();
                lastErrorR = 0;
                lastErrorL = 0;
                integralSumR = 0;
                integralSumL = 0;
                pidTimer.reset();
                slideExtendR.setPower(gamepad2.right_trigger);
                slideExtendL.setPower(gamepad2.left_trigger);

            }
            //retract slide
            else if (gamepad2.left_trigger > 0.25)
            {
                referenceR = slideExtendR.getCurrentPosition();
                referenceL = slideExtendL.getCurrentPosition();
                lastErrorR = 0;
                lastErrorL = 0;
                integralSumR = 0;
                integralSumL = 0;
                pidTimer.reset();
                slideExtendR.setPower(-gamepad2.left_trigger/3);
                slideExtendL.setPower(-gamepad2.left_trigger/3);
            }
            else {

                double leftPos = slideExtendL.getCurrentPosition();
                double rightPos = slideExtendR.getCurrentPosition();

                double leftError = referenceL - leftPos;
                double rightError = referenceR - rightPos;

                double derivativeL = (leftError - lastErrorL) / pidTimer.seconds();
                double derivativeR = (rightError - lastErrorR) / pidTimer.seconds();

                double integralL = integralSumL + (leftError) * pidTimer.seconds();
                double integralR = integralSumR + (rightError) * pidTimer.seconds();

                double leftPow = (KpL * leftError) + (KiL * integralL) + (KdL * derivativeL);
                double rightPow = (KpR * rightError) + (KiR * integralR) + (KdR * derivativeR);

                slideExtendL.setPower(leftPow);
                slideExtendR.setPower(rightPow);

                lastErrorL = leftError;
                lastErrorR = rightError;
                pidTimer.reset();
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Pivot", "%4.2f, %4.2f", clawPivot.getPosition());
            //telemetry.addData("Opener", "%4.2f, %4.2f", clawOpen.getPosition());
            telemetry.update();
        }
        /*
        double lastError = 0; //vel probably
        double integralSum = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd;

        private double stopController(double vel)
        {
            double error = -1*vel;
            double derivative = (error-lastError)/pidTimer.seconds();
            double integral = integralSum + (error*pidTimer.seconds());
            double out = K

        }

         */
    }
}
