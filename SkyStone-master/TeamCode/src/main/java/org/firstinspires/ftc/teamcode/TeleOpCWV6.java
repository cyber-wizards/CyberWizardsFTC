/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpCWV6", group="Linear Opmode")
//@Disabled
public class TeleOpCWV6 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor downleft;
    DcMotor downright;
    DcMotor GrabberMotor1;
    DcMotor GrabberMotor2;
    DcMotor Arm;
    Servo wrist;
    Servo Collector;
    Servo FrontCollector;
    Servo FoundationGrabber1;
    Servo FoundationGrabber2;
    Servo Capstone;
    Servo GrabberRamp;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        downleft = hardwareMap.get(DcMotor.class, "downleft");
        downright = hardwareMap.get(DcMotor.class, "downright");
        GrabberMotor1 = hardwareMap.get(DcMotor.class, "GrabberMotor1");
        GrabberMotor2 = hardwareMap.get(DcMotor.class, "GrabberMotor2");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        wrist = hardwareMap.servo.get("wrist");
        Collector = hardwareMap.servo.get("collector");
        FrontCollector = hardwareMap.servo.get("FrontCollector");
        FoundationGrabber1 = hardwareMap.servo.get("FoundationGrabber1");
        FoundationGrabber2 = hardwareMap.servo.get("FoundationGrabber2");
        Capstone = hardwareMap.servo.get("Capstone");
        GrabberRamp = hardwareMap.servo.get("GrabberRamp");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        downleft.setDirection(DcMotor.Direction.FORWARD);
        downright.setDirection(DcMotor.Direction.REVERSE);
        GrabberMotor1.setDirection(DcMotor.Direction.FORWARD);
        GrabberMotor2.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);

        FoundationGrabber1.setPosition(1.0);
        FoundationGrabber2.setPosition(0.0);
        FrontCollector.setPosition(1.0);
        Collector.setPosition(1.0);
        wrist.setPosition(0.93);
        Capstone.setPosition(1.0);
        GrabberRamp.setPosition(0.0);



        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            frontleft.setPower(gamepad1.left_stick_y);
            downleft.setPower(gamepad1.left_stick_y);
            frontright.setPower(gamepad1.right_stick_y);
            downright.setPower(gamepad1.right_stick_y);


            frontleft.setPower(gamepad1.right_trigger);
            //We are moving the front left motor forward
            downleft.setPower(-gamepad1.right_trigger);
            //We are moving the back left motor backwards
            frontright.setPower(-gamepad1.right_trigger);
            //We are moving the front right motor backwards
            downright.setPower(gamepad1.right_trigger);
            //We are moving the back right motor forward
            frontleft.setPower(-gamepad1.left_trigger);
            //We are moving the front left motor backwards
            downleft.setPower(gamepad1.left_trigger);
            //We are moving the back left motor forward
            frontright.setPower(gamepad1.left_trigger);
            //We are moving the front right motor forward
            downright.setPower(-gamepad1.left_trigger);
            // We are moving the back right motor backwards.
            if (gamepad1.left_bumper){
                FoundationGrabber1.setPosition(0.0);
                FoundationGrabber2.setPosition(1.0);
            } else {
                FoundationGrabber1.setPosition(1.0);
                FoundationGrabber2.setPosition(0.0);

            }
            if (gamepad1.right_bumper){
                FrontCollector.setPosition(0.0);
            } else {
                FrontCollector.setPosition(1.0);
            }
            GrabberMotor1.setPower(-gamepad2.right_stick_y);
            GrabberMotor2.setPower(-gamepad2.right_stick_y);
            Arm.setPower(-gamepad2.left_stick_y/1.5);

            if(gamepad2.left_bumper){
//
                wrist.setPosition(0.0);
            } else {
                wrist.setPosition(0.93);
            }

            if (gamepad2.right_bumper){
                Collector.setPosition(0.0);
            } else {
                Collector.setPosition(1.0);

            }

            if (gamepad2.x){
                Capstone.setPosition(0.0);
            } else {
                Capstone.setPosition(1.0);

            }

            if (gamepad2.b){
                GrabberRamp.setPosition(1.0);
            } else {
                GrabberRamp.setPosition(0.0);

            }







        }
    }
}