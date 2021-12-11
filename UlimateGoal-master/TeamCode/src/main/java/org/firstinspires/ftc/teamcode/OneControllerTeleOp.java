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
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;

//This is test
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

@TeleOp(name="OneControllerTeleOp", group="Linear Opmode")
//@Disabled
public class OneControllerTeleOp extends LinearOpMode {

    HardwareTest2 robot=new HardwareTest2();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    /*DcMotor frontleft;
    DcMotor frontright;
    DcMotor downleft;
    DcMotor downright;*/





    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        robot.frontleft.setDirection(DcMotor.Direction.FORWARD);
        robot.frontright.setDirection(DcMotor.Direction.REVERSE);
        robot.downleft.setDirection(DcMotor.Direction.FORWARD);
        robot.downright.setDirection(DcMotor.Direction.REVERSE);





        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            /*if (gamepad1.left_stick_y > 0.0){
                frontleft.setPower(1.0);
                downleft.setPower(1.0);
            } else{
                frontleft.setPower(0.0);
                downleft.setPower(0.0);
            }
            if (gamepad1.right_stick_y > 0.0){
                frontright.setPower(1.0);
                downright.setPower(1.0);
            } else{
                frontright.setPower(0.0);
                downright.setPower(0.0);
            }
            if (gamepad1.left_stick_y < 0.0){
                frontleft.setPower(-1.0);
                downleft.setPower(-1.0);
            } else{
                frontleft.setPower(0.0);
                downleft.setPower(0.0);
            }
            if (gamepad1.right_stick_y < 0.0){
                frontright.setPower(-1.0);
                downright.setPower(-1.0);
            } else{
                frontright.setPower(0.0);
                downright.setPower(0.0);
            }


            if (gamepad1.left_trigger > 0.0){
                frontleft.setPower(1.0);
                //We are moving the front left motor forward
                downleft.setPower(-1.0);
                //We are moving the back left motor backwards
                frontright.setPower(-1.0);
                //We are moving the front right motor backwards
                downright.setPower(1.0);
                //We are moving the back right motor forward
            }else{
                frontleft.setPower(0.0);
                downleft.setPower(0.0);
                frontright.setPower(0.0);
                downright.setPower(0.0);
            }
            if (gamepad1.right_trigger > 0.0) {
                frontleft.setPower(-1.0);
                //We are moving the front left motor backwards
                downleft.setPower(1.0);
                //We are moving the back left motor forward
                frontright.setPower(1.0);
                //We are moving the front right motor forward
                downright.setPower(-1.0);
                // We are moving the back right motor backwards.
            }else{
                frontleft.setPower(0.0);
                downleft.setPower(0.0);
                frontright.setPower(0.0);
                downright.setPower(0.0);
            }*/

            //Make it for omni-directional movement. It is confusing but once you run through it, it becomes easier
            /*double forward = -gamepad1.left_stick_y; // these are desired speeds
            double right = gamepad1.left_stick_x;
            double clockwise=0;

            if(gamepad1.right_stick_x<0.5 && gamepad1.right_stick_x>-0.5) {
                if (gamepad1.right_stick_y > 0.0) {
                    robot.sucker.setPower(1.0);
                } else if (gamepad1.right_stick_y < 0.0) {
                    robot.sucker.setPower(-1.0);
                } else {
                    robot.sucker.setPower(0.0);
                }
            }else{
                clockwise = gamepad1.right_stick_x;
            }*/


            /*double angle = Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x);
            double magnitude=Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(-gamepad1.left_stick_y,2));

            robot.frontleft.setPower(Math.sin(angle+(Math.PI/4))* magnitude+gamepad1.right_stick_x);
            robot.downleft.setPower(Math.sin(angle-(Math.PI)/4)* magnitude+gamepad1.right_stick_x);
            robot.frontright.setPower(Math.sin(angle-(Math.PI/4))* magnitude - gamepad1.right_stick_x);
            robot.downright.setPower(Math.sin(angle+(Math.PI/4))* magnitude - gamepad1.right_stick_x);*/

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.5;
            double rx = gamepad1.right_stick_x;


            double frontLeftPower = y+x+rx;
            double backLeftPower = y-x+rx;
            double frontRightPower = y-x-rx;
            double backRightPower = y+x-rx;


            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }

            robot.frontleft.setPower(frontLeftPower);
            robot.downleft.setPower(backLeftPower);
            robot.frontright.setPower(frontRightPower);
            robot.downright.setPower(backRightPower);





            /*If the gamepad's right stick y value is greater than zero, give the sucker power.
            If the y value is less than zero, give it negative power. If the y value is zero, we give it no power. */



            //Set the wobble goal's arm to the value of the y value on gamepad2's left stick. (It is negative because the y value is negative for some reason)
            //robot.wobblegoalarm.setPower(-gamepad2.left_stick_y/1.5);



            if (gamepad1.left_trigger > 0.0){
                robot.frontleft.setPower(-1.0);
                //We are moving the front left motor forward
                robot.downleft.setPower(1.0);
                //We are moving the back left motor backwards
                robot.frontright.setPower(1.0);
                //We are moving the front right motor backwards
                robot.downright.setPower(-1.0);
                //We are moving the back right motor forward
            }else{
                robot.frontleft.setPower(0.0);
                robot.downleft.setPower(0.0);
                robot.frontright.setPower(0.0);
                robot.downright.setPower(0.0);
            }

            if (gamepad1.right_trigger > 0.0) {
                robot.frontleft.setPower(1.0);
                //We are moving the front left motor backwards
                robot.downleft.setPower(-1.0);
                //We are moving the back left motor forward
                robot.frontright.setPower(-1.0);
                //We are moving the front right motor forward
                robot.downright.setPower(1.0);
                // We are moving the back right motor backwards.
            }else{
                robot.frontleft.setPower(0.0);
                robot.downleft.setPower(0.0);
                robot.frontright.setPower(0.0);
                robot.downright.setPower(0.0);
            }

            if(gamepad1.left_stick_y>0){
                robot.downleft.setPower(1.0);
                robot.downright.setPower(1.0);
                robot.frontleft.setPower(1.0);
                robot.frontright.setPower(1.0);
            } else{
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }
            if(gamepad1.left_stick_y<0) {
                robot.downleft.setPower(-1.0);
                robot.downright.setPower(-1.0);
                robot.frontleft.setPower(-1.0);
                robot.frontright.setPower(-1.0);
            }else{
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }

            if(gamepad1.right_stick_x<-0.5){
                robot.downleft.setPower(-1.0);
                robot.downright.setPower(1.0);
                robot.frontleft.setPower(-1.0);
                robot.frontright.setPower(1.0);
            }  else {
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }
            if(gamepad1.right_stick_x>0.5){
                robot.downleft.setPower(1.0);
                robot.downright.setPower(-1.0);
                robot.frontleft.setPower(1.0);
                robot.frontright.setPower(-1.0);
            } else {
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }







            robot.wobblegoalarm.setPower(gamepad2.right_trigger-gamepad2.left_trigger/1.5);
            if (gamepad2.left_stick_y > 0) {
                robot.Wheelintake.setPower(1.0);
                robot.sucker.setPower(1.0);
            }
            else{
                robot.Wheelintake.setPower(0.0);
                robot.sucker.setPower(0.0);
            }
            if(gamepad2.left_bumper){
                robot.rampservo.setPosition(0.6);
            }else{
                robot.rampservo.setPosition(0.0);
            }
            if(gamepad2.right_stick_y < 0){
                robot.shooterone.setPower(-1.0);
            }
            else{
                robot.shooterone.setPower(0.0);
            }
            if(gamepad2.right_bumper){
                robot.Launcher.setPosition(1.0);
            }else{
                robot.Launcher.setPosition(0.0);
            }

            if(gamepad2.b){
                robot.wobblegoaler.setPosition(0.0);


            }else{
                robot.wobblegoaler.setPosition(1.0);

            }




        }
    }
}
