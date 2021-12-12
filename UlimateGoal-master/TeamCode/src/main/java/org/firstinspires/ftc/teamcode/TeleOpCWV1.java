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

@TeleOp(name="TeleOpCWV1", group="Linear Opmode")
//@Disabled
public class TeleOpCWV1 extends LinearOpMode {

    HardwareTest robot=new HardwareTest();
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
            double forward = -gamepad1.left_stick_y; // these are desired speeds
            double right = gamepad1.left_stick_x;
            double clockwise = gamepad1.right_stick_x;

            //We do this for slowdown and fine movements
            double slowdown=1.0-gamepad1.left_trigger;

            double lf = forward + right + clockwise;
            double lb = forward - right + clockwise;
            double rf = forward - right - clockwise;
            double rb = forward + right - clockwise;

            //Where all the ingredients come together (we set the power here)
            robot.frontleft.setPower(lf*slowdown);
            robot.downleft.setPower(lb*slowdown);
            robot.frontright.setPower(rf*slowdown);
            robot.downright.setPower(rb*slowdown);

            /*If the gamepad's right stick y value is greater than zero, give the sucker power.
            If the y value is less than zero, give it negative power. If the y value is zero, we give it no power. */
            if(gamepad2.right_stick_y>0.0){
                robot.sucker.setPower(1.0);
            }else if(gamepad2.right_stick_y<0.0){
                robot.sucker.setPower(-1.0);
            }else{
                robot.sucker.setPower(0.0);
            }

            //Set the wobble goal's arm to the value of the y value on gamepad2's left stick. (It is negative because the y value is negative for some reason)
            robot.wobblegoalarm.setPower(-gamepad2.left_stick_y/1.5);

            //If gamepad2's "a" button is pressed, activate the shooters. Else, put them to rest
            if(gamepad2.a){
                robot.shooterone.setPower(1.0);
                robot.shootertwo.setPower(1.0);
            }else{
                robot.shooterone.setPower(0.0);
                robot.shootertwo.setPower(0.0);
            }

            if(gamepad2.b){
                robot.intakepusher.setPosition(1.0);

            }else{
                robot.intakepusher.setPosition(0.0);
            }
            if(gamepad2.x){
                robot.rampservo.setPosition(0.5);
            }else{
                robot.rampservo.setPosition(0.0);
            }

            if(gamepad2.dpad_up){
                robot.wobblegoaler.setPosition(0.0);


            }else{
                robot.wobblegoaler.setPosition(1.0);

            }

            if(gamepad2.y){
                robot.secondarypusher.setPosition(1.0);
            }else{
                robot.secondarypusher.setPosition(0.0);
            }




        }
    }
}
