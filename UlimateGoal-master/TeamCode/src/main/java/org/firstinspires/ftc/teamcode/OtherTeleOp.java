package org.firstinspires.ftc.teamcode;
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



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OtherTeleOp", group="Linear Opmode")
public class OtherTeleOp extends LinearOpMode {
    HardwareTest2 robot = new HardwareTest2();
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


        robot.frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.downleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.downright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        robot.frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.downleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.downright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            if(gamepad1.left_stick_y<0){
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
            if(gamepad1.left_stick_y>0) {
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


            //encoderDrive(0.8,0.4,1.0,1.0);
            if(gamepad1.left_trigger>0){
                robot.downleft.setPower(0.9);
                robot.downright.setPower(-0.95);
                robot.frontleft.setPower(-0.95);
                robot.frontright.setPower(0.9);
            }  else {
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }
            if(gamepad1.right_trigger>0){
                robot.downleft.setPower(-1.0);
                robot.downright.setPower(1.0);
                robot.frontleft.setPower(1.0);
                robot.frontright.setPower(-0.8);
            } else {
                robot.downleft.setPower(0.0);
                robot.downright.setPower(0.0);
                robot.frontleft.setPower(0.0);
                robot.frontright.setPower(0.0);
            }



            if(gamepad1.right_stick_x<0){
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
            if(gamepad1.right_stick_x>0){
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

            robot.wobblegoalarm.setPower(gamepad2.right_trigger*0.7-gamepad2.left_trigger*0.33);
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
            } else if(gamepad2.y){
                robot.shooterone.setPower(-0.85);
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
            if(gamepad2.x){
                robot.wobblestop.setPosition(0.0);
            } else {
                robot.wobblestop.setPosition(1.0);
            }



        }
    }
}
