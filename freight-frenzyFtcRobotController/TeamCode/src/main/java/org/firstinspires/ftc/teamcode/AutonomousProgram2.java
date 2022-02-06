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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoBlueCam", group="Pushbot")
//@Disabled
public class AutonomousProgram2 extends LinearOpMode {

    /* Declare OpMode members. */
    RobotOld robot   = new RobotOld();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static List<String> logs = new ArrayList<>();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initVision(hardwareMap);
        robot.lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        logs.add("Status,:Resetting Encoders");

        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rbDrive.getCurrentPosition(),
                robot.lbDrive.getCurrentPosition(),
                robot.lfDrive.getCurrentPosition(),
                robot.rfDrive.getCurrentPosition());
        telemetry.update();

        logs.add("Path0,:"+robot.rbDrive.getCurrentPosition()+" "+
                robot.lbDrive.getCurrentPosition()+" "+
                robot.lfDrive.getCurrentPosition()+" "+
                robot.rfDrive.getCurrentPosition());

        robot.Dropper1.setPosition(0.0);
        robot.Dropper2.setPosition(0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
       /* robot.lfDrive.setPower(0.5);
        robot.lbDrive.setPower(0.5);
        robot.rfDrive.setPower(0.5);
        robot.rbDrive.setPower(0.5);
        sleep(1000);*/


        encoderDrive(0.75, 8, 8, 8, 8, 5);
//
        int level=1;

        for(int i=0;i<3;i++) {
            level = i+1;
            sleep(500);
            List<Recognition> detectedObject = robot.detectObject(hardwareMap);
            if((detectedObject.size()>0 && detectedObject.get(0).getLabel() != "Marker")||(detectedObject.size()==0)){
                break;
            }
            encoderDrive(0.25, -10.5, 10.5, 10.5, -10.5, 5);

        }
        double strafe = ((3-level)*10.5);

        encoderDrive(0.5, -35-strafe, 35+strafe, 35+strafe, -28-strafe, 5);
        encoderDrive(0.4, 3, -4, 3, -4, 5);

        robot.carousel.setPower(0.5);
        sleep(2500);
        robot.carousel.setPower(-0.0);

        encoderDrive(0.4, -3, 3, -3, 3, 5);
        encoderDrive(0.7, 55, -55, -55, 65, 5);
        encoderDrive(0.2, -20, -20, -20, -20, 5);
        encoderDrive(0.5, 25, 25, 25, 25, 5);
        if(level == 1){
            robot.Wrist.setPosition(0.6);
            sleep(750);
            robot.arm.setPower(0.6);
            sleep(550);
            robot.arm.setPower(0.0);
            sleep(100);
//        robot.arm.setPower(-0.05);
//        sleep(100);
            robot.Dropper1.setPosition(0.25);
            robot.Dropper2.setPosition(0.25);
            sleep(750);
            robot.arm.setPower(-0.7);
            sleep(600);
            robot.arm.setPower(0.0);

            robot.Dropper1.setPosition(0.0);
            robot.Dropper2.setPosition(0.0);
            sleep(400);
            robot.Wrist.setPosition(0.1);
            sleep(400);
        }else if(level == 2){
            encoderDrive(0.1, -5, -5, -5, -5, 5);
            robot.Wrist.setPosition(0.6);
            sleep(500);
            robot.arm.setPower(0.6);
            sleep(550);
            robot.arm.setPower(0.0);
            sleep(100);
//        robot.arm.setPower(-0.05);
//        sleep(100);
            robot.Dropper1.setPosition(0.25);
            robot.Dropper2.setPosition(0.25);
            sleep(750);
            encoderDrive(0.1, -1, -1, -1, -1, 5);
            robot.arm.setPower(-0.8);
            sleep(800);
            robot.arm.setPower(0.0);
            robot.Dropper1.setPosition(0.0);
            robot.Dropper2.setPosition(0.0);
            sleep(500);
            robot.Wrist.setPosition(0.1);
            sleep(400);
        } else {
            encoderDrive(0.1, -10, -10, -10, -10, 5);
            robot.Wrist.setPosition(0.6);
            sleep(500);
            robot.arm.setPower(0.6);
            sleep(550);
            robot.arm.setPower(0.0);
            sleep(100);
//        robot.arm.setPower(-0.05);
//        sleep(100);
            robot.Dropper1.setPosition(0.25);
            robot.Dropper2.setPosition(0.25);
            sleep(750);
            robot.arm.setPower(-0.8);
            sleep(800);
            robot.arm.setPower(0.0);

            robot.Dropper1.setPosition(0.0);
            robot.Dropper2.setPosition(0.0);
            sleep(400);
            robot.Wrist.setPosition(0.1);
            sleep(400);
        }


        encoderDrive(0.3, -5, -5, -5, -5, 5);
        encoderDrive(0.7, -65, 65, 65, -65, 5);





        switch (level){
            case(1):
//                sleep(1000);
//                robot.arm.setPower(0.6);
//                sleep(1000);
////        robot.arm.setPower(-0.05);
////        sleep(100);
//                robot.arm.setPower(0.0);
//                robot.Wrist.setPosition(0.6);
//                sleep(500);
//                robot.Dropper1.setPosition(0.25);
//                robot.Dropper2.setPosition(0.25);
//                sleep(500);
//                robot.arm.setPower(-0.4);
//                sleep(300);
//                robot.arm.setPower(0.0);
//
//                robot.Dropper1.setPosition(0.0);
//                robot.Dropper2.setPosition(0.0);
//                sleep(300);
//
//
//                robot.Wrist.setPosition(0.0);
//                sleep(300);
//
//                encoderDrive(0.3, -5, -5, -5, -5, 5);
//
//
//                robot.arm.setPower(-0.6);
//                sleep(550);
//                robot.arm.setPower(0.0);
                break;
            case(2):

                break;
            case(3):

                break;
        }

//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
        logs.add("Path,:Complete");

//        for(String a:logs){
//            telemetry.addData(a.split(",:")[0],a.split(",:")[1]);
//        }
        telemetry.update();
//        sleep(30000);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double frontleftInches, double frontrightInches,double downleftInches, double downrightInches,
                             double timeoutS) {
        int newfrontleftTarget;
        int newfrontrightTarget;
        int newdownleftTarget;
        int newdownrightTarget;

//        frontleftInches = (-frontleftInches);
//        downleftInches = (-downleftInches);
//        downrightInches = (-downrightInches);
//        frontrightInches = (-frontrightInches);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newfrontleftTarget = robot.lfDrive.getCurrentPosition() + (int)(-(frontleftInches) * COUNTS_PER_INCH/7);
            newfrontrightTarget = robot.rfDrive.getCurrentPosition() + (int)(-(frontrightInches) * COUNTS_PER_INCH/7);
            newdownleftTarget = robot.lbDrive.getCurrentPosition() + (int)(-(downleftInches) * COUNTS_PER_INCH/7);
            newdownrightTarget = robot.rbDrive.getCurrentPosition() + (int)(-(downrightInches) * COUNTS_PER_INCH/7);
            robot.lfDrive.setTargetPosition(newfrontleftTarget);
            robot.rfDrive.setTargetPosition(newfrontrightTarget);
            robot.lbDrive.setTargetPosition(newdownleftTarget);
            robot.rbDrive.setTargetPosition(newdownrightTarget);

            // Turn On RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.lfDrive.setPower(Math.abs(speed));
            robot.rfDrive.setPower(Math.abs(speed));
            robot.lbDrive.setPower(Math.abs(speed));
            robot.rbDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.lfDrive.isBusy() && robot.rfDrive.isBusy()
                            && robot.lbDrive.isBusy() && robot.rbDrive.isBusy()))

            {

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d",newfrontleftTarget,
                        newfrontrightTarget,newdownleftTarget,newdownrightTarget);
                logs.add("Path1,:"+newdownleftTarget+" "+newfrontrightTarget+" "+newdownleftTarget+" "+newdownrightTarget);

                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition()
                        ,robot.lbDrive.getCurrentPosition(),
                        robot.rbDrive.getCurrentPosition());
                logs.add("Path2,:"+robot.rbDrive.getCurrentPosition()+" "+
                        robot.lbDrive.getCurrentPosition()+" "+
                        robot.lfDrive.getCurrentPosition()+" "+
                        robot.rfDrive.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            robot.lfDrive.setPower(0);
            robot.rfDrive.setPower(0);
            robot.lbDrive.setPower(0);
            robot.rbDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //  sleep(250);   // optional pause after each move
        }

    }
}