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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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

@Autonomous(name="BlueSideRight", group="Pushbot")
//@Disabled
public class BlueSideRight extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareTest2 robot   = new HardwareTest2();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    static final double     COUNTS_PER_MOTOR_REV    = 480 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.585 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     DRIVE_SPEED2            = 0.45;
    static final double     DRIVE_SPEED3            = 0.2;
    static final double     DRIVE_SPEED4            = 0.75;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //Change this to the next comment line to get webcam working
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);


        // Don't burn CPU cycles busy-looping in this sample
        sleep(50);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.downleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.downright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.downleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.downright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.frontleft.getCurrentPosition(),
                robot.frontright.getCurrentPosition()
                ,robot.downleft.getCurrentPosition(),
                robot.downright.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.downleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.downright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderDrive(1.0,84,0.45,-84,0.85,-84,1.0,84,5.0);
        sleep(1000);

        phoneCam.stopStreaming();
        sleep(500);
        /*if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
            phoneCam.stopStreaming();
            phoneCam.closeCameraDevice();
            encoderDrive(DRIVE_SPEED,3.5,1,3.5,1,5.0);
            encoderDrive(DRIVE_SPEED,145,145,145,145,5.0);
            encoderDrive(DRIVE_SPEED,-1.5,2.5,-1.5,2.5,5.0);
            encoderDrive(DRIVE_SPEED,84,-84,-84,84,5.0);
            encoderDrive(DRIVE_SPEED,20,20,20,20,5.0);
            encoderDrive(DRIVE_SPEED,-2,1,-2,1,5.0);
            encoderDrive(DRIVE_SPEED,5,-5,-5,5,5.0);
            robot.wobblegoalarm.setPower(-0.45);
            sleep(600);
            robot.wobblegoaler.setPosition(0.0);
            encoderDrive(DRIVE_SPEED4,10,-10,-10,10,5.0);
            sleep(500);
            robot.wobblegoaler.setPosition(1.0);
            encoderDrive(DRIVE_SPEED,1,-2,1,-2,5.0);
            encoderDrive(DRIVE_SPEED4,-53,-53,-53,-53,5.0);
            encoderDrive(DRIVE_SPEED,-8.5,8.5,8.5,-8.5,5.0);
            encoderDrive(DRIVE_SPEED,-1,0.5,-1,0.5,5.0);

            for(int i=0;i<3;i++){
                robot.shooterone.setPower(-1.0);
                sleep(2000);
                robot.Launcher.setPosition(1.0);
                sleep(500);
                robot.Launcher.setPosition(0.0);

            }





        } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
            phoneCam.stopStreaming();
            phoneCam.closeCameraDevice();
            encoderDrive(DRIVE_SPEED,4,1.5,4,1.5,5.0);

            encoderDrive(DRIVE_SPEED,180,180,180,180,5.0);
            robot.wobblegoalarm.setPower(-0.45);
            sleep(600);
            robot.wobblegoaler.setPosition(0.0);
            encoderDrive(DRIVE_SPEED4,20,-20,-20,20,5.0);
            sleep(500);
            robot.wobblegoaler.setPosition(1.0);
            encoderDrive(DRIVE_SPEED,-2,2.5,-2,2.5,5.0);
            encoderDrive(DRIVE_SPEED,90,-90,-90,90,5.0);
            encoderDrive(DRIVE_SPEED,-65,-65,-65,-65,5.0);
            encoderDrive(DRIVE_SPEED,-20,20,20,-20,5.0);
            encoderDrive(DRIVE_SPEED,7,9,7,9,5.0);
            for(int i=0;i<3;i++){
                robot.shooterone.setPower(-1.0);
                sleep(2000);
                robot.Launcher.setPosition(1.0);
                sleep(500);
                robot.Launcher.setPosition(0.0);

            }



        }else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
            phoneCam.stopStreaming();
            phoneCam.closeCameraDevice();
            encoderDrive(DRIVE_SPEED,4,1.5,4,1.5,5.0);
            encoderDrive(DRIVE_SPEED,105,105,105,105,5.0);
            encoderDrive(DRIVE_SPEED,5,-5,-5,5,5.0);
            robot.wobblegoalarm.setPower(-0.45);
            sleep(600);
            robot.wobblegoaler.setPosition(0.0);
            encoderDrive(DRIVE_SPEED4,17,-17,-17,17,5.0);
            sleep(500);
            robot.wobblegoaler.setPosition(1.0);
            encoderDrive(DRIVE_SPEED,-2,2.5,-2,2.5,5.0);
            encoderDrive(DRIVE_SPEED,62,-62,-62,62,5.0);
            encoderDrive(DRIVE_SPEED,7,8,7,8,5.0);
            for(int i=0;i<3;i++){
                robot.shooterone.setPower(-1.0);
                sleep(2000);
                robot.Launcher.setPosition(1.0);
                sleep(500);
                robot.Launcher.setPosition(0.0);

            }



        }

        /*encoderDrive(DRIVE_SPEED2,40,-40,-40,40,5.0);
        encoderDrive(DRIVE_SPEED2,-25,-25,-25,-25,5.0);
        sleep(1000);
        for(int i=0;i<2;i++){
            robot.intakepusher.setPosition(0.0);
            sleep(500);
            robot.intakepusher.setPosition(1.0);
            sleep(500);

            robot.shooterone.setPower(1.0);
            robot.shootertwo.setPower(1.0);
            sleep(3000);
        }
        encoderDrive(DRIVE_SPEED4,24,24,24,24,5.0);

         */




        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


        //This be da home of the encoderDrive method. Protecc the home of the encoderDrive method...Respecc







        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double frontleftInches,double speed2,double frontrightInches,double speed3,double downleftInches,double speed4, double downrightInches,
                             double timeoutS) {
        int newfrontleftTarget;
        int newfrontrightTarget;
        int newdownleftTarget;
        int newdownrightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newfrontleftTarget = robot.frontleft.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH*speed);
            newfrontrightTarget = robot.frontright.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH*speed2);
            newdownleftTarget = robot.downleft.getCurrentPosition() + (int)(downleftInches * COUNTS_PER_INCH*speed3);
            newdownrightTarget = robot.downright.getCurrentPosition() + (int)(downrightInches * COUNTS_PER_INCH*speed4);
            robot.frontleft.setTargetPosition(newfrontleftTarget);
            robot.frontright.setTargetPosition(newfrontrightTarget);
            robot.downleft.setTargetPosition(newdownleftTarget);
            robot.downright.setTargetPosition(newdownrightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.downleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.downright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontleft.setPower(Math.abs(speed));
            robot.frontright.setPower(Math.abs(speed2));
            robot.downleft.setPower(Math.abs(speed3));
            robot.downright.setPower(Math.abs(speed4));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontleft.isBusy() && robot.frontright.isBusy()
                            && robot.downleft.isBusy() && robot.downright.isBusy()))

            {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", pipeline.position);
                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d",newfrontleftTarget,
                        newfrontrightTarget,newdownleftTarget,newdownrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontleft.getCurrentPosition(),
                        robot.frontright.getCurrentPosition()
                        ,robot.downleft.getCurrentPosition(),
                        robot.downright.getCurrentPosition());
                telemetry.update();
            }



            // Stop all motion;
            robot.frontleft.setPower(0);
            robot.frontright.setPower(0);
            robot.downleft.setPower(0);
            robot.downright.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.downleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.downright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }

    }
    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(215,30);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystoneDeterminationPipeline.RingPosition position = SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

}
