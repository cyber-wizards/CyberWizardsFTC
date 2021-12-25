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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.Robot;

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

@Autonomous(name="Auto", group="Pushbot")
//@Disabled
public class AutonomousProgram extends LinearOpMode {

    /* Declare OpMode members. */
    Robot robot   = new Robot();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
//            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
            "AR4BaoH/////AAABmUfI9MVryEosoKpSgalFS9xaJ1QLidk13Y6d1uRhcd+USJBp9UCErESjjaqqRDiuhhF+dATZ1RinzpA4BeK3ogznKGKzd18DH7/1vOLhmJL2WH1iACJj5UytH6HoELaKMROQrCHKUSPamRT2617qldBngNtU+rjq3Wu6bxTTIU5aYIikuWKGi9K6XKwBQywcVMEBU1WbXkp2gUCMR8kLMP7mMRN0CalzcWu/PDa73t4wJeg4us6UZrUW7RcTR+FLZuYOEYZuhw0Ny0dLOkwtqCuleqMlF5veyp9U3QqZ4guCkkfUgE5vByNTHdoOiCXqE2J4ZfOKqHRrw54H4uOL4B44mp4Bkk/JcXWMLVPd4G7a";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public String detectObject(HardwareMap hardwareMap) {

        /**
         * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
         * determine the position of the Freight Frenzy game elements.
         *
         * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
         * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
         *
         * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
         * is explained below.
         */
//    @TeleOp(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
//@Disabled
//    public class ConceptTensorFlowObjectDetectionWebcamTest extends LinearOpMode {
        /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
         * the following 4 detectable objects
         *  0: Ball,
         *  1: Cube,
         *  2: Duck,
         *  3: Marker (duck location tape marker)
         *
         *  Two additional model assets are available which only contain a subset of the objects:
         *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
         *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
         */


//        @Override
//        public void runOpMode () {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(hardwareMap);
        initTfod(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        String currentObject = "";
//
        for(int i=0;i<5;i++){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null && updatedRecognitions.size()>0) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
//                    int i = 0;
//                    for (Recognition recognition : updatedRecognitions) {
//                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
////                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
////                                recognition.getLeft(), recognition.getTop());
////                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
////                                recognition.getRight(), recognition.getBottom());
//                        i++;
//                    }
                    currentObject = updatedRecognitions.get(0).getLabel();
//                    telemetry.update();
                }
            }
            this.sleep(1000);
        }
        return currentObject;
    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia(HardwareMap hardwareMap) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rbDrive.getCurrentPosition(),
                robot.lbDrive.getCurrentPosition(),
                robot.lfDrive.getCurrentPosition(),
                robot.rfDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("U:", "phrates river");
        telemetry.update();

        encoderDrive(0.5, -13, -13, -13, -13, 5);


        String detectedObject = detectObject(hardwareMap);



        if(detectedObject == "Duck"){
            encoderDrive(0.5, -13, -13, -13, -13, 5);
        }



        telemetry.addData("Object:", detectedObject);
        telemetry.update();

//        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
//        robot.rightClaw.setPosition(0.0);
//        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newfrontleftTarget = robot.lfDrive.getCurrentPosition() + (int)(frontleftInches * COUNTS_PER_INCH);
            newfrontrightTarget = robot.rfDrive.getCurrentPosition() + (int)(frontrightInches * COUNTS_PER_INCH);
            newdownleftTarget = robot.lbDrive.getCurrentPosition() + (int)(downleftInches * COUNTS_PER_INCH);
            newdownrightTarget = robot.rbDrive.getCurrentPosition() + (int)(downrightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.lfDrive.getCurrentPosition(),
                        robot.rfDrive.getCurrentPosition()
                        ,robot.lbDrive.getCurrentPosition(),
                        robot.rbDrive.getCurrentPosition());
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


            //  sleep(250);   // optional pause after each move
        }

    }
}
