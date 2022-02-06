package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//
//import java.util.ArrayList;
//import java.util.List;


public class RobotT1 {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lfDrive = null;
    public DcMotor rfDrive = null;
    public DcMotor lbDrive = null;
    public DcMotor rbDrive = null;
    public DcMotor slider = null;
    public DcMotor intake = null;
    public Servo dropper = null;
    public WebcamName Webcam = null;
    public DcMotor carousel = null;
//    public Servo Dropper1 = null;
//    public Servo Dropper2 = null;
//    public Servo Wrist = null;
//    public WebcamName Webcam = null;

    public void initVision(HardwareMap hardwareMap){
        this.initVuforia();
        this.initTfod(hardwareMap);
    }

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public void sleep(long time){
        try{
            Thread.sleep(time);
        }catch(InterruptedException e){
            e.printStackTrace();
        }
    }

    public void init(HardwareMap hardwareMap){
        rfDrive = hardwareMap.get(DcMotor.class, "rfDrive");
        rbDrive = hardwareMap.get(DcMotor.class, "rbDrive");
        lfDrive  = hardwareMap.get(DcMotor.class, "lfDrive");
        lbDrive  = hardwareMap.get(DcMotor.class, "lbDrive");

        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

//        Dropper1  = hardwareMap.get(Servo.class, "dropper1");
//        Dropper2 = hardwareMap.get(Servo.class, "dropper2");
//        Wrist = hardwareMap.get(Servo.class, "wrist");
//        Webcam = hardwareMap.get(WebcamName.class, "Webcam 1");


        slider  = hardwareMap.get(DcMotor.class, "slider");
        intake  = hardwareMap.get(DcMotor.class, "intake");

        carousel = hardwareMap.get(DcMotor.class, "carousel");

        dropper = hardwareMap.get(Servo.class, "dropper");



        //intakeLifter  = hardwareMap.get(DcMotor.class, "lifter");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        rbDrive.setDirection(DcMotor.Direction.REVERSE);

        slider.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        //change this later ig
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        // intakeLifter.setDirection(DcMotor.Direction.FORWARD);
        //Dropper1.setPosition(1.0);

        dropper.setPosition(1.0);
    }


//    public void initVision(HardwareMap hardwareMap){
//        this.initVuforia();
//        this.initTfod(hardwareMap);
//    }

//    public void pickup(){
//        this.arm.setPower(0.4);
//        this.sleep(700);
//
//
//        this.Dropper1.setPosition(0.25);
//        this.Dropper2.setPosition(0.25);
//        this.Wrist.setPosition(0.6);
//        this.sleep(300);
//
//        this.sleep(250);
//
//        this.arm.setPower(-0.1);
//        this.sleep(100);
//        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        this.Dropper1.setPosition(0.0);
//        this.Dropper2.setPosition(0.0);
//        this.sleep(300);
//        this.arm.setPower(-0.6);
//        this.sleep(1000);
//        this.Wrist.setPosition(0.0);
//        this.sleep(300);
//    }

//    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
//    private static final String[] LABELS = {
//            "Ball",
//            "Cube",
//            "Duck",
//            "Marker"
//    };
    private static final String VUFORIA_KEY =
//            " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";
//            "AR4BaoH/////AAABmUfI9MVryEosoKpSgalFS9xaJ1QLidk13Y6d1uRhcd+USJBp9UCErESjjaqqRDiuhhF+dATZ1RinzpA4BeK3ogznKGKzd18DH7/1vOLhmJL2WH1iACJj5UytH6HoELaKMROQrCHKUSPamRT2617qldBngNtU+rjq3Wu6bxTTIU5aYIikuWKGi9K6XKwBQywcVMEBU1WbXkp2gUCMR8kLMP7mMRN0CalzcWu/PDa73t4wJeg4us6UZrUW7RcTR+FLZuYOEYZuhw0Ny0dLOkwtqCuleqMlF5veyp9U3QqZ4guCkkfUgE5vByNTHdoOiCXqE2J4ZfOKqHRrw54H4uOL4B44mp4Bkk/JcXWMLVPd4G7a";
"AVJZGR3/////AAABmW7vjrSEBUx3r+OoZNhGTHQkTNjH0B34dPXOah0FS0iHXW9QKzet6THKR+Sxxn7tIDrRE6yjTpxUBeB4xtpTgzCYlfDhWpNVZspxu1HmbAKRsL0xDDjQHSMWlbWj1LM/iSzNEXEcgP+34yWbzOP+5FY+z+GMshWOCL62lrc1DyHh/b0zsAXX3iBmbFgw4SQCnJQk+byrGhG1KR6j/kyyYHeCtdvGf7bsr0hBxt5EElKPb5pYkF8TmhnO8HJDpmmOSq1kc9pYFvDnJ8c8b6mcn//0YC/1735o3Dnd07QpD2yJqZcUE5HbaQcHO5mt0Fp3usgyJsN07n+nfTNQuW+Wd0AuShblCK8x+PR/ubBv26I/";
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

    public List<Recognition> detectObject(HardwareMap hardwareMap) {

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
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        List<Recognition> currentObject = new ArrayList<>();
//
        for(int i=0;i<2;i++){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                this.sleep(500);
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
                    currentObject = updatedRecognitions;
//                    telemetry.update();
                }
            }
            this.sleep(500);
        }
        return currentObject;
    }




    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = Webcam;

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
}

//

