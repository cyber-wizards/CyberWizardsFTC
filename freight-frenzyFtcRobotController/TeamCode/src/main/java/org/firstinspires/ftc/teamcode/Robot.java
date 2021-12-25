package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class Robot {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lfDrive = null;
    public DcMotor rfDrive = null;
    public DcMotor lbDrive = null;
    public DcMotor rbDrive = null;
    public DcMotor arm = null;
    public DcMotor carousel = null;
    public Servo Dropper1 = null;
    public Servo Dropper2 = null;
    public Servo Wrist = null;

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
        Dropper1  = hardwareMap.get(Servo.class, "dropper1");
        Dropper2 = hardwareMap.get(Servo.class, "dropper2");
        Wrist = hardwareMap.get(Servo.class, "wrist");


        arm  = hardwareMap.get(DcMotor.class, "arm");
        carousel  = hardwareMap.get(DcMotor.class, "carousel");

        //intakeLifter  = hardwareMap.get(DcMotor.class, "lifter");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        rbDrive.setDirection(DcMotor.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        // intakeLifter.setDirection(DcMotor.Direction.FORWARD);
        //Dropper1.setPosition(1.0);
    }

    public void pickup(){
        this.arm.setPower(0.4);
        this.sleep(700);


        this.Dropper1.setPosition(0.25);
        this.Dropper2.setPosition(0.25);
        this.Wrist.setPosition(0.6);
        this.sleep(300);

        this.sleep(250);

        this.arm.setPower(-0.1);
        this.sleep(100);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.Dropper1.setPosition(0.0);
        this.Dropper2.setPosition(0.0);
        this.sleep(300);
        this.arm.setPower(-0.6);
        this.sleep(1000);
        this.Wrist.setPosition(0.0);
        this.sleep(300);
    }

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
    }



