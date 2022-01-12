package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpProgram", group="Linear Opmode")
public class TeleOpProgram extends LinearOpMode
{

// Declare OpMode members.

    Robot robot = new Robot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        double xNew = 1.0;

        // this is a comment

        /*
         * Code to run ONCE when the driver hits INIT
         */

        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);
        robot.lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).




        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        boolean gamepad2XIsPressed = false;
        boolean slow = false;
        String currentMode = "Speed";

//    public enum GrabState {
//        GRAB_START,
//        GRAB_MOVE_DOWN,
//        LIFT_DUMP,
//        LIFT_RETRACT
//    };
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double lfPower;
            double lbPower;
            double rfPower;
            double rbPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double x = 1.0;


            //  boolean wasPressed = gamepad1.right_bumper && !bumperWasHeld;
//
//        if(wasPressed){
//
////            if(slow){
////                x*=3;
////                slow = !slow;
////                currentMode = "Speed";
////            }else{
////                x/=3;
////                slow = !slow;
////                currentMode = "Precision";
////            }
//
//            currentMode = "Very Amogus";
//
//            if(slow){
//                slow = false;
//                x=1.0;
//                currentMode = "Speed";
//            }else{
//                slow = true;
//                x/=3;
//                currentMode="Precision";
//            }
//        }else{
//            currentMode = "Not Amogus";
//        }
//
//        bumperWasHeld = gamepad1.right_bumper;

            if (gamepad1.right_bumper) {
                x /= 3;
                currentMode = "Precision";
            } else {
                x = 1.0;
                currentMode = "Sped. I am sped";
            }

            if (gamepad1.left_stick_y < 0) {
                robot.lbDrive.setPower(-x);
                robot.rbDrive.setPower(-x);
                robot.lfDrive.setPower(-x);
                robot.rfDrive.setPower(-x);
            } else if (gamepad1.left_stick_y > 0) {
                robot.lbDrive.setPower(x);
                robot.rbDrive.setPower(x);
                robot.lfDrive.setPower(x);
                robot.rfDrive.setPower(x);
            } else if (gamepad1.left_trigger > 0) {
                robot.lbDrive.setPower(x);
                robot.rbDrive.setPower(-x);
                robot.lfDrive.setPower(-x);
                robot.rfDrive.setPower(x);
            } else if (gamepad1.right_trigger > 0) {
                robot.lbDrive.setPower(-x);
                robot.rbDrive.setPower(x);
                robot.lfDrive.setPower(x);
                robot.rfDrive.setPower(-x);
            } else if (gamepad1.right_stick_x < 0) {
                robot.lbDrive.setPower(-x);
                robot.rbDrive.setPower(x);
                robot.lfDrive.setPower(-x);
                robot.rfDrive.setPower(x);
            } else if (gamepad1.right_stick_x > 0) {
                robot.lbDrive.setPower(x);
                robot.rbDrive.setPower(-x);
                robot.lfDrive.setPower(x);
                robot.rfDrive.setPower(-x);
            } else {
                robot.lbDrive.setPower(0.0);
                robot.rbDrive.setPower(0.0);
                robot.lfDrive.setPower(0.0);
                robot.rfDrive.setPower(0.0);
            }

//        double y = gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x;
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio, but only when
//        // at least one is out of the range [-1, 1]
//
//        if(Math.abs(x)>0.0 && (Math.abs(y)<Math.abs(0.5))){
//            if(x<0){
//            robot.lbDrive.setPower(xNew);
//            robot.rbDrive.setPower(-xNew);
//            robot.lfDrive.setPower(-xNew);
//            robot.rfDrive.setPower(xNew);
//        }else if(x>0){
//            robot.lbDrive.setPower(-xNew);
//            robot.rbDrive.setPower(xNew);
//            robot.lfDrive.setPower(xNew);
//            robot.rfDrive.setPower(-xNew);
//        }
//        }
//        else{
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//            robot.lfDrive.setPower(frontLeftPower);
//            robot.lbDrive.setPower(backLeftPower);
//            robot.rfDrive.setPower(frontRightPower);
//            robot.rbDrive.setPower(backRightPower);
//        }

            if (gamepad2.x && !gamepad2XIsPressed) {
                telemetry.addData("tes", "bueno");

//                robot.arm.setPower(0.4);
//                robot.sleep(700);
//
//
//                robot.Dropper1.setPosition(0.25);
//                robot.Dropper2.setPosition(0.25);
//                robot.Wrist.setPosition(0.6);
//                robot.sleep(300);
//
//                robot.sleep(250);
//
//                robot.arm.setPower(-0.1);
//                robot.sleep(100);
//                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//                robot.Dropper1.setPosition(0.0);
//                robot.Dropper2.setPosition(0.0);
//                robot.sleep(300);
//                robot.arm.setPower(-0.6);
//                robot.sleep(850);
//                robot.Wrist.setPosition(0.0);
//                robot.sleep(300);
                robot.pickup();




//            robot.Dropper1.setPosition(0.0);
//            robot.Dropper2.setPosition(0.0);
//
//            robot.sleep(100);
//
//            robot.sleep(100);

            } else {
                telemetry.addData("no", "bueno");
            }


            robot.arm.setPower((gamepad2.right_trigger / 1.5) - (gamepad2.left_trigger / 1.5));
            //robot.carousel.setPower(((gamepad2.right_bumper) ? 1.0: 0.0)-((gamepad2.left_bumper) ? -1.0: 0.0));
            if (gamepad2.dpad_right) {
                robot.carousel.setPower(1.0);
            } else {
                robot.carousel.setPower(0.0);
            }

            if (gamepad2.dpad_left) {
                robot.carousel.setPower(-1.0);
            } else {
                robot.carousel.setPower(0.0);
            }

            //todo monkey
            if(gamepad2.a){
                robot.Dropper1.setPosition(0.25);
                robot.Dropper2.setPosition(0.25);
            }else {
                robot.Dropper1.setPosition(0.0);
                robot.Dropper2.setPosition(0.0);
            }


            if (gamepad2.right_bumper) {
                robot.Wrist.setPosition(0.6);

            } else {
                robot.Wrist.setPosition(0.0);

            }


            gamepad2XIsPressed = gamepad2.x;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Mode:", currentMode);

        }
    };}
