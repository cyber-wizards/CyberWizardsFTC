package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoPlace", group="Pushbot")
public class AutoPlace extends LinearOpMode {
    RobotOld robot   = new RobotOld();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.Dropper1.setPosition(0.0);
        robot.Dropper2.setPosition(0.0);
        robot.Wrist.setPosition(0.0);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rbDrive.getCurrentPosition(),
                robot.lbDrive.getCurrentPosition(),
                robot.lfDrive.getCurrentPosition(),
                robot.rfDrive.getCurrentPosition(),
                robot.arm.getCurrentPosition());


        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.arm.setPower(0.2);
        sleep(700);
//        robot.arm.setPower(-0.05);
//        sleep(100);
        robot.arm.setPower(0.0);
        robot.Wrist.setPosition(0.6);
        sleep(300);
        robot.Dropper1.setPosition(0.25);
        robot.Dropper2.setPosition(0.25);

        sleep(300);
        robot.arm.setPower(-0.6);
        sleep(150);

        robot.Dropper1.setPosition(0.0);
        robot.Dropper2.setPosition(0.0);
        sleep(300);


        robot.Wrist.setPosition(0.0);
        sleep(300);

        encoderDrive(0.3, -5, -5, -5, -5, 5);


        robot.arm.setPower(-0.6);
        sleep(550);

    }

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


            robot.lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            //  sleep(250);   // optional pause after each move
        }

    }
}
