package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Test", group="Iterative Opmode")
public class Test extends OpMode
{
    // Declare OpMode members.

    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor intakeLifter = null;
    // this is a comment

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        robot.lbDrive.setPower(gamepad1.left_stick_y);
        robot.lfDrive.setPower(gamepad1.left_stick_y);
        robot.rbDrive.setPower(gamepad1.left_stick_y);
        robot.rfDrive.setPower(gamepad1.left_stick_y);

        if (gamepad1.a) {
            robot.Dropper1.setPosition(0.0);
            telemetry.addData("Servo", "Dropper");
        } else {
            robot.Dropper1.setPosition(1.0);
        }

        if(gamepad1.b){
            robot.Dropper2.setPosition(0.0);
            telemetry.addData("Servo", "Wrist--");
        }else{
            robot.Dropper2.setPosition(1.0);
        }




        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
