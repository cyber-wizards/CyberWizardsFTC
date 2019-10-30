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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
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
//Rishi's butt waz here
@Autonomous(name="AutonomousTest", group="Linear Opmode")
//@Disabled
public class AutonomousTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor downleft;
    DcMotor downright;
    Servo FoundationGrabber1;
    Servo FoundationGrabber2;
    DigitalChannel digitalTouch;

        double power = 1.0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        downleft = hardwareMap.get(DcMotor.class, "downleft");
        downright = hardwareMap.get(DcMotor.class, "downright");
        FoundationGrabber1 = hardwareMap.servo.get("FoundationGrabber1");
        FoundationGrabber2 = hardwareMap.servo.get("FoundationGrabber2");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        downleft.setDirection(DcMotor.Direction.FORWARD);
        downright.setDirection(DcMotor.Direction.REVERSE);

        FoundationGrabber1.setPosition(1.0);
        FoundationGrabber2.setPosition(0.0);


        if (digitalTouch.getState() == true) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
            frontleft.setPower(power);
            frontright.setPower(power);
            downleft.setPower(power);
            downright.setPower(power);
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
            frontleft.setPower(0.0);
            frontright.setPower(0.0);
            downleft.setPower(0.0);
            downright.setPower(0.0);
            sleep(1000);
            FoundationGrabber1.setPosition(0.0);
            FoundationGrabber2.setPosition(1.0);
            sleep(1000);
            frontleft.setPower(-power);
            frontright.setPower(-power);
            downleft.setPower(-power);
            downright.setPower(-power);
            sleep(5000);


        }




        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

    }
}
