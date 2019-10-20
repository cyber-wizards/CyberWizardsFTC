package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="UtsavDrive")
public class UtsavLinearProgram extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        DcMotor LeftFront;
        DcMotor LeftBack;
        DcMotor RightFront;
        DcMotor RightBack;
        double RightStickY = gamepad1.right_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double RightTrigger = gamepad1.right_trigger;
        double LeftTrigger = gamepad1.left_trigger;
        LeftFront = hardwareMap.dcMotor.get("");//Enter DC motor name
        LeftBack = hardwareMap.dcMotor.get("");//Enter DC motor name
        RightFront = hardwareMap.dcMotor.get("");//Enter DC motor name
        RightBack = hardwareMap.dcMotor.get("");//Enter DC motor name
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (RightTrigger > 0){
            LeftFront.setPower(-1);
            LeftBack.setPower(1);
            RightBack.setPower(-1);
            RightFront.setPower(1);
        }
        else if (LeftTrigger > 0){
            LeftFront.setPower(1);
            LeftBack.setPower(-1);
            RightBack.setPower(1);
            RightFront.setPower(-1);
        }
        else{
            LeftFront.setPower(LeftStickY);
            LeftBack.setPower(LeftStickY);
            RightBack.setPower(RightStickY);
            RightFront.setPower(RightStickY);
        }
    }
}
