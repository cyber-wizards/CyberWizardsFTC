package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;


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
}
