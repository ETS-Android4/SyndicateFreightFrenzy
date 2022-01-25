package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleOpMain extends LinearOpMode {
    DcMotor FL, FR, BL, BR, arm, flywheel, slides;
    Servo gripper, outtake;
    private boolean directionState;


    public void initialize() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setTargetPosition(0);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides = hardwareMap.get(DcMotor.class, "slides");
        outtake = hardwareMap.get(Servo.class, "outtake");
        gripper = hardwareMap.get(Servo.class ,"gripper");
        arm = hardwareMap.get(DcMotor.class, "arm");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        while(opModeIsActive()) {
            driveControl();
            dpadDrive();
            slidesControl();
            armControl();
            gripperControl();
            flywheelControl();
            outtakeControl();
            intakeControl();
        }
    }
    void intakeControl() {
        if (gamepad2.a) {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setTargetPosition(1500) //experiment with value
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(0.5);
            while (intake.isBusy()) {
                telemetry.addData("Target Position", 1500);
                telemtry.addData("Current Position", intake.getCurrentPosition());
                telemtry.update();           
            }
        }
        if (gamepad2.b) {
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setTargetPosition(750) //experiment with value
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setPower(-0.5);
            while (intake.isBusy()) {
                telemetry.addData("Target Position", 750);
                telemtry.addData("Current Position", intake.getCurrentPosition());
                telemtry.update();   
            }
        }
    }
    void driveControl() {

        double LY = gamepad1.left_stick_y;
        double RY = gamepad1.right_stick_y;
        LY = Range.clip(LY, -0.75, 0.75);
        RY = Range.clip(RY, -0.75, 0.75);
        FL.setPower(LY);
        FR.setPower(RY);
        BL.setPower(LY);
        BR.setPower(RY);

    }
    void dpadDrive() {
        if (gamepad1.dpad_up) {
            FL.setPower(-.4);
            FR.setPower(-.4);
            BL.setPower(-.4);
            BR.setPower(-.4);
        } else if (gamepad1.dpad_down) {
            FL.setPower(.4);
            FR.setPower(.4);
            BL.setPower(.4);
            BR.setPower(.4);
        } else if (gamepad1.dpad_left) {
            FL.setPower(.4);
            FR.setPower(-.4);
            BL.setPower(.4);
            BR.setPower(-.4);
        } else if (gamepad1.dpad_right) {
            FL.setPower(-.4);
            FR.setPower(.4);
            BL.setPower(-.4);
            BR.setPower(.4);
        }
    }
    void slidesControl() {
        slides.setPower(Range.clip(-gamepad2.left_stick_y, -1, .6));
    }
    void armControl() {
        if (gamepad2.dpad_up) {
            arm.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            arm.setPower(-1);
        }
    }
    
    void gripperControl() {
        int dpad_right_count = 0; 
        directionState = false;      
        if(directionState == false){
            if(gamepad2.dpad_right){
                dpad_right_count += 1;
                directionState = true;

                gripper.setPosition(0.435);
            }
        }
        else {
            if(gamepad2.dpad_right){
                dpad_right_count += 1;
                directionState = false;

                gripper.setPosition(0.915);
            }
        } 
        telemetry.addData("Direction State:", directionState);
        telemetry.addData("dpad_right_count:", dpad_right_count);
    }
    void flywheelControl() {
        int dpad_left_count = 0; 
        directionState = false;      
        if(directionState == false){
            if(gamepad2.dpad_left){
                dpad_left_count += 1;
                directionState = true;

                flywheel.setPower(0.2);
            }
        }
        else {
            if(gamepad2.dpad_left){
                dpad_left_count += 1;
                directionState = false;

                flywheel.setPower(0);
            }
        } 
        telemetry.addData("Direction State:", directionState);
        telemetry.addData("dpad_left_count:", dpad_left_count);
    }
    void outtakeControl(){
        boolean bumperPressed = false;
        if(gamepad2.right_bumper) {
            bumperPressed = true;
            directionState = true;
            gripper.setPosition(0.435);
        }
        else {
            gripper.setPosition(0.915);
        }
        telemetry.addData("bumperPressed? ", bumperPressed);
    }
}
