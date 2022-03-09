package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleOpMain extends LinearOpMode {
    DcMotor FL, FR, BL, BR, arm, flywheel, slides, intake;
    Servo outtake, hook;
    private boolean directionState;
    int slideStartPosition;
    int hookCounter;


    public void initialize() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        slides = hardwareMap.get(DcMotor.class, "slides");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(Servo.class, "outtake");
        // = hardwareMap.get(Servo.class ,"gripper");
        arm = hardwareMap.get(DcMotor.class, "arm");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        hook = hardwareMap.get(DcMotor.class, "hook");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    @Override
    public void runOpMode() {

        initialize();

        waitForStart();

        slideStartPosition = slides.getCurrentPosition();

        while(opModeIsActive()) {
            driveControl();
            dpadDrive();
            slidesControl();
            armControl();
            //gripperControl();
            flywheelControl();
            outtakeControl();
            //intakeControl();
        }
    }
    void driveControl() {

        double LY = gamepad1.right_stick_y;
        double RY = gamepad1.left_stick_y;
        telemetry.addData("left", LY);
        telemetry.addData("right", RY);
        telemetry.update();
        LY = Range.clip(LY, -0.75, 0.75);
        RY = Range.clip(RY, -0.75, 0.75);
        FL.setPower(LY);
        FR.setPower(RY);
        BL.setPower(LY);
        BR.setPower(RY);

    }
    /*
    void intakeControl() {
        if (gamepad2.a) {
            intake.setPower(1);
        }
        else if (gamepad2.b) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

    }*/
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
            FL.setPower(.6);
            FR.setPower(-.6);
            BL.setPower(.6);
            BR.setPower(-.6);
        } else if (gamepad1.dpad_right) {
            FL.setPower(-.6);
            FR.setPower(.6);
            BL.setPower(-.6);
            BR.setPower(.6);
        }
    }
    void slidesControl() {
        telemetry.addData("Updated" , 6);
        telemetry.addData("Slides Position" , slides.getCurrentPosition());
        telemetry.addData("Slide Start Position" , slideStartPosition);

        // -0.7 is up | 0.4 is down
        double slidePower = Range.clip(gamepad2.left_stick_y, -0.7, 0.4);
        telemetry.addData("slidePower1" , slidePower);

        if((slidePower < 0 && slides.getCurrentPosition() < slideStartPosition - 3200) ||
                (slidePower > 0 && slides.getCurrentPosition() > slideStartPosition - 20))
            slidePower = 0;

        telemetry.addData("slidePower2" , slidePower);
        slides.setPower(slidePower);
        telemetry.addData("Slides Power" , slides.getPower());
    }

    void armControl() {
        if (gamepad2.dpad_up) {
            arm.setPower(0.4);
        }
        else if (gamepad2.dpad_down) {
            arm.setPower(-0.1);
        }
        else{
            arm.setPower(0);
        }
    }
    void hookControl() {
        if (gamepad2.b) {
            if (hookCounter == 1) {
                hook.setPosition(0.7);
                hookCounter--;
            }
            else {
                hook.setPosition(-0.4);
                hookCounter++;
            }
        }
    }
    /*
    void gripperControl() {
        telemetry.addData("gripper", 1);
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
    }*/
    void flywheelControl() {
        int dpad_left_count = 0;
        if(gamepad2.dpad_left){
            dpad_left_count += 1;
            flywheel.setPower(-0.5);
        }

        if(gamepad2.dpad_right){
            dpad_left_count += 1;

            flywheel.setPower(0);
        }
        telemetry.addData("Direction State:", directionState);
        telemetry.addData("dpad_left_count:", dpad_left_count);
    }
    void outtakeControl(){
        telemetry.addData("bumpers", 1);
        boolean bumperPressed = false;
        if(gamepad2.right_bumper) {


            bumperPressed = true;
            directionState = true;
            outtake.setPosition(0.7);
        }

        else if (gamepad2.left_bumper)
        {
            bumperPressed = true;
            directionState = true;
            outtake.setPosition(-0.4);
        }
        telemetry.addData("bumperPressed? ", bumperPressed);
    }
}
