package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class TeleOpMain extends LinearOpMode {
    DcMotor FL, FR, BL, BR, flywheel, slides, intake, arm;
    Servo outtake, hook;
    private boolean directionState;
    double LY = gamepad1.right_stick_y;
    double RY = gamepad1.left_stick_y;


    public void initialize() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        slides = hardwareMap.get(DcMotor.class, "slides");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(Servo.class, "gripper");
        hook = hardwareMap.get(Servo.class, "hook");
        arm = hardwareMap.get(DcMotor.class, "arm");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    @Override
    public void runOpMode() {

        initialize();

        waitForStart();
        /*
        slides.setTargetPosition(-3200);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(0.8);
        while(slides.isBusy()) {}
        slides.setPower(0);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
        while(opModeIsActive()) {
            driveControl();
            dpadDrive();
            slidesControl();
            armControl();
            //gripperControl();
            flywheelControl();
            outtakeControl();
            telemetry.addData("Slides" , slides.getCurrentPosition());
            intakeControl();
        }
    }
    void driveControl() {
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
        
    }
    void armControl() {
        if (gamepad1.dpad_up) {
            arm.setPower(-0.4);
        } 
        else if (gamepad1.dpad_down) {
            arm.setPower(0.4);
        }
        else {
            arm.setPower(0);
        }
           
    }
    void hookControl() {
        //trying to find a button to control this thing
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
        if(gamepad2.y) {
            slides.setTargetPosition(-3200);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(0.8);
            while(slides.isBusy()) {
                telemetry.addData("Slides" , slides.getTargetPosition());
                telemetry.update();
                FL.setPower(LY);
                FR.setPower(RY);
                BL.setPower(LY);
                BR.setPower(RY);
            }
            slides.setPower(0);
        }
        if(gamepad2.x) {
            slides.setTargetPosition(0);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slides.setPower(0.6);
            while(slides.isBusy()) {
                telemetry.addData("Slides" , slides.getTargetPosition());
                telemetry.update();
                FL.setPower(LY);
                FR.setPower(RY);
                BL.setPower(LY);
                BR.setPower(RY);
            }
            slides.setPower(0);
        }
        /*
        if (gamepad2.left_stick_y != 0) {
            slides.setPower(0);
        }
        slides.setPower(Range.clip(gamepad2.left_stick_y, -1.0, 1.0));*/
        /*with encoders
        slides.set
        */
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
                flywheel.setPower(-0.65);
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
            outtake.setPosition(0.175);
        }
        
        else if (gamepad2.left_bumper)
        {
            bumperPressed = true;
            directionState = true;
            outtake.setPosition(0.7);
            sleep(200);
            outtake.setPosition(0.175);
        }
        telemetry.addData("bumperPressed? ", bumperPressed);
    }
}
