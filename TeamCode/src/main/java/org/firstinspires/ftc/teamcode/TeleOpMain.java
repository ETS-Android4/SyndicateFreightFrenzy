package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class TeleOpMain extends LinearOpMode {
    DcMotor FL, FR, BL, BR, arm, flywheel;
    Servo gripper;
    private boolean directionState;


    @Override
    public void runOpMode() {
        
        intialize();
        
        waitForStart();

        while(opModeIsActive()) {
            mainDriveControl();
            microDrive();
            armControl();
            gripperControl();
            flywheelControl();
        }
    }
    void initialize() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper = hardwareMap.get(Servo.class ,"gripper");
        
        arm = hardwareMap.get(DcMotor.class, "arm");
        
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

    }
    void mainDriveControl() {

        double LY = gamepad1.left_stick_y;
        double RY = gamepad1.right_stick_y;

        FL.setPower(LY);
        FR.setPower(RY);
        BL.setPower(LY);
        BR.setPower(RY);

    }
    void microDrive() {
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
    void armControl() {
        arm.setPower(Range.clip(-gamepad2.left_stick_y, -1, .6));
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
}
