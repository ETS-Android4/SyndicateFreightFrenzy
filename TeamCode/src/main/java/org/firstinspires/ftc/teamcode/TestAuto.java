package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.*;
import java.util.concurrent.*;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;

// ****READ QUALCOMM PACKAGE Docs

@Autonomous

public class TestAuto extends LinearOpMode {

    //CHANGE THESE DURING TEAM BRIEF WITH ALLIANCE MEMBER
    //SHOULD ALLOW FOR SOME FLEXIBILITY DEPENDING ON THEIR AUTONOMOUS
    //MAY SCRAP THIS IDEA
    private StartPosition startPos = StartPosition.BLUE_DEPOT;
    private ParkLocation parkLocation = ParkLocation.DEPOT;
    private boolean spinCarousel = true;
    private boolean placeBlockOnShipping = true;

    private DcMotor FL, FR, BL, BR, armMotor, flywheel , slides;
    private Servo gripperServo;
    private IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        //IMU setup. Sets the necessary parameters. Using degrees, not radians!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Is this built in? Hopefully
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(hardwareMap);
        //imu.initialize(parameters);

        // 548 ticks -> 360 degrees
        FL = hardwareMap.get(DcMotor.class, "FL"); // Port 1
        FR = hardwareMap.get(DcMotor.class, "FR"); // Port 2
        BR = hardwareMap.get(DcMotor.class, "BR"); // Port 0
        BL = hardwareMap.get(DcMotor.class, "BL"); // Port 3
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        slides = hardwareMap.get(DcMotor.class, "slides");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        
        Motors leftMotors = new Motors(FL , BL);
        Motors rightMotors = new Motors(FR , BR);
        Motors backMotors = new Motors(BL , BR);
        Motors frontMotors = new Motors(FL , FR);
        Motors allMotors = new Motors(FL , BL , FR , BR);

        // FORWARD = positive = forward (right)
        // REVERSE = negative = forward (right)
        leftMotors.setDirection(DcMotor.Direction.REVERSE);
        rightMotors.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        slides.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        allMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        allMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // testMotors((byte)1);
        // testMotors((byte)-1);
        
        //590 = 90 DEGREES PROBABLY
        while(opModeIsActive()) {
            BR.setTargetPosition(590*4);
            BL.setTargetPosition(-590*4);
            
            leftMotors.setPower(-1.0);
            rightMotors.setPower(1.0);
            
            backMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(isAtAngle(90) || backMotors.isBusy()) {
                telemetry.addData("Angle" , imu.getAngle());
                telemetry.addData("FL" , FL.getCurrentPosition());
                telemetry.addData("BL" , BL.getCurrentPosition());
                telemetry.addData("FR" , FR.getCurrentPosition());
                telemetry.addData("BR" , BR.getCurrentPosition());
                telemetry.update();
            }
            
            allMotors.off();
            
            while(opModeIsActive()) {}
        }
        // idle();
    }
    
    private void turn(double degree) {
        R.setTargetPosition(degree*6);
            BL.setTargetPosition(degree*6*-1);
            
            leftMotors.setPower(-1.0);
            rightMotors.setPower(1.0);
            
            backMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(isAtAngle(degree*6) || backMotors.isBusy()) {
                telemetry.addData("Angle" , imu.getAngle());
                telemetry.addData("FL" , FL.getCurrentPosition());
                telemetry.addData("BL" , BL.getCurrentPosition());
                telemetry.addData("FR" , FR.getCurrentPosition());
                telemetry.addData("BR" , BR.getCurrentPosition());
                telemetry.update();
            }
    }
    
    private void testMotors(byte change) {
        FL.setPower(change * 0.5);
        readTelemetryMotorPower();
        sleep(1000);
        FL.setPower(0);
        
        BL.setPower(change * 0.5);
        readTelemetryMotorPower();
        sleep(1000);
        BL.setPower(0);
        
        FR.setPower(change * 0.5);
        readTelemetryMotorPower();
        sleep(1000);
        FR.setPower(0);
        
        BR.setPower(change * 0.5);
        readTelemetryMotorPower();
        sleep(1000);
        BR.setPower(0);
        readTelemetryMotorPower();
    }
    
    private void readTelemetryMotorPower() {
        telemetry.addData("FL" , FL.getPower());
        telemetry.addData("BL" , BL.getPower());
        telemetry.addData("FR" , FR.getPower());
        telemetry.addData("BR" , BR.getPower());
        telemetry.update();
    }
    private boolean isAtAngle(int targetAngle) {
        int angle = (int)Math.abs(imu.normalizeAngle(targetAngle));
        return Math.abs(imu.getAngle()) >= angle - 1;
    }
}
