package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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
    
    //Constants for the webcam width and height
    private static final int WEBCAM_WIDTH = 640;
    private static final int WEBCAM_HEIGHT = 480;
    
    //Webcam
    private OpenCvCamera webcam;

    private DcMotor FL, FR, BL, BR, armMotor, flywheel , slides;
    private Motors leftMotors , rightMotors , backMotors , frontMotors , allMotors;
    private Servo outtake;
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
        FR = hardwareMap.get(DcMotor.class, "FL"); // Port 1
        FL = hardwareMap.get(DcMotor.class, "FR"); // Port 2
        BL = hardwareMap.get(DcMotor.class, "BR"); // Port 0
        BR = hardwareMap.get(DcMotor.class, "BL"); // Port 3
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        slides = hardwareMap.get(DcMotor.class, "slides");
        outtake = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        
        leftMotors = new Motors(FL , BL);
        rightMotors = new Motors(FR , BR);
        backMotors = new Motors(BL , BR);
        frontMotors = new Motors(FL , FR);
        allMotors = new Motors(FL , BL , FR , BR);

        // FORWARD = positive = forward (right)
        // REVERSE = negative = forward (right)
        leftMotors.setDirection(DcMotor.Direction.REVERSE);
        rightMotors.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        slides.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        allMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        allMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//-----------------------------------------------------------------------------
//---------------------------------AUTON START---------------------------------
//-----------------------------------------------------------------------------
/*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Sets the pipeline
        PicturePipeline pipeline = new PicturePipeline();
        webcam.setPipeline(pipeline);
*/
        waitForStart();
        
        // testMotors((byte)1);
        // testMotors((byte)-1);
        
        while(opModeIsActive()) {
            
            
            // telemetry.addData("Position" , pipeline.getBarcodeLevel());
            // telemetry.update();
            
            
            //encoderTurn(200);
            move(0.5 , 900);
            
            flywheel.setPower(-0.6);
            sleep(3000);
            flywheel.setPower(0);
            
            encoderTurn(-200);
            move(-1.0 , 800);
            encoderTurn(-500);
            
            allMotors.setPower(-0.5);
            sleep(500);
            allMotors.off();
            
            slide(2);
            outtake.setPosition(0.7);
            sleep(800);
            outtake.setPosition(0.175);
            // slide(-slides.getCurrentPosition());
            
            allMotors.setPower(0.5);
            sleep(200);
            encoderTurn(520);
            WEEEEEEEEEEEEEEE(2400);
            slide(-slides.getCurrentPosition());
            
            while(opModeIsActive()) {}
            
            idle();
            /*
            slides level 3: 
            slides.setPower(0.5);
            sleep(600);
            slides level 2: 
            slides.setPower(0.5);
            sleep(400);
            lides level 1: 
            slides.setPower(0.5);
            sleep(200);
            */
        }
        idle();
    }
    
//-----------------------------------------------------------------------------
//---------------------------------AUTON END-----------------------------------
//-----------------------------------------------------------------------------
    
/**
 * encoderTurn - turns the robot the specified degrees
 * (based on 90 degrees = +/-600 ticks on each motor)
 * positive degrees turns clockwise
 * 
 * Values because I'm too lazy to make a correction function right now
 * 300 = ~45 degrees
 * 600 = ~90 degrees
 * 1300 = ~180 degrees
 * 2700 = ~360 degrees
 * 
 * @param int degrees
 */
    private void slide(int CVpos) {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (CVpos == 2) {
            slides.setTargetPosition(-3200); //Test
        }
        else if (CVpos == 1) {
            slides.setTargetPosition(-2400); //Test
        }
        else if (CVpos == 0) {
            slides.setTargetPosition(-1200); //Test
        }else {
            armMotor.setPower(0);
            slides.setTargetPosition(CVpos);
        }
        slides.setPower(0.8);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while(slides.isBusy()) {
                telemetry.addData("Slides" , slides.getCurrentPosition());
                telemetry.update();
        }
            
        slides.setPower(0);
        
        allTelemetry();
        sleep(200);
    }
    private void encoderTurn(int position) {
        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        int negCorrect = position < 0 ? -1 : 1;
        
        FR.setTargetPosition(-position);
        FL.setTargetPosition(position);
            
        leftMotors.setPower(1.0  * negCorrect);
        rightMotors.setPower(-1.0  * negCorrect);
            
        frontMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // while(frontMotors.isBusy()) allTelemetry();
        allTelemetry();
        sleep(800 + ((position * negCorrect) / 2));
            
        allMotors.off();
        allMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        allTelemetry();
        sleep(200);
    }
    
/**
 * testMotors - runs each motor one at a time
 * Hardcoded to ensure that the Motors class or encoders aren't the problem
 */
    private void testMotors(byte change) {
        /*
        for(DcMotor motor : allMotors.getMotors()) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(change * 0.5);
            allTelemetry();
            sleep(1000);
            motor.setPower(0);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        allTelemetry();
        */
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        FL.setPower(change * 0.5);
        allTelemetry();
        sleep(1000);
        FL.setPower(0);
        sleep(500);
        
        BL.setPower(change * 0.5);
        allTelemetry();
        sleep(1000);
        BL.setPower(0);
        sleep(500);
        
        FR.setPower(change * 0.5);
        allTelemetry();
        sleep(1000);
        FR.setPower(0);
        sleep(500);
        
        BR.setPower(change * 0.5);
        allTelemetry();
        sleep(1000);
        BR.setPower(0);
        sleep(500);
        
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        allTelemetry();
    }
    
    private void allTelemetry() {
        telemetry.addData("Angle" , imu.getAngle());
        telemetry.addData("FL" , FL.getCurrentPosition());
        telemetry.addData("BL" , BL.getCurrentPosition());
        telemetry.addData("FR" , FR.getCurrentPosition());
        telemetry.addData("BR" , BR.getCurrentPosition());
        telemetry.addData("FL Power" , FL.getPower());
        telemetry.addData("BL Power" , BL.getPower());
        telemetry.addData("FR Power" , FR.getPower());
        telemetry.addData("BR Power" , BR.getPower());
        telemetry.update();
    }
    private boolean isAtAngle(int targetAngle) {
        int angle = (int)Math.abs(imu.normalizeAngle(targetAngle));
        return Math.abs(imu.getAngle()) >= angle - 1;
    }
    private void move(double power , int time) {
        allMotors.setPower(power);
        sleep(time);
        allMotors.off();
    }
    private void WEEEEEEEEEEEEEEE(int time) {
        allMotors.setPower(-0.6);
        sleep(time);
        allMotors.off();
    }
}
