package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



// ****READ QUALCOMM PACKAGE Docs


@Autonomous

public class TestAuto extends LinearOpMode {


    private DcMotor FL, FR, BL, BR, armMotor, flywheel , slides;
    private Motors leftMotors , rightMotors , backMotors , frontMotors , allMotors;
    private Servo outtake;
    private IMU imu;
    private Orientation angles;

    // 548 ticks = 360 degrees (Without Load)
    // 580 tick = 360 degrees (With Load) = 12 inches
    // ~49 ticks = 30 degrees (With Load) = 1 inch
    final int INCH = 49;

    Async async = new Async(telemetry , this);

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

        FR = hardwareMap.get(DcMotor.class, "FL"); // Port 1
        FL = hardwareMap.get(DcMotor.class, "FR"); // Port 2
        BL = hardwareMap.get(DcMotor.class, "BR"); // Port 0
        BR = hardwareMap.get(DcMotor.class, "BL"); // Port 3
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        slides = hardwareMap.get(DcMotor.class, "slides");
        outtake = hardwareMap.get(Servo.class, "outtake");
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
        allMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//-----------------------------------------------------------------------------
//---------------------------------AUTON START---------------------------------
//-----------------------------------------------------------------------------

        waitForStart();

        // testMotors((byte)1);
        // testMotors((byte)-1);

        while(opModeIsActive()) {

            //encoderTurn(200);
            move(0.5 , 900);

            flywheel.setPower(-0.6);
            sleep(3000);
            flywheel.setPower(0);

            encoderTurn(-100);
            move(-1.0 , 1000);
            encoderTurn(-600);

            // Fill Slides Auto

            encoderTurn(500);
            WEEEEEEEEEEEEEEE(1500);

            while(opModeIsActive()) {}

            idle();
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
     * @param //int degrees
     */
    private void encoderTurn(int position) {
        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int negCorrect = position < 0 ? -1 : 1;

        FR.setTargetPosition(-position);
        FL.setTargetPosition(position);

        leftMotors.setPower(negCorrect);
        rightMotors.setPower(negCorrect);

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
     * Moves forward based on the targetPositions passed using encoders
     * Moves both motors with the passed power
     *
     * @param targetPositionLeft - Left Motor Target Position
     * @param targetPositionRight - Right Motor Target Position
     * @param power - Motor Power
     */
    private void encoderMove(int targetPositionLeft,
                             int targetPositionRight,
                             double power) {
        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setTargetPosition(targetPositionLeft);
        FR.setTargetPosition(targetPositionRight);

        allMotors.setPower(power);
        frontMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        async.setTimeout(() -> allMotors.off() , 5000);
        while(frontMotors.isBusy()) {
            allTelemetry();
            frontMotors.taperedStop();
            BL.setPower(FL.getPower());
            BR.setPower(FR.getPower());
        }

        allMotors.off();
        allTelemetry();

        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void customaryMove(double inches , double power) {
        int inchesToTicks = (int)Math.round(inches * INCH);
        encoderMove(inchesToTicks , inchesToTicks , power);
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
        allMotors.setPower(-1.0);
        sleep(time);
        allMotors.off();
    }
}