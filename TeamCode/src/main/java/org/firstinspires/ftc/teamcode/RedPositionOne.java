package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


// ****READ QUALCOMM PACKAGE Docs


@Autonomous(name = "RedDuck" , group = "Main Autons")

public class RedPositionOne extends LinearOpMode {

    //Constants for the webcam width and height
    private static final int WEBCAM_WIDTH = 864;
    private static final int WEBCAM_HEIGHT = 480;

    //Webcam
    private OpenCvCamera webcam;

    // Try converting to DcMotorEx
    private DcMotorEx FL, FR, BL, BR, armMotor, flywheel , slides;
    private MotorsEx leftMotors , rightMotors , backMotors , frontMotors , allMotors;
    private Servo outtake;
    private IMU imu;
    private Orientation angles;
    private LynxModule lynx;

    // 548 ticks = 360 degrees (Without Load)
    // 580 tick = 360 degrees (With Load) = 12 inches
    // ~49 ticks = 30 degrees (With Load) = 1 inch
    final int INCH = 49;

    private final Async async = new Async(telemetry , this);
    private Differential bootleg;

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

        FR = hardwareMap.get(DcMotorEx.class, "FL"); // Port 1
        FL = hardwareMap.get(DcMotorEx.class, "FR"); // Port 2
        BL = hardwareMap.get(DcMotorEx.class, "BR"); // Port 0
        BR = hardwareMap.get(DcMotorEx.class, "BL"); // Port 3
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        outtake = hardwareMap.get(Servo.class, "outtake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        // Workout Voltage Sensor Configurations
        lynx = (LynxModule)hardwareMap.get(LynxModule.class, "Expansion Hub 1");

        leftMotors = new MotorsEx(FL , BL);
        rightMotors = new MotorsEx(FR , BR);
        backMotors = new MotorsEx(BL , BR);
        frontMotors = new MotorsEx(FL , FR);
        allMotors = new MotorsEx(FL , BL , FR , BR);

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
        backMotors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bootleg = new Differential(leftMotors , rightMotors , allMotors , this);
        //Just copy paste this part
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Also copy paste this
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Sets the pipeline
        PicturePipeline pipeline = new PicturePipeline();
        webcam.setPipeline(pipeline);

//-----------------------------------------------------------------------------
//---------------------------------AUTON START---------------------------------
//-----------------------------------------------------------------------------

        waitForStart();

        // testMotors((byte)1);
        // testMotors((byte)-1);
        /*
        async.perpetual(() -> {
            if(lynx.getInputVoltage(VoltageUnit.VOLTS) < 10 ||
                    lynx.getCurrent(CurrentUnit.AMPS) > 18.69) {
                allMotors.off();
            }
        });

*/

        int barcodeLevel = pipeline.getBarcodeLevel();
        webcam.stopStreaming();

        while(opModeIsActive()) {

            bootleg.differentialLeft(6 , 50 , -0.75);

            sleep(1000);

            encoderTurn( 900);

            customaryMove(4 , 0.5);

            flywheel.setPower(-0.6);
            sleep(3000);
            flywheel.setPower(0);

            customaryMove(-32 , -0.75);

            encoderTurn(-700);

            customaryMove(-4 , -0.6);


            slides.setTargetPosition(-3200);
            slides.setPower(0.9);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(4000);
            slides.setPower(0);
            outtake.setPosition(0.7);
            sleep(1000);
            outtake.setPosition(0.2);

            customaryMove(4 , 0.6);

            encoderTurn(650);

            slides.setTargetPosition(0);
            slides.setPower(0.4);
            slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(4000);
            slides.setPower(0);

            WEEEEEEEEEEEEEEE(3200);

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

        leftMotors.setPower(1.0  * negCorrect);
        rightMotors.setPower(-1.0  * negCorrect);

        frontMotors.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // while(frontMotors.isBusy()) allTelemetry();
        allTelemetry();
        sleep(800 + ((position * negCorrect)));

        allMotors.off();
        frontMotors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        allTelemetry();
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

        /**
         * TEST THIS AGAIN BUT CHANGE THE SETTIMEOUT TIME
         * TO THE FUNCTION USED TO CALCULATE SLEEP
         * This way we can talk about the tapered stop in logbook
         */
        /*
        async.setTimeout( () -> allMotors.off() , 5000 );
        while(frontMotors.isBusy()) {
            checkpoint = 3.9;
            allTelemetry();
            frontMotors.taperedStop();
            BL.setPower(FL.getPower());
            BR.setPower(FR.getPower());
            if(!allMotors.hasPower()) break;
        }
         */
        sleep(800 + (int)(Math.abs(Math.max(targetPositionLeft , targetPositionRight) / (2 / power))));

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
        telemetry.addData("Voltage" , lynx.getInputVoltage(VoltageUnit.VOLTS));
        telemetry.addData("FL Velocity" , FL.getVelocity());
        // telemetry.addData("Angle" , imu.getAngle());
        telemetry.addData("FL" , FL.getCurrentPosition());
        telemetry.addData("BL" , BL.getCurrentPosition());
        telemetry.addData("FR" , FR.getCurrentPosition());
        telemetry.addData("BR" , BR.getCurrentPosition());
        telemetry.addData("FL" , FL.getTargetPosition());
        telemetry.addData("BL" , BL.getTargetPosition());
        telemetry.addData("FR" , FR.getTargetPosition());
        telemetry.addData("BR" , BR.getTargetPosition());
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