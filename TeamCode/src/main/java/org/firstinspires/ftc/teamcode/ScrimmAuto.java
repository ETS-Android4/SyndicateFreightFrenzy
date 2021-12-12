//PLEASE DO NOT USE THIS OPMODE


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous
public class ScrimmAuto extends LinearOpMode {
    /*
     * Syndicate Autonomous Strategy 1: move to depot
     * Authors: Harish Varadarajan
     * Purpose: autonomous program for FTC Freight Frenzy: park in the depot for 10 points
 30-second Autonomous period
 */
    private BNO055IMU imu;
    private Orientation angles;
    //Iteration 1
    private ElapsedTime runtime = new ElapsedTime();
    // declare motors for mapping
    private static DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor flywheel;
    //encoder values
    private static final double wheelRadius = 3, length = 18, width = 18, ticksPerRev = 1440;
    private static final double ticksPerInches = (ticksPerRev)/(2 * Math.PI * wheelRadius), turnRadius = Math.sqrt(Math.pow((double) length / 2, 2) + Math.pow((double) width / 2,2 ));
    private static final double circumference = (2 * Math.PI * turnRadius);
    //if this method does not work, tell me
    /**
     * Drives forward until stopped
     * @param power Power of the motors. 0.0-1.0
     */
    void driveForward(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Drives forward the specified amount of ticks
     * @param power Power of the motors. 0.0-1.0
     * @param ticks Amount of ticks to drive.
     */
    void driveForward(double power, double ticks) {
        driveForward(power);
        while(frontLeft.getCurrentPosition() < ticks && frontRight.getCurrentPosition() < ticks && backLeft.getCurrentPosition() < ticks && backRight.getCurrentPosition() < ticks) {}
        stopMotors();
        resetEncoders();
    }
    void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Resets all drivetrain encoders
     */
    void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    private void drive (double power, int inchOrDeg, String isInchOrDeg) {
        /* drive forward/backward (negative inches is driving backward) */
        if (isInchOrDeg.equals("inchForward")) {
            frontLeft.setTargetPosition((int) ticksPerInches * inchOrDeg);
            backLeft.setTargetPosition((int) ticksPerInches * inchOrDeg);
            frontRight.setTargetPosition((int) ticksPerInches * inchOrDeg);
            backRight.setTargetPosition((int) ticksPerInches * inchOrDeg);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
            while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy());
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        else if (isInchOrDeg.equals("leftDegree")){
            frontLeft.setTargetPosition((-inchOrDeg/360) * ((int) circumference));
            backLeft.setTargetPosition((-inchOrDeg/360) * ((int) circumference));
            frontRight.setTargetPosition((-inchOrDeg/360) * ((int) circumference));
            backRight.setTargetPosition((-inchOrDeg/360) * ((int) circumference));
            // turn left IN PLACE
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(-power);
            while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy());
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        else if (isInchOrDeg.equals("rightDegree")){
            frontLeft.setTargetPosition((inchOrDeg/360) * ((int) circumference));
            backLeft.setTargetPosition((inchOrDeg/360) * ((int) circumference));
            frontRight.setTargetPosition((inchOrDeg/360) * ((int) circumference));
            backRight.setTargetPosition((inchOrDeg/360) * ((int) circumference));
            // turn right IN PLACE
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
            backLeft.setPower(power);
            while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy());
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        else {
            idle();
        }
    }
    private void park() {
       /*
The drive method has two parameters respectively for inching forward: power and inches. Alter them to meet your needs.
The drive method also has 3 parameters for turning: power (which is almost always 1), degrees, and direction of turn. Alter them to meet your needs
*/
        resetEncoders();
        frontLeft.setPower(1);
        backLeft.setPower(1);
        frontRight.setPower(-1);
        backRight.setPower(-1);
        driveForward(0.4, 24);
        //if the flywheel does not work as intended, remove the loop and try again. If that doesn’t work either, tell me
        flywheel.setPower(0.2);
        while (runtime.time() < 5) {
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            sleep(100);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            sleep(100);
        }
        frontLeft.setPower(-1);
        backLeft.setPower(-1);
        frontRight.setPower(1);
        backRight.setPower(1);
        driveForward(0.6, 24);
        frontLeft.setPower(-1);
        backLeft.setPower(-1);
        frontRight.setPower(1);
        backRight.setPower(1);
        driveForward(0.6, 72);
        idle();
    }
    //initialize

    public void runOpMode() throws InterruptedException  {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Is this built in? Hopefully
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        /* map motors */
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        // map elevators and arm
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        //brake when power is 0
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double power = 0.6;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        driveForward(power);
        sleep(1000);
        stopMotors();
        badTurnPleaseDelete(power, "blue");
        driveForward(power);
        sleep(1000);
        stopMotors();


        //gripperServo.setPosition(0);


        while(opModeIsActive()) {
            idle();
        }
    }
    //DELETE THIS AFTER SCRIMM!!!!!
    void badTurnPleaseDelete(double power, String color) {
        if(color.equals("blue")) {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        }
        else if(color.equals("red")){
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);
            backRight.setPower(-power);

        }
        sleep(2000);
        stopMotors();
        resetEncoders();
    }

}











