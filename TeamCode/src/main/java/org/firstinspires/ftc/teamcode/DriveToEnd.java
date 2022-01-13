/*
 * NOTICE THIS IS COMPLETELY UNTESTED
 * IT PROBABLY DOES NOT WORK
 * I'LL CLEAN IT UP AFTER I GET IT TO WORK
 * THANK YOU
 * -Peter Tvedt
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

//Might get rid of this idk
enum StartingPosition {
    BLUE_SIDE,
    RED_SIDE
}

@Autonomous
public class DriveToEnd extends LinearOpMode {

    /*
     * General idea:
     * 1. set facing direction to 0 degrees (might be done automatically idk)
     * 2. set target direction (either 90 degrees or -90 degrees depending on side)
     * 3. move in a straight line until in parking space (maybe can check when pitch changes?)
     * 4. done
     */

    private DcMotor frontLeft, frontRight, backLeft, backRight, armMotor, flywheel;
    private Servo gripperServo;
    private BNO055IMU imu;
    private Orientation angles;

    private double power = 0.25;


    //If we're allowed to change code before hand just change this variable ig
    private static final StartingPosition STARTING_POSITION = StartingPosition.RED_SIDE;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //IMU setup. Sets the necessary parameters. Using degrees, not radians!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Is this built in? Hopefully
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);



        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Telemetry info
        composeTelemetry();

        double targetAngle = 0;

        //Play with values
        switch (STARTING_POSITION) {
            case RED_SIDE:
                targetAngle = -90;
                break;
            case BLUE_SIDE:
                targetAngle = 90;
                break;
        }

        waitForStart();

        resetEncoders();
        updateOrientation();
        driveForward(power, 200);

        turnToAngle(power, targetAngle);


        double angleThreshold = 5; //CHANGE THIS LATER, FOR PARKING BIT

        updateOrientation();
        double angleX = angles.thirdAngle;

        //Drive until hit bump twice
        driveForward(power);
        while(opModeIsActive() && isInRange(angleX, angleX, angleThreshold)) {
            updateOrientation();
            telemetry.update();
        }
        stopMotors();
        resetEncoders();
        idle();
    }

    /**
     * Turns the robot until desired target angle is reached. Will probably break!
     * Precondition: targetAngle >= 0 && targetAngle < 360
     * @param targetAngle the target angle the robot should face.
     */
    void turnToAngle( double power, double targetAngle) {
        double error = 5;
        updateOrientation();
        double originalAngle = angles.firstAngle;

        //Make sure target angle isn't too close to the original angle idk I feel like this would cause issues
        if(isInRange(originalAngle, targetAngle, error)) return;

        if(shouldTurnLeft(originalAngle, targetAngle)) {
            //Left negative Right positive
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);
        }
        else {
            //Right negative left positive
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
        }
        //Turn until target angle met
        while(opModeIsActive() && !isInRange(originalAngle, targetAngle, error)) {}
        stopMotors();
        resetEncoders();

    }

    /**
     * Drives forward until stopped
     * @param power Power of the motors. -1.0 - 1.0
     */
    void driveForward(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Drives forward the specified amount of ticks
     * @param power Power of the motors. -1.0 - 1.0
     * @param ticks Amount of ticks to drive.
     */
    void driveForward(double power, int ticks) {
        resetEncoders();
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        driveForward(power);
        while(frontLeft.getCurrentPosition() < frontLeft.getTargetPosition() && opModeIsActive()) {}
        stopMotors();
        resetEncoders();
    }


    /**
     * Sets all drivetrain motor powers to 0
     */
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


    /**
     * Updates the angle when called
     * Precondition: Orientation angles is declared
     */
    private void updateOrientation() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    /**
     * Checks if a given double is close to a target double within a range of error
     * @param num The input to check
     * @param target The target number that num must be close to
     * @param error The amount of error off target that num is allowed to be
     * @return true if target-error <= num <= target+error
     */
    private boolean isInRange(double num, double target, double error) {
        return num >= target - error && num <= target + error;
    }


    /**
     * Returns true or false if the robot should turn left or right.
     * Precondition: Both parameters are >= 0 and < 360
     * @param currentAngle The current angle the robot is facing
     * @param targetAngle The target angle for the robot
     * @return true if the robot should turn left, false if it should turn right
     */
    boolean shouldTurnLeft(double currentAngle, double targetAngle) {
        double angle1 = currentAngle;
        double angle2 = angle1 + 180;
        if(angle2 >= 360) angle2 -= 360;
        double newTargetAngle = targetAngle;
        if(angle1 < 180) {
            angle1 += 360;
            if(newTargetAngle >= 0 && newTargetAngle <= currentAngle) newTargetAngle += 360;
        }
        return newTargetAngle >= angle2 && newTargetAngle <= angle1;
    }




    /**
     * Copy pasted for testing reasons
     * Shows info about pitch roll and heading mostly
     * Add lines if needed
     */
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

