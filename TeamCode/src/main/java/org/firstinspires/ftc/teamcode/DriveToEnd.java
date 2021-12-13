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

    private DcMotor FL, FR, BL, BR, armMotor, flywheel;
    private Servo gripperServo;
    private BNO055IMU imu;
    private Orientation angles;

    private double power = 0.5;


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


        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);



        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            FL.setPower(-power);
            BL.setPower(-power);
            FR.setPower(power);
            BR.setPower(power);
        }
        else {
            //Right negative left positive
            FL.setPower(power);
            BL.setPower(power);
            FR.setPower(-power);
            BR.setPower(-power);
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
        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);
    }

    /**
     * Drives forward the specified amount of ticks
     * @param power Power of the motors. -1.0 - 1.0
     * @param ticks Amount of ticks to drive.
     */
    void driveForward(double power, int ticks) {
        resetEncoders();
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setTargetPosition(ticks);
        FR.setTargetPosition(ticks);
        BL.setTargetPosition(ticks);
        BR.setTargetPosition(ticks);

        driveForward(power);
        while(FL.getCurrentPosition() < FL.getTargetPosition() && opModeIsActive()) {}
        stopMotors();
        resetEncoders();
    }


    /**
     * Sets all drivetrain motor powers to 0
     */
    void stopMotors() {
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    /**
     * Resets all drivetrain encoders
     */
    void resetEncoders() {
        setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODERS, true);
    }
    /**
     * Sets the mode to whatever mode is specified
     * @param mode The mode that's set (STOP_AND_RESET_ENCODER, RUN_TO_POSITION, etc)
     * @param resetEncoders Set to true to reset the encoders along with whatever else you're doing
     */
    void setDrivetrainMode(DcMotor.RunMode mode, boolean resetEncoders) {
        if(resetEncoders) setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, false);
        
        FL.setMode(mode);
        FR.setMode(mode);
        BL.setMode(mode);
        BR.setMode(mode);
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
     * Checks if double x is between min and Max
     * @param x the input to check
     * @param min the min
     * @param the max
     */
    boolean isBetween(double x, double min, double max) {
        return x >= min && x <= max;
    }

    /**
     * Returns true or false if the robot should turn left or right.
     * Precondition: Both parameters are >= 0 and < 360
     * @param currentAngle The current angle the robot is facing
     * @param targetAngle The target angle for the robot
     * @return true if the robot should turn left, false if it should turn right
     */
    boolean shouldTurnLeft(double currentAngle, double targetAngle) {
        //Min and Max angles. Min = current angle, Max = current angle + 180
        double a1 = currentAngle;
        double a2 = a1 + 180; 
        
        if(a2 >= 360) a2 -= 360; // make sure a2 is from 0-359
        double theta = targetAngle;
        //If a1 is less than 180, add 360. This way a1 will not be less than a2
        if(a1 < 180) {
            a1 += 360;
            //Add 360 to theta if it's between 0 and the original angle
            if(isBetween(theta, 0, currentAngle)) theta += 360;
        }
        //true if theta is >=a2 and <=a1
        return isBetween(theta, a2, a1);
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

