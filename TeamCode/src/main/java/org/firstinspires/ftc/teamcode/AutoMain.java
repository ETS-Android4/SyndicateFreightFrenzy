package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.*;
import java.util.concurrent.*;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

enum StartPosition {
    RED_DEPOT,
    RED_WAREHOUSE,
    BLUE_DEPOT,
    BLUE_WAREHOUSE
}
enum ParkLocation {
    DEPOT,
    WAREHOUSE
}

@Autonomous
public class AutoMain extends LinearOpMode {

    //CHANGE THESE DURING TEAM BRIEF WITH ALLIANCE MEMBER
    //SHOULD ALLOW FOR SOME FLEXIBILITY DEPENDING ON THEIR AUTONOMOUS
    //MAY SCRAP THIS IDEA
    private StartPosition startPos = StartPosition.BLUE_DEPOT;
    private ParkLocation parkLocation = ParkLocation.DEPOT;
    private boolean spinCarousel = true;
    private boolean placeBlockOnShipping = true;



    private DcMotor FL, FR, BL, BR, armMotor, flywheel;
    DcMotor[] motors = new DcMotor[4];
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

        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        motors[0] = FL;
        motors[1] = FR;
        motors[3] = BR;
        motors[4] = BL;

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            gripperServo.setPosition(1);
            //@TODO: MAP OUT PATH
            // ticks
            /*
            drive(100 , 0);
            powerMotors(0.75 , armMotor);
            sleep(1000);
            stopMotors(armMotor);
            turn(180);
            drive(212 , 0);
            idle();
            */
            turn(90);
            drive(100,0);
            flywheel.setPower(0.25);
            sleep(500);
            turn(-90);
            drive(1000);
            
        }
    }
    public void turn(double angle) {
        /*
        Reads the IMU's heading, then sets it to the be the target
         */
        double target = imu.normalizeAngle(imu.getAngle() + angle);

        double left, right;

        boolean exit = false;
        boolean negative = target < imu.getAngle() && target - imu.getAngle() < 180;
        ElapsedTime timeout = new ElapsedTime();

        while (imu.getAngle() < target - .08 || imu.getAngle() > target + .08) {

            imu.update();
            left = -approx(IMU.Kp * (target - imu.getAngle()) * (target - imu.getAngle() > 180 || target - imu.getAngle() < -180 ? -1 : 1), .18, .6, false);
            right = -left;

            if (!exit) {
                if (negative && left < 0 || !negative && left > 0) {
                    exit = true;
                    timeout.reset();
                }
            }

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("angle", imu.getAngle());
            telemetry.update();

        }

        stopMotors();

    }
    public void powerMotors(double speed, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
    }
    public void stopMotors() {
        powerMotors(0);
    }
    public void drive(int distance, double targetAngle) {

        resetEncoders();
        double target = getEncoder() + distance;

        double left, right;

        boolean exit = false;
        boolean negative = target < 0;

        ElapsedTime timeout = new ElapsedTime();

        while (getEncoder() < target - 12 || getEncoder() > target + 12) {

            if (exit && timeout.time(TimeUnit.MILLISECONDS) > 500) {
                break;
            }

            // proportional drive
            left = approx(.005 * (target - getEncoder()), .18, .8, false);
            right = left;

            if (!exit) {
                if (negative && left > 0 || !negative && left < 0) {
                    exit = true;
                    timeout.reset();
                }
            }

            imu.update();
            double imuError = targetAngle - imu.getAngle(); // assume 0 -> 360 is clockwise
            left -= .05 * imuError;
            right += .05 * imuError;

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BR.getCurrentPosition());
            telemetry.addData("lp", left);
            telemetry.addData("rp", right);
            telemetry.addData("imu", imuError);
            telemetry.update();

        }

        stopMotors();
    }
    public void driveSpeed(int distance, double targetAngle, double speed) {

        resetEncoders();
        double target = getEncoder() + distance;

        double left, right;

        boolean condition = true;

        while (condition) {

            // proportional drive
            left = speed * (distance < 0 ? -1 : 1);
            right = left;

            imu.update();
            double imuError = targetAngle - imu.getAngle(); // assume 0 -> 360 is clockwise
            left -= .03 * imuError;
            right += .03 * imuError;

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("left", FL.getCurrentPosition());
            telemetry.addData("right", BR.getCurrentPosition());
            telemetry.addData("lp", left);
            telemetry.addData("rp", right);
            telemetry.addData("imu", imuError);
            telemetry.update();

            if (target < 0) {
                condition = getEncoder() > target;
            } else {
                condition = getEncoder() < target;
            }

        }

        stopMotors();

    }
    public void resetEncoders() {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void turnSpeed(double angle, double speed) {

        imu.update();
        double target = IMU.normalizeAngle(imu.getAngle() + angle);

        double left, right;

        boolean condition = true;

        while (condition) {

            imu.update();
            left = -speed * (angle < 0 ? -1 : 1);
            right = speed * (angle < 0 ? -1 : 1);

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("angle", imu.getAngle());
            telemetry.update();

            if (angle < 0) {
                condition = imu.getAngle() > target;
            } else {
                condition = imu.getAngle() < target;
            }

        }

        stopMotors();

    }
    public double getEncoder() {
        double sum = 0;
        for (DcMotor motor : motors) {
            sum += motor.getCurrentPosition();
        }
        return sum/motors.length;
    }
    public double approx(double value, double min, double max, boolean signed) {
        if (!signed && value >= 0)
            return Math.max(min, Math.min(value, max));
        else
            return Math.max(-max, Math.min(value, -min));
    }

}
