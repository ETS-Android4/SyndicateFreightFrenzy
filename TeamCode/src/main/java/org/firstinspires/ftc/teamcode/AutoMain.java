package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private Servo gripperServo;
    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {

        //IMU setup. Sets the necessary parameters. Using degrees, not radians!
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Is this built in? Hopefully
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        
        angles = getAngularOrientation();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

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
            gripper.setPosition(1);
            idle();
        }
    }
    public static double normalizeAngle(double angle) {
        if (angle < -180) {
            return normalizeAngle(angle + 360);
        } else if (angle > 180) {
            return normalizeAngle(angle - 360);
        }
        return angle;
    }
    public void turn(double angle) {

        update();
        double target = normalizeAngle(getAngle() + angle);

        double left, right;

        boolean exit = false;
        boolean negative = target < getAngle() && target - getAngle() < 180;
        ElapsedTime timeout = new ElapsedTime();

        while (getAngle() < target - .08 || getAngle() > target + .08) {

            update();
            left = -approx(Kt * (target - getAngle()) * (target - getAngle() > 180 || target - getAngle() < -180 ? -1 : 1), .18, .6, false);
            right = -left;

            if (!exit) {
                if (negative && left < 0 || !negative && left > 0) {
                    exit = true;
                    timeout.reset();
                }
            }

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("angle", getAngle());
            telemetry.update();

        }

        stopMotors();

    }
    public double getAngle() {
        return angles.firstAngle;
    }
    public void powerMotors(double speed, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }
    }
    public void stopMotors() {
        powerAllMotors(0);
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

            update();
            double imuError = targetAngle - getAngle(); // assume 0 -> 360 is clockwise
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
    public void update() {
        angles = getAngularOrientation();
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

            update();
            double imuError = targetAngle - getAngle(); // assume 0 -> 360 is clockwise
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

        update();
        double target = normalizeAngle(getAngle() + angle);

        double left, right;

        boolean condition = true;

        while (condition) {

            update();
            left = -speed * (angle < 0 ? -1 : 1);
            right = speed * (angle < 0 ? -1 : 1);

            powerMotors(left, FL, BL);
            powerMotors(right, FR, BR);

            telemetry.addData("angle", getAngle());
            telemetry.update();

            if (angle < 0) {
                condition = getAngle() > target;
            } else {
                condition = getAngle() < target;
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
