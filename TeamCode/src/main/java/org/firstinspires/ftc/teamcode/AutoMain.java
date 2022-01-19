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



    private DcMotor FL, FR, BL, BR, armMotor, flywheel , slides;
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
        slides = hardwareMap.get(DcMotor.class, "slides");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        motors[0] = FL;
        motors[1] = FR;
        motors[2] = BR;
        motors[3] = BL;

        // FORWARD = positive = forward (right)
        // REVERSE = negative = forward (right)
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        slides.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        //while (opModeIsActive()) {
            // gripperServo.setPosition(1);
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
            // slides.setPower(0.75);
            // sleep(1000);
            /*
            telemetry.addData("Gripper: " , gripperServo.getPosition());
            telemetry.update();
            sleep(1000);
            gripperServo.setPosition(gripperServo.getPosition() + 0.1);
            telemetry.addData("Gripper: " , gripperServo.getPosition());
            telemetry.update();
            // gripperServo.setPosition(0.9);
            slides.setPower(0);
            */
            // sleep(5000);
            // slides.setPower(-0.75);
            // sleep(1000);
            // slides.setPower(0);
            // powerMotors(-0.5 , FR , BR);
            // powerMotors(0.5 , FL , BL);
            //for(int i = 0 ; i < 3 ; i++) {
            
            
            /*
            {
            // 1st Move
            moveForward(.3,250);
            moveForward(-0.5 , 250);
            
            // 1st Turn and 2nd Move Forward
            turnSpeed(90 , 1.0);
            moveForward(-0.5 , 1800);
            
            // 2nd Turn and 3rd Move Forward
            turnSpeed(-90 , -1.0);
            moveForward(-0.5 , 1450);
            
                /*drive(-100,0);
                flywheel.setPower(0.25);
                sleep(500);
                turn(-90);
                drive(-1000, 0); 
                
            // Ducky go spin
            flywheel.setPower(-0.50);
            sleep(2000);
            flywheel.setPower(0);
            sleep(2000);
            
            // Backup
            moveForward(0.5 , 800);
            
            // Zoom Zoom to warehouse
            turnSpeed(107 , 1.0);
            moveForward(-1.0 , 2600);
            }
            */
            
            FR.setPower(-1.0);
            BR.setPower(-1.0);
            FL.setPower(-1.0);
            BL.setPower(-1.0);
            sleep(1300);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            
            /*
            // Initial Positioning
            FR.setPower(-0.25);
            BR.setPower(-0.25);
            FL.setPower(-0.25);
            BL.setPower(-0.25);
            sleep(300);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            turnSpeed(-20 , 1.0);

            
            // 1st Move Forward
            FR.setPower(-0.25);
            BR.setPower(-0.25);
            FL.setPower(-0.25);
            BL.setPower(-0.25);
            sleep(500);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            
            // 1st Turn 2nd Move Forward
            turnSpeed(90 , 1.0);
            telemetry.addData("turn: " , "first");
            telemetry.update();
            sleep(3000);
            FR.setPower(-0.40);
            BR.setPower(-0.40);
            FL.setPower(-0.40);
            BL.setPower(-0.40);
            sleep(1370);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            
            // 2nd Turn 3rd Move Forward
            turnSpeed(-90 , -1.0);
            telemetry.addData("turn: " , "second");
            telemetry.update();
            sleep(3000);
            FR.setPower(-0.25);
            BR.setPower(-0.25);
            FL.setPower(-0.25);
            BL.setPower(-0.25);
            sleep(1500);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
                /*drive(-100,0);
                flywheel.setPower(0.25);
                sleep(500);
                turn(-90);
                drive(-1000, 0); /
            flywheel.setPower(-0.5);
            sleep(3500);
            flywheel.setPower(0);
            
            // Backup
            FR.setPower(0.25);
            BR.setPower(0.25);
            FL.setPower(0.25);
            BL.setPower(0.25);
            sleep(800);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            
            turnSpeed(-270 , -1.0);
            telemetry.addData("turn: " , "last");
            telemetry.update();
            
            
            FR.setPower(-1.0);
            BR.setPower(-1.0);
            FL.setPower(-1.0);
            BL.setPower(-1.0);
            sleep(2600);
            FR.setPower(0);
            BR.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
*/
            idle();
        //}
    }
    
    private void moveForward(double power , int time) {
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        BL.setPower(power);
        sleep(time);
        FR.setPower(0);
        BR.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
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

