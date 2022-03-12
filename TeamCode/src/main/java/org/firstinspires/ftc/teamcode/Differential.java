package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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

// ****READ QUALCOMM PACKAGE Docs

public class Differential {
    
    private MotorsEx leftMotors , rightMotors , allMotors;
    private LinearOpMode opMode;
    private Telemetry telemetry;

    public Differential(MotorsEx leftMotors,
                        MotorsEx rightMotors,
                        MotorsEx allMotors,
                        LinearOpMode opMode) {
        this.rightMotors = rightMotors;
        this.leftMotors = leftMotors;
        this.allMotors = allMotors;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    Async async = new Async(telemetry , opMode);

        // 548 ticks = 360 degrees (Without Load)
        // 580 ticks = 360 degrees (With Load) = 12 inches
        // ~49 ticks = 30 degrees (With Load) = 1 inch
        final int INCH = 49;

        /**
     * Turns the robot in a circle of radius radius (inches)
     * With a multiplier of speed for degrees degrees
     *
     * @param radius    Radius to turn in inches
     * @param degrees   The degrees around the circle traveled
     * @param speed     Speed
     */
    public void differentialLeft(double radius,
                                 double degrees,
                                 double speed) {
        double left = (degrees / 360) * (radius - 7.5) * Math.PI;
        double right = (degrees / 360) * (radius + 7.5) * Math.PI;

        left = (left / 12) * 360 * speed;
        right = (right / 12) * 360 * speed;

        leftMotors.setVelocity(left , AngleUnit.DEGREES);
        rightMotors.setVelocity(right , AngleUnit.DEGREES);

        opMode.sleep((int)(1000 / Math.abs(speed)));

        allMotors.off();
    }

    /**
     * Turns the robot in a circle of radius radius (inches)
     * With a multiplier of speed for degrees degrees
     *
     * @param radius    Radius to turn in inches
     * @param degrees   The degrees around the circle traveled
     * @param speed     Speed
     */
    public void differentialRight(double radius,
                                 double degrees,
                                 double speed) {
        double right = (degrees / 360) * (radius - 7.5) * Math.PI;
        double left = (degrees / 360) * (radius + 7.5) * Math.PI;

        right = (right / 12) * 360 * speed;
        left = (left / 12) * 360 * speed;

        rightMotors.setVelocity(right , AngleUnit.DEGREES);
        leftMotors.setVelocity(left , AngleUnit.DEGREES);

        opMode.sleep((int)(1000 / Math.abs(speed)));

        allMotors.off();
    }
}
