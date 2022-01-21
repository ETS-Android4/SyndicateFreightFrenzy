package org.firstinspires.ftc.teamcode;

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
    private static final int LENGTH = 14, WIDTH = 18, WHEEL_RADIUS = 2, TICKS_PER_ROT = 1440, GEAR_RATIO = 60,
            TICKS_PER_DEGREE = TICKS_PER_ROT * GEAR_RATIO / 360;
    private static final double TICKS_PER_INCH = TICKS_PER_ROT * GEAR_RATIO / 2 * Math.PI * WHEEL_RADIUS,
            TURN_RADIUS = Math.sqrt(Math.pow(LENGTH/2) + Math.pow(WIDTH/2));

    @Override
    public void runOpMode() {
        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        FR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        BL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
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
            turn(-90);
            drive(-100,0);
            flywheel.setPower(0.25);
            sleep(500);
            turn(-90);
            drive(-1000, 0);
            idle();
        }
    }
    public static void turn(double degrees) {
            double turnInches = (degrees/360) * (2 * Math.PI * TURN_RADIUS);
            FL.setTargetPosition((int)(FL.getCurrentPosition() + (turnInches * TICKS_PER_INCH / 2)));
            FR.setTargetPosition((int)(FR.getCurrentPosition() - (turnInches * TICKS_PER_INCH / 2)));
            BL.setTargetPosition((int)(BL.getCurrentPosition() + (turnInches * TICKS_PER_INCH / 2)));
            BR.setTargetPosition((int)(BR.getCurrentPosition() - (turnInches * TICKS_PER_INCH / 2)));
            if (degrees > 0) {
                FL.setPower(0.5);
                FR.setPower(0.5);
                BL.setPower(0.5);
                BR.setPower(0.5);
            }
            else {
                FL.setPower(-0.5);
                FR.setPower(0.5);
                BL.setPower(-0.5);
                BR.setPower(0.5);
            }
        }
    private static void drive(String type, double value) {
            type = type.toLowerCase();
            if (type == "inches") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_INCH)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() + (value * TICKS_PER_INCH)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() + (value * TICKS_PER_INCH)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_INCH)));
                BR.setPower(.5);
            }
            else if (type == "degrees") {
                FL.setTargetPosition((int)(FL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                FL.setPower(.5);
                FR.setTargetPosition((int)(FR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                FR.setPower(.5);
                BL.setTargetPosition((int)(BL.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                BL.setPower(.5);
                BR.setTargetPosition((int)(BR.getCurrentPosition() + (value * TICKS_PER_DEGREE)));
                BR.setPower(.5);
            }
        }

}
