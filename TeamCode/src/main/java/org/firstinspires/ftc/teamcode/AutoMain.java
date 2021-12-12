package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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



    private DcMotor frontLeft, frontRight, backLeft, backRight, armMotor, flywheel;
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        gripperServo = hardwareMap.get(Servo.class, "gripper");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        while (opModeIsActive()) {
            idle();
        }
    }

}
