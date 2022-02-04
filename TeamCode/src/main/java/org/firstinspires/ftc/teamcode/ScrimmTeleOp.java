package org.firstinspires.ftc.teamcode;


//importing necessary packages
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.Servo;



//Declaring the name and group. This code is basic drivetrain code for the tank drive, which makes use of 2 sprockets for each edge and 2 treads.
@TeleOp
//extending OpMode so that we can use certain methods like init, loop, stop, etc
public class ScrimmTeleOp extends OpMode {

    //declaring local variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private DcMotor armMotor;
    private DcMotor flywheel;
    private Servo gripperServo;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void start() {
        runtime.reset();
    }

    //the code will loop through the content of this method and keep checking controller inputs



    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // initializing motors and servos
        // Create a new configuration on the DS and add these names to the correct ports on the hub

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
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // Gamepad 1 controls the motors and gamepad 2 controls the arm and gripper
        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_y;

        //precise movement with joysticks
        rightStick = Range.clip(rightStick, -0.5, 0.5);
        leftStick = Range.clip(leftStick, -0.5, 0.5);
       /*
       if 14-15 volts

       rightStick = Range.clip(rightStick, -1, 1);
       leftStick = Range.clip(leftStick, -1, 1);
        */


        //drivetrain
        frontLeft.setPower(leftStick);
        backLeft.setPower(leftStick);
        frontRight.setPower(rightStick);
        backRight.setPower(rightStick);
        if (gamepad1.dpad_left){
            frontRight.setPower(1);
            backRight.setPower(1);
            frontLeft.setPower(-1);
            backRight.setPower(-1);

        }
        if (gamepad1.dpad_right) {
            frontLeft.setPower(1);
            backLeft.setPower(1);
            frontRight.setPower(-1);
            backRight.setPower(-1);
        }

        if (gamepad2.dpad_up)
        {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(-.3);
        }
        else if(gamepad2.dpad_down)
        {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(.05);
        }
        else{
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(0);

        }


        //Button Control with Encoders Y is the Up position
        if(gamepad2.y)
        {
            //Encoder Motor Control
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setTargetPosition(-1300); //vary the parameter to meet your needs
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.2);

            while(armMotor.isBusy())
            {
                telemetry.addData("Target Position", -1300);
                telemetry.addData("Current Position", armMotor.getCurrentPosition());
                telemetry.update();
            }
        }
        //Button B is the Middle Position
        else if(gamepad2.b)
        {
            //Encoder Motor Control
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armMotor.setTargetPosition(-500); //vary the parameter to meet your needs
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.2);

            while(armMotor.isBusy())
            {
                telemetry.addData("Target Position", -500);
                telemetry.addData("Current Position", armMotor.getCurrentPosition());
                telemetry.update();
            }


        }
        //Button A is the Down Position
        else if(gamepad2.a)
        {
            //Encoder Motor Control
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            armMotor.setTargetPosition(-10); //vary the parameter to meet your needs
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(.2);

            while(armMotor.isBusy())
            {
                telemetry.addData("Target Position", -10);
                telemetry.addData("Current Position", armMotor.getCurrentPosition());
                telemetry.update();
            }

        }

        else if (gamepad2.dpad_left) {
            flywheel.setPower(0.2);
        }
        else if (gamepad2.dpad_right) {
            flywheel.setPower(0);
        }
        //Servo Open Position

        if (gamepad2.left_bumper)
        {
            gripperServo.setPosition(0); //Test the values
        }
        //Servo close Position
        else if (gamepad2.right_bumper)
        {
            gripperServo.setPosition(.5); //Test the values
        }

    }

    @Override
    public void stop() {

    }

    //end of program
}








