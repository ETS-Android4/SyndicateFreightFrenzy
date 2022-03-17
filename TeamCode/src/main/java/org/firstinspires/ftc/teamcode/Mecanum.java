package org.firstinspires.ftc.teamcode.TankDrive;
//importing necessary packages
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


//Declaring the name and group. This code is basic drivetrain code for the tank drive, which makes use of 2 sprockets for each edge and 2 treads.
@TeleOp(name="Mecanum Drive", group="Iterative Opmode")
@Disabled

//extending OpMode so that we can use certain methods like init, loop, stop, etc
public class MecanumDrive extends OpMode {

   //declaring local variables
   private DcMotor frontLeft, frontRight, backRight, backLeft, armMotor, flywheel, slides;


   @Override
   public void start() {
  
   }

   //the code will loop through the content of this method and keep checking controller inputs



   @Override
   public void init() {
       telemetry.addData("Status", "Initialized");

       // Initialize the hardware variables. Note that the strings used here as parameters
       // to 'get' must correspond to the names assigned during the robot configuration
       // step (using the FTC Robot Controller app on the phone).
       // initializing motors and servos
       frontLeft  = hardwareMap.get(DcMotor.class, "FL");
       frontRight = hardwareMap.get(DcMotor.class, "FR");
       backRight = hardwareMap.get(DcMotor.class, "BR");
       backLeft = hardwareMap.get(DcMotor.class, "BL");


       //reversing the direction of certain motors, so that the controller inputs will accurately control the motor in the right direction
       frontLeft.setDirection(DcMotor.Direction.FORWARD);
       backLeft.setDirection(DcMotor.Direction.FORWARD);
       frontRight.setDirection(DcMotor.Direction.REVERSE);
       backRight.setDirection(DcMotor.Direction.REVERSE);

       //when the power is set to 0, the motor or servo stops; no questions asked
       backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   }

   @Override
   public void loop() {
       double drive  = gamepad1.left_stick_y;
       double twist  = gamepad1.right_stick_x;
       // Gamepad 1 controls the motors and gripper
       double leftStick;
       leftStick = gamepad1.left_stick_y;
       double leftX = -gamepad1.left_stick_x;
     

       //precise movement with joysticks
       rightStick = Range.clip(rightStick, -1, 1);
       leftStick = Range.clip(leftStick, -1, 1);

       double[] speeds = new double[]{
               (drive + leftX + twist),
               (drive - leftX - twist),
               (drive - leftX + twist),
               (drive + leftX - twist)
       };
       double max = Math.abs(speeds[0]);
       for(int i = 0; i < speeds.length; i++)
           if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);

       if (max > 1) {
           for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
       }
       //drivetrain
       frontLeft.setPower(speeds[0]);
       frontRight.setPower(speeds[1]);
       backLeft.setPower(speeds[2]);
       backRight.setPower(speeds[3]);

   }

   @Override
   public void stop() {

   }

   //end of program
}






