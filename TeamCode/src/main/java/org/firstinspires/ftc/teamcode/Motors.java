package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.teamcode.IMotorMethods;

import java.util.List;

public class Motors implements IMotorMethods {
    // Fields
    private DcMotor[] motors;
    public static String[] standardMotorOrder = {"FL" , "BL" , "FR" , "BR"};

    // Constructor
    public Motors(DcMotor... motors) {
        this.motors = motors;
    }

    // DcMotorSimple.setPower(int)
    public void setPower(double power) {
        for(DcMotor motor : motors)
            motor.setPower(power);
    }

    // DcMotorSimple.getPower()
    public double getPower() {
        return motors[0].getPower();
    }

    /**
     * Determines if any of the motors have power
     *
     * @return boolean
     */
    public boolean hasPower() {
        double total = 0;
        for(DcMotor motor : motors)
            total += motor.getPower();
        return (total / motors.length) > 0.05;
    }

    // Turns off all motors
    public void off() {
        for(DcMotor motor : motors)
            motor.setPower(0);
    }

    // DcMotorSimple.setDirection(DcMotorSimple.Direction)
    public void setDirection(DcMotor.Direction direction) {
        for(DcMotor motor : motors)
            motor.setDirection(direction);
    }

    // DcMotorSimple.getDirection()
    public DcMotor.Direction getDirection() {
        return motors[0].getDirection();
    }

    // DcMotor.setMode(DcMotor.RunMode)
    public void setMode(DcMotor.RunMode mode) {
        for(DcMotor motor : motors)
            motor.setMode(mode);
    }

    // DcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior)
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for(DcMotor motor : motors)
            motor.setZeroPowerBehavior(behavior);
    }

    // DcMotor.isBusy()
    public boolean isBusy() {
        for(DcMotor motor : motors)
            if(motor.isBusy()) return true;
        return false;
    }

    // DcMotor.setTargetPosition(int)
    public void setTargetPosition(int position) {
        for(DcMotor motor : motors)
            motor.setTargetPosition(position);
    }

    // DcMotor.getTargetPosition()
    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    // DcMotor.getCurrentPosition()
    public int[] getCurrentPosition() {
        int[] positions = new int[motors.length];
        for(int i = 0 ; i < positions.length ; i++)
            positions[i] = motors[i].getCurrentPosition();
        return positions;
    }

    public int getCurrentPosition(DcMotor specificMotor) {
        for(DcMotor motor : motors)
            if(motor == specificMotor)
                return motor.getCurrentPosition();
        return Integer.MIN_VALUE;
    }

    // Returns private DcMotors[] motors
    public DcMotor[] getMotors() {
        return motors;
    }

    // DcMotor.getController()
    public DcMotorController[] getController() {
        DcMotorController[] controllers = new DcMotorController[motors.length];
        for(int i = 0 ; i < controllers.length ; i++)
            controllers[i] = motors[i].getController();
        return controllers;
    }

    public DcMotor[] concat(DcMotor[]... motorArrays) {
        int totalLength = 0;
        for(DcMotor[] array : motorArrays)
            totalLength += array.length;
        DcMotor[] returned = new DcMotor[totalLength];
        int i = 0;
        for(DcMotor[] array : motorArrays) {
            for(int j = 0 ; j < array.length ; j++) {
                returned[i] = array[j];
                i++;
            }
        }
        return returned;
    }

    public List<DcMotor> asList() {
        return Arrays.asList(motors);
    }

    /**
     * Gradually stops the motor instead of abruptly
     *
     * Precondition: The motor is using encoders
     */
    public void taperedStop() {
        for(DcMotor motor : motors) {
            double difference = Math.abs(motor.getTargetPosition()) -
                    Math.abs(motor.getCurrentPosition());
            double sixthEnd = Math.abs(motor.getTargetPosition()) / 6.0;

            if(Math.abs(motor.getPower()) > 0 &&
                    difference <= sixthEnd &&
                    difference > (sixthEnd / 4))
                motor.setPower(motor.getPower() * (difference / sixthEnd));
            else if(difference <= 10)
                motor.setPower(0);
        }
    }
}
