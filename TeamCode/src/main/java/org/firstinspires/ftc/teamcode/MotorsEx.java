package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.List;

public class MotorsEx implements IMotorMethods {
    // Fields
    private DcMotorEx[] motors;
    public static String[] standardMotorOrder = {"FL" , "BL" , "FR" , "BR"};

    // Constructor
    public MotorsEx(DcMotorEx... motors) {
        this.motors = motors;
    }

    // DcMotorSimple.setPower(int)
    public void setPower(double power) {
        for(DcMotorEx motor : motors)
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
        for(DcMotorEx motor : motors)
            total += motor.getPower();
        return (total / motors.length) > 0.05;
    }

    // Turns off all motors
    public void off() {
        for(DcMotorEx motor : motors)
            motor.setPower(0);
    }

    // DcMotorSimple.setDirection(DcMotorSimple.Direction)
    public void setDirection(DcMotor.Direction direction) {
        for(DcMotorEx motor : motors)
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
        for(DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(behavior);
    }

    // DcMotor.isBusy()
    public boolean isBusy() {
        for(DcMotorEx motor : motors)
            if(motor.isBusy()) return true;
        return false;
    }

    // DcMotor.setTargetPosition(int)
    public void setTargetPosition(int position) {
        for(DcMotorEx motor : motors)
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

    public int getCurrentPosition(DcMotorEx specificMotor) {
        for(DcMotorEx motor : motors)
            if(motor == specificMotor)
                return motor.getCurrentPosition();
        return Integer.MIN_VALUE;
    }

    // Returns private DcMotors[] motors
    public DcMotorEx[] getMotors() {
        return motors;
    }

    // DcMotor.getController()
    public DcMotorController[] getController() {
        DcMotorController[] controllers = new DcMotorController[motors.length];
        for(int i = 0 ; i < controllers.length ; i++)
            controllers[i] = motors[i].getController();
        return controllers;
    }

    public DcMotorEx[] concat(DcMotorEx[]... motorArrays) {
        int totalLength = 0;
        for(DcMotorEx[] array : motorArrays)
            totalLength += array.length;
        DcMotorEx[] returned = new DcMotorEx[totalLength];
        int i = 0;
        for(DcMotorEx[] array : motorArrays) {
            for(int j = 0 ; j < array.length ; j++) {
                returned[i] = array[j];
                i++;
            }
        }
        return returned;
    }

    public List<DcMotorEx> asList() {
        return Arrays.asList(motors);
    }

    /**
     * Gradually stops the motor instead of abruptly
     *
     * Precondition: The motor is using encoders
     */
    public void taperedStop() {
        for(DcMotorEx motor : motors) {
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

    // DcMotorEx setVelocity (Ticks / sec)
    public void setVelocity(double angularRate) {
        for(DcMotorEx motor : motors)
            motor.setVelocity(angularRate);
    }

    // DcMotorEx setVelocity (Units / sec)
    public void setVelocity(double angularRate , AngleUnit angleUnit) {
        for(DcMotorEx motor : motors)
            motor.setVelocity(angularRate , angleUnit);
    }

    public double getAverageVelocity() {
        double total = 0;
        for(DcMotorEx motor : motors)
            total += motor.getVelocity();
        return total / motors.length;
    }
}
