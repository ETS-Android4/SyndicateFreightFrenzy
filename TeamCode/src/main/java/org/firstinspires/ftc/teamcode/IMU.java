package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {

    BNO055IMU imu;
    Orientation angles;
    public static final double Kp = .001;

    public IMU(HardwareMap hardwareMap) {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(initialize());

        angles = imu.getAngularOrientation();

    }
    
    /**
     * Normalizes the angle to be between -180 and 180.
     * @param angle     The angle to be normalized.
     * @return          The normalized angle
     */
    public static double normalizeAngle(double angle) {
        if (angle <= -180) {
            return normalizeAngle(angle + 360);
        } else if (angle >= 180) {
            return normalizeAngle(angle - 360);
        }
        return angle;
    }

    public double getAngle() {
        update();
        return angles.firstAngle;
    }

    public void update() {
        angles = imu.getAngularOrientation();
    }

    BNO055IMU.Parameters initialize() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        return parameters;

    }

}