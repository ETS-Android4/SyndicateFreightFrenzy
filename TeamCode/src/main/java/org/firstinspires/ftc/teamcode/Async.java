package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Async {
    private Telemetry telemetry;
    private LinearOpMode opmode;

    public Async(Telemetry telemetry,
                 LinearOpMode opmode) {
        this.telemetry = telemetry;
        this.opmode = opmode;
    }

    /**
     * Executes the passed runnable after delay milliseconds
     *
     * @param //Runnable - the runnable to be executed
     * @param //int - the delay in milliseconds
     */
    public void setTimeout(Runnable runnable, int delay) {
        new Thread(() -> {
            try {
                Thread.sleep(delay);
                runnable.run();
            }catch(Exception e) {
                if(telemetry != null) {
                    telemetry.addData("Error", e);
                    telemetry.update();
                }
            }
        }).start();
    }

    /**
     * Runs the passed code until the OpMode is terminated
     *
     * @param //Runnable - the runnable to be executed
     */
    public void perpetual(Runnable runnable) {
        new Thread(() -> {
            while(opmode.opModeIsActive()) {
                try {
                    while (opmode.opModeIsActive()) {
                        runnable.run();
                    }
                } catch (Exception e) {
                    if (telemetry != null) {
                        telemetry.addData("Error", e);
                        telemetry.update();
                    }
                }
            }
        }).start();
    }
}
