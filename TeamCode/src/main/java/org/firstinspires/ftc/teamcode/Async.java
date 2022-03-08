package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Async /* implements Runnable */{
    private Telemetry telemetry;

    public Async(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
    * Executes the passed runnable after delay milliseconds
    *
    * @param Runnable - the runnable to be executed
    * @param int - the delay in milliseconds
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
    * @param Runnage - the runnable to be executed
    */
    public void perpetual(Runnable runnable) {}
}
