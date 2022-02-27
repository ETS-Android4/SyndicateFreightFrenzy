package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Async {
    private Telemetry telemetry;

    public Async(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

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
}
