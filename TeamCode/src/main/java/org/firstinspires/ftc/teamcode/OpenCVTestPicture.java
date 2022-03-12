package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name="Open CV Test (Picture)",group="tests")
public class OpenCVTestPicture extends LinearOpMode {

    //Constants for the webcam width and height
    private static final int WEBCAM_WIDTH = 864;
    private static final int WEBCAM_HEIGHT = 480;

    //Webcam
    private OpenCvCamera webcam;




    @Override
    public void runOpMode() throws InterruptedException
    {
        //Just copy paste this part
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Also copy paste this
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Sets the pipeline
        PicturePipeline pipeline = new PicturePipeline();
        webcam.setPipeline(pipeline);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            //Once you get the barcode level, make sure to stop streaming!!!
        }


    }
}


