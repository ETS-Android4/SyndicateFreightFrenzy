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
import org.openftc.easyopencv.OpenCvPipeline;


public class PicturePipeline extends OpenCvPipeline {

    //The submats are the 3 sections of the original mat.
    private Mat submat1, submat2, submat3;

    //Gray version of the input mat.
    private Mat gray = new Mat();

    //Barcode level.
    private int level;

    //Not really needed tbh but just to make sure level is never null
    public PicturePipeline() {
        level = 0;
    }

    //Main process frame method. Input is the input frame,
    //the return value is what is displayed on the preview
    @Override
    public Mat processFrame(Mat input) {
        //Converts the image to grayscale, easier to work with
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        //Makes 3 submats for each section of the camera.
        submat1 = new Mat(gray, new Rect(0, 0, input.width()/3, input.height()));
        submat2 = new Mat(gray, new Rect(input.width()/3, 0, input.width()/3, input.height()));
        submat3 = new Mat(gray, new Rect(2 * input.width()/3, 0, input.width()/3, input.height()));

        //Average RGB values of each submat in an array.
        int[] avgRGBs = {(int) findAvgRGB(submat1), (int) findAvgRGB(submat2), (int) findAvgRGB(submat3)};

        //Sets level to the max average RGB value
        level = findMax(avgRGBs);

        //Puts text on the submats, mostly just for debugging. The text is kinda hard to see because grayscale lol
        Imgproc.putText(submat1, String.valueOf(avgRGBs[0]), new Point(10, 100), Imgproc.FONT_HERSHEY_PLAIN, 3, new Scalar(0, 0, 0, 1));
        Imgproc.putText(submat2, String.valueOf(avgRGBs[1]), new Point(10, 100), Imgproc.FONT_HERSHEY_PLAIN, 3, new Scalar(0, 0, 0, 1));
        Imgproc.putText(submat3, String.valueOf(avgRGBs[2]), new Point(10, 100), Imgproc.FONT_HERSHEY_PLAIN, 3, new Scalar(0, 0, 0, 1));


        //Combines all 3 submats into one new mat. This is mostly for the preview.
        Mat newMat = new Mat();
        Core.hconcat(Arrays.asList(submat1, submat2, submat3), newMat);

        //Returns the new modified mat. It's grayscale and has 3 numbers,
        //being the average RGB color of each section.
        //Only displays the R value because R G and B are equal
        return newMat;
    }

    /**
     * Finds the average Red value of each section of the mat. Since each mat is
     * grayscale we don't have to care about green or blue.
     * Note: this loops over every pixel of the mat which can be expensive!
     *
     * @param src   The mat to find the average color of
     * @return      The average Red value of the Mat
     */
    private static double findAvgRGB(Mat src) {
        double output = 0;
        for(int r = 0; r < src.width(); r++) {
            for(int c = 0; c < src.height(); c++) {
                output += src.get(c, r)[0];
            }
        }
        return output/(src.width() * src.height());
    }

    /**
     * Finds the index of the max value of an array.
     *
     * @param arr   The array to check
     * @return      The index of the max value.
     */
    public int findMax(int[] arr) {
        int output = 0;
        for(int i = 1; i < arr.length; i++) {
            if(arr[i] > arr[output]) {
                output = i;
            }
        }
        return output;
    }


    /**
     * Returns the level that is the brightest, AKA the one with the capstone.
     * This should always be a number between 0 and 2!!!
     */
    public int getBarcodeLevel() {
        return level;
    }
}