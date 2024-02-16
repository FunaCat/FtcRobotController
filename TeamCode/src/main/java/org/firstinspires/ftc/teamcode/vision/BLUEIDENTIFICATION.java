package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BLUEIDENTIFICATION extends OpenCvPipeline
{
    Mat defaultMAT = new Mat();
    Mat mixture_1MAT = new Mat();

    static final Rect ROI = new Rect(
            new Point(100, 175),
            new Point(300, 325));

    static double PERCENT_THRESHOLD = 0.4;


    private Telemetry telemetry;

    public BLUEIDENTIFICATION(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mixture_1MAT, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(92, 50, 60);
        Scalar highHSV = new Scalar(160, 250, 250);
        Core.inRange(mixture_1MAT, lowHSV, highHSV, mixture_1MAT);

        Mat rectangle = mixture_1MAT.submat(ROI);
        double rectanglePercentage = Core.sumElems(rectangle).val[0] / ROI.area() / 255;
        rectangle.release();

        telemetry.addData("rectangle Percentage: ", Math.round(rectanglePercentage * 100) + "%");
        Scalar color = new Scalar(255, 100, 0);

        Imgproc.rectangle(mixture_1MAT, ROI, color);

        if (rectanglePercentage > PERCENT_THRESHOLD) {
            telemetry.addLine("Blue detected");
        }
        telemetry.update();
        return mixture_1MAT;

    }
}
