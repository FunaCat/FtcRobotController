package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


public class EOCVOpModeTestCam extends LinearOpMode {

    BLUEIDENTIFICATION blueIdentificationProcess;
    REDIDENTIFICATION redIdentificationProcess;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(
                WebcamName.class, "Webcam 1"), blueIdentificationProcess, redIdentificationProcess);
        visionPortal.setProcessorEnabled(blueIdentificationProcess, true);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Connected");
        }
        visionPortal.close();
    }
}
