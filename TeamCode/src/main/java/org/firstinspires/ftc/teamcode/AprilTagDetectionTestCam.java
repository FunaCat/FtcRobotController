package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;
import java.util.concurrent.TimeUnit;

@TeleOp
public class AprilTagDetectionTestCam extends LinearOpMode {

    double coordinateX = 0;
    double coordinateY = 0;
    double heading = 0;
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private int     myExposure;
    private int     minExposure;
    private int     maxExposure;
    private int     myGain;
    private int     minGain;
    private int     maxGain;

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        getCameraSetting();
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;

        setManualExposure(6, 250);


        //myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
        //setManualExposure(myExposure, myGain);

        waitForStart();


        while (!isStopRequested() && opModeIsActive()) {
            tagProcessor.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
            if (tagProcessor.getDetections().size() > 0) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                switch (tag.metadata.id) {
                    case 1: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(-42, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 2: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(-36, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 3: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(-30, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 4: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(30, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 5: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(36, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 6: {
                        coordinateX = calculatePositionx(60, tag.ftcPose.x);
                        coordinateY = calculatePositionY(42, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                        break;
                    }
                    case 7: {
                        coordinateX = calculatePositionx(-72, tag.ftcPose.x);
                        coordinateY = calculatePositionY(42, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                    }
                    case 10: {
                        coordinateX = calculatePositionx(-72, tag.ftcPose.x);
                        coordinateY = calculatePositionY(-42, tag.ftcPose.y);
                        heading = tag.ftcPose.yaw;
                    }
                    default: {
                        break;
                    }
                }
            }
        }
    }

    public double calculatePositionx(double IDx, double Apriltagx) {
        return IDx - Apriltagx;
    }

    public double calculatePositionY(double Apriltagy, double IDy) {
        return IDy - Apriltagy;
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int)exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}
