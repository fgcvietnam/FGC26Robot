/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package RoadRunner;



import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import java.util.concurrent.atomic.AtomicReference;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Mat;


import global.first.IgnitingInnovationGameDatabase;
/**
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * using the Igniting Innovation tag library from IgnitingInnovationAprilTagLibrary.jar.
 */
@TeleOp(name = "Igniting Innovation April Tag Sample")
//@Disabled
public class IgnitingInnovationAprilTagSample extends LinearOpMode {
    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private FtcDashboard ftcDashboard;

    private TelemetryPacket packet;
    private CameraStreamProcessor cameraStreamProcessor;

    @Override
    public void runOpMode() {

        initAprilTag();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
            .setTagLibrary(IgnitingInnovationGameDatabase.getIgnitingInnovationTagLibrary())
            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setDrawTagID(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera.
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

            // ... these parameters are fx, fy, cx, cy.

            .build();

        // Create the vision portal by using a builder.

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(
                hardwareMap.get(WebcamName.class, "Webcam")
        );

        builder.addProcessor(aprilTag);

        builder.setCameraResolution(new Size(1280, 720));

        visionPortal = builder.build();
        // Set the webcam (name is assumed to be "Webcam 1")

        // Choose a camera resolution. Not all cameras support all resolutions.

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.

        // Build the Vision Portal, using the above settings.

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Function to add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                telemetry.addLine(detection.metadata.name);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(
                        Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565)
                );

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(
                    Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565)
            );
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {

            Bitmap bitmap = Bitmap.createBitmap(
                    frame.width(),
                    frame.height(),
                    Bitmap.Config.RGB_565
            );

            Utils.matToBitmap(frame, bitmap);
            lastFrame.set(bitmap);

            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
            // Nothing needed here.
        }

        @Override
        public void getFrameBitmap(
                Continuation<? extends Consumer<Bitmap>> continuation
        ) {
            continuation.dispatch(
                    bitmapConsumer -> bitmapConsumer.accept(lastFrame.get())
            );
        }
    }
}   // end class
