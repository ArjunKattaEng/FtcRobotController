package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.Collections;
import java.util.List;

/**
 * Webcam + AprilTag processor for the BIOBUZZ hives.
 *
 * Each hive carries 4 AprilTags. The SDK tag library groups those 4 tags into one
 * "cluster", so a hive is reported as ONE {@link AprilTagClusterDetection}, never as
 * single tags with IDs. Look hives up by name:
 *
 *   RED SCORING   (tags 30-33)     BLUE AUDIENCE (tags 38-41)
 *   RED AUDIENCE  (tags 34-37)     BLUE SCORING  (tags 42-45)
 *
 * ftcPose is measured from the camera to the cluster origin, in CM and DEGREES.
 * Seeing more of the hive's 4 tags (percentClusterFound) gives a steadier pose.
 *
 * init() does not wait for the camera, so it is safe in an iterative OpMode.
 * Call update() once per loop(); it returns no detections until the camera is streaming.
 */
public class AprilTagWebCam {
    public static final String WEBCAM_NAME = "Webcam 1";

    public static final String RED_SCORING   = "RED SCORING";
    public static final String RED_AUDIENCE  = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING  = "BLUE SCORING";

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private Telemetry telemetry;

    private List<AprilTagDetection> detectedTags = Collections.emptyList();

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                // Only the BIOBUZZ hive clusters (the default library also adds sample test tags).
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /** Call once per loop(). Detections are cleared while the camera is not streaming. */
    public void update() {
        if (isStreaming()) {
            detectedTags = aprilTagProcessor.getDetections();
        } else {
            detectedTags = Collections.emptyList();
        }
    }

    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Everything seen in the latest frame (hive clusters, plus any unknown single tags). */
    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    /** The hive with this name (e.g. {@link #BLUE_SCORING}), or null if it is not in view. */
    public AprilTagClusterDetection getHive(String name) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection hive = (AprilTagClusterDetection) detection;
                if (hive.metadata.shortName.equals(name)) {
                    return hive;
                }
            }
        }
        return null;
    }

    /** The nearest hive in view, or null if no hive is in view. */
    public AprilTagClusterDetection getClosestHive() {
        return closestHive(null);
    }

    /** The nearest BLUE hive (blue = true) or RED hive (blue = false) in view, or null if none. */
    public AprilTagClusterDetection getClosestHive(boolean blue) {
        return closestHive(blue);
    }

    /** blue == null means any color. */
    private AprilTagClusterDetection closestHive(Boolean blue) {
        AprilTagClusterDetection closest = null;
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection hive = (AprilTagClusterDetection) detection;
                if (blue != null && isBlue(hive) != blue) {
                    continue;
                }
                if (closest == null || hive.ftcPose.range < closest.ftcPose.range) {
                    closest = hive;
                }
            }
        }
        return closest;
    }

    /** True for BLUE SCORING / BLUE AUDIENCE, false for the red hives. */
    public static boolean isBlue(AprilTagClusterDetection hive) {
        return hive.metadata.shortName.startsWith("BLUE");
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public void displayDetectionTelemetry() {
        if (!isStreaming()) {
            telemetry.addData("Camera", visionPortal == null ? "not initialized" : visionPortal.getCameraState());
            return;
        }

        int hives = 0;
        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection hive = (AprilTagClusterDetection) detection;
                AprilTagPoseFtc pose = hive.ftcPose;
                hives++;
                telemetry.addLine(String.format("\n>>>  %s  <<<  (%d%% of tags seen)",
                        hive.metadata.name, hive.percentClusterFound));
                telemetry.addLine(String.format("  Range %6.1f cm   Bearing %6.1f deg   Yaw %6.1f deg",
                        pose.range, pose.bearing, pose.yaw));
                telemetry.addLine(String.format("  XYZ %6.1f %6.1f %6.1f (cm)   Elev %6.1f deg",
                        pose.x, pose.y, pose.z, pose.elevation));
            } else {
                // Not a hive tag: unknown size, so the SDK gives no pose for it.
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                telemetry.addLine(String.format("\n  Ignored tag ID %d (not a hive tag)", tag.id));
            }
        }

        telemetry.addData("Hives in view", hives);
        if (hives == 0) {
            telemetry.addLine("No hive in view - point camera at a hive");
        }
    }
}
