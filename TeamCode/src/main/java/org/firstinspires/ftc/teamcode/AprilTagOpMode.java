package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;

@TeleOp(name = "AprilTag Test", group = "Test")
public class AprilTagOpMode extends OpMode {

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);
        telemetry.addLine("Point camera at hive to scan AprilTags");
        telemetry.addLine("Hive names (RED/BLUE SCORING or AUDIENCE) appear below");
    }

    @Override
    public void loop() {
        aprilTagWebCam.update();
        aprilTagWebCam.displayDetectionTelemetry();
    }

    @Override
    public void stop() {
        aprilTagWebCam.stop();
    }
}
