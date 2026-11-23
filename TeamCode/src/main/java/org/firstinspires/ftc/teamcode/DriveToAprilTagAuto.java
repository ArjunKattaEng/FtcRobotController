package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/*
 * Alliance: after INIT and before START, press gamepad1 X for BLUE or B for RED.
 * The robot then aims only at its own alliance's hives (the nearer of SCORING / AUDIENCE),
 * and on both alliances it shoots the 4 preloaded balls, then drives under the hive.
 *
 * Start position: the BACK of the robot faces the hive, with the webcam mounted on the
 * back looking at the hive (the launcher also shoots out the back, as in BioBuzzStarterbotAuto).
 * So "toward the hive" is BACKWARD: the approach and the cross-field drive use negative power.
 * Turning needs no change: rotating the robot pans a rear camera the same way as a front one,
 * so a positive bearing (hive to the camera's left) still means turn counter-clockwise.
 *
 * 30 s budget: the Driver Station stops an Autonomous OpMode at 30 s, so the code stops itself
 * first. The approach must arrive by APPROACH_DEADLINE_SEC, shooting stops by SHOOT_DEADLINE_SEC
 * so the cross-field drive still fits, and everything is off by FINISH_BY_SEC.
 */
@Autonomous(name = "Drive2AprilTagAuto", group = "Test")
public class DriveToAprilTagAuto extends OpMode {

    // ---- Orientation: -1 = the back faces the hive (drive backward to go toward it) ----
    private static final double TOWARD_HIVE = -1.0;

    // ---- Alliance used if nobody presses X / B during INIT ----
    private static final boolean DEFAULT_BLUE_ALLIANCE = true;

    // ---- Drive tuning ----
    private static final double DESIRED_DISTANCE_CM = 30.0;
    private static final double SPEED_GAIN          = 0.02;
    private static final double TURN_GAIN           = 0.01;
    private static final double MAX_AUTO_SPEED      = 0.5;
    private static final double MAX_AUTO_TURN       = 0.25;

    // ---- Timeouts ----
    private static final double SEARCH_TIMEOUT_SEC   = 10.0;
    private static final double APPROACH_TIMEOUT_SEC = 10.0;
    private static final double ARRIVED_HOLD_SEC     = 1.0;

    // ---- 30 s budget (times on the auto clock, which starts at START) ----
    private static final double AUTO_SEC              = 30.0;  // the Driver Station stops the OpMode here
    private static final double FINISH_BY_SEC         = 29.0;  // everything stopped by code, 1 s early
    private static final double SHOOT_DEADLINE_SEC    = 24.0;  // stop shooting: leaves 4 s cross + 1 s spare
    private static final double APPROACH_DEADLINE_SEC = 14.0;  // not arrived by now -> give up
    // 14 s leaves 10 s before SHOOT_DEADLINE_SEC: hold 1 + spin-up up to 5 + 4 s of feeding.

    // ---- Shooter tuning (copied from BioBuzzStarterbotAuto) ----
    private static final int    LAUNCHER_TARGET_VELOCITY = 1210;
    private static final int    LAUNCHER_MIN_VELOCITY    = 1160;
    private static final double LAUNCHER_FALLBACK_POWER  = 0.85;
    private static final double FEED_INTAKE_POWER        = 0.5;
    private static final double FEED_SECONDS_PER_BALL    = 1.0;
    private static final int    BALLS_TO_SHOOT           = 4;
    private static final double SPIN_UP_TIMEOUT_SEC      = 5.0;
    private static final double SHOOT_MAX_SEC            = 10.0;

    // ---- Cross-field drive ----
    private static final double CROSS_FIELD_SEC = 4.0;
    private static final double CROSS_FIELD_POWER = 0.5;

    // ---- State machine ----
    private enum State {
        SEARCHING, APPROACHING, ARRIVED,
        SPIN_UP, SHOOTING,
        CROSS_FIELD,
        DONE
    }
    private State state;
    private ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();   // the 30 s auto clock
    private String shotResult = "";                            // how shooting went (shown in DONE)
    private String doneReason = "";                            // why the run ended (shown in DONE)

    // ---- Alliance (chosen with the gamepad during INIT) ----
    private boolean blueAlliance = DEFAULT_BLUE_ALLIANCE;
    private boolean allianceChosen = false;

    // ---- Target hive (locked when first seen, so a nearer hive can't steal the approach) ----
    private String targetHiveName = null;

    // ---- Hardware ----
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx launcher;
    private DcMotorEx intake;
    private CRServo windmill;
    private boolean shooterOk = false;
    private boolean windmillOk = false;
    private AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    // ---- Shooting progress (same feed logic as BioBuzzStarterbotAuto) ----
    private final ElapsedTime loopTimer = new ElapsedTime();   // measures each loop's real length
    private double fedSeconds = 0;      // time actually spent feeding with the launcher at speed
    private boolean feedOn = false;
    private int feedPauses = 0;         // times feeding paused to let the launcher recover
    private boolean openLoop = false;   // spin-up timed out: launcher runs on plain power from then on

    @Override
    public void init() {
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        try {
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
            shooterOk = true;
        } catch (Exception e) {
            shooterOk = false;
        }

        try {
            windmill = hardwareMap.get(CRServo.class, "windmill");
            windmill.setDirection(DcMotorSimple.Direction.REVERSE);
            windmillOk = true;
        } catch (Exception e) {
            windmillOk = false;
        }

        aprilTagWebCam.init(hardwareMap, telemetry);
    }

    /** After INIT, before START: choose the alliance and check what the camera sees. */
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            blueAlliance = true;
            allianceChosen = true;
        } else if (gamepad1.b) {
            blueAlliance = false;
            allianceChosen = true;
        }

        telemetry.addLine(String.format("ALLIANCE: %s%s", allianceName(),
                allianceChosen ? "" : "   (default — press X or B to choose)"));
        telemetry.addLine("Gamepad 1:  X = BLUE    B = RED");
        telemetry.addLine("Place the robot with its BACK (webcam) facing the hive");

        aprilTagWebCam.update();
        if (!aprilTagWebCam.isStreaming()) {
            telemetry.addData("Camera", "starting...");
        } else {
            AprilTagClusterDetection hive = aprilTagWebCam.getClosestHive(blueAlliance);
            telemetry.addData("Camera sees", hive == null
                    ? "none of the " + allianceName() + " hives"
                    : String.format("%s at %.0f cm", hive.metadata.shortName, hive.ftcPose.range));
        }

        telemetry.addData("Shooter", shooterOk ? "OK" : "NOT FOUND — will skip shooting");
        telemetry.addData("Windmill", windmillOk ? "OK" : "NOT FOUND");
    }

    @Override
    public void start() {
        state = State.SEARCHING;
        stateTimer.reset();
        loopTimer.reset();
        autoTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();   // real time since the last loop (not an assumed 20 ms)
        loopTimer.reset();

        aprilTagWebCam.update();
        telemetry.addData("Alliance", allianceName());
        telemetry.addData("Auto clock", "%.1f / %.0f s", autoTimer.seconds(), AUTO_SEC);

        switch (state) {
            case SEARCHING:   runSearching();   break;
            case APPROACHING: runApproaching(); break;
            case ARRIVED:     runArrived();     break;
            case SPIN_UP:     runSpinUp();      break;
            case SHOOTING:    runShooting(dt);  break;
            case CROSS_FIELD: runCrossField();  break;
            case DONE:        runDone();        break;
        }
    }

    @Override
    public void stop() {
        stopMotors();
        stopShooter();
        aprilTagWebCam.stop();
    }

    // ======== SEARCHING — spin until one of OUR alliance's hives is in view ========
    private void runSearching() {
        AprilTagClusterDetection hive = aprilTagWebCam.getClosestHive(blueAlliance);

        if (hive != null) {
            targetHiveName = hive.metadata.shortName;
            telemetry.addData("State", "FOUND: %s", targetHiveName);
            state = State.APPROACHING;
            stateTimer.reset();
            return;
        }

        if (stateTimer.seconds() > SEARCH_TIMEOUT_SEC) {
            finish("SEARCH TIMEOUT — no " + allianceName() + " hive seen");
            return;
        }

        telemetry.addData("State", "SEARCHING for a %s hive (%.0fs left)",
                allianceName(), SEARCH_TIMEOUT_SEC - stateTimer.seconds());
        moveRobot(0, 0.15);
    }

    // ======== APPROACHING ========
    private void runApproaching() {
        if (autoTimer.seconds() > APPROACH_DEADLINE_SEC) {
            finish(String.format("OUT OF TIME — not at %s by %.0f s", targetHiveName, APPROACH_DEADLINE_SEC));
            return;
        }

        AprilTagClusterDetection target = aprilTagWebCam.getHive(targetHiveName);

        if (target != null) {
            double rangeError   = target.ftcPose.range - DESIRED_DISTANCE_CM;
            double headingError = target.ftcPose.bearing;

            telemetry.addData("State", "APPROACHING %s", targetHiveName);
            telemetry.addData("Range",   "%5.1f cm (want %.0f)", target.ftcPose.range, DESIRED_DISTANCE_CM);
            telemetry.addData("Bearing", "%3.0f deg", target.ftcPose.bearing);

            if (Math.abs(rangeError) < 3.0) {
                stopMotors();
                state = State.ARRIVED;
                stateTimer.reset();
                return;
            }

            // Too far (rangeError > 0) = move toward the hive, which is backward for this robot.
            double drive = TOWARD_HIVE * Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            moveRobot(drive, turn);
            stateTimer.reset();
        } else {
            stopMotors();
            telemetry.addData("State", "APPROACHING — tag lost, waiting...");

            if (stateTimer.seconds() > APPROACH_TIMEOUT_SEC) {
                finish(String.format("HIVE LOST — %s out of view for %.0f s", targetHiveName, APPROACH_TIMEOUT_SEC));
            }
        }
    }

    // ======== ARRIVED — settle, then shoot (or skip to the cross if there is no shooter) ========
    private void runArrived() {
        stopMotors();
        telemetry.addData("State", "ARRIVED at %s", targetHiveName);

        if (stateTimer.seconds() > ARRIVED_HOLD_SEC) {
            if (shooterOk) {
                state = State.SPIN_UP;
                stateTimer.reset();
            } else {
                startCrossField("no shooter hardware — skipped shooting");
            }
        }
    }

    // ======== SPIN_UP — get launcher to speed ========
    private void runSpinUp() {
        if (autoTimer.seconds() > SHOOT_DEADLINE_SEC) {
            startCrossField("out of time while spinning up — did not shoot");
            return;
        }

        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        double vel = launcher.getVelocity();

        telemetry.addData("State", "SPIN UP");
        telemetry.addData("Launcher", "%.0f / %d ticks/s", vel, LAUNCHER_TARGET_VELOCITY);

        // Signed on purpose: a launcher spinning the wrong way reads negative and never "reaches speed".
        if (vel >= LAUNCHER_MIN_VELOCITY) {
            startShooting();
            return;
        }

        if (stateTimer.seconds() > SPIN_UP_TIMEOUT_SEC) {
            // Never reached speed: switch to plain power for the rest of the run.
            openLoop = true;
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
            startShooting();
        }
    }

    private void startShooting() {
        state = State.SHOOTING;
        stateTimer.reset();
        fedSeconds = 0;
        feedPauses = 0;
    }

    // ======== SHOOTING — feed balls into launcher ========
    private void runShooting(double dt) {
        commandLauncher();
        double vel = launcher.getVelocity();

        // Feed only while the launcher is at speed; pause so it can recover between balls.
        // Only time spent feeding counts toward the balls. In open loop the speed can't be
        // trusted, so it feeds without waiting.
        if (openLoop || vel >= LAUNCHER_MIN_VELOCITY) {
            feed(true);
            fedSeconds += dt;
        } else {
            if (feedOn) feedPauses++;
            feed(false);
        }

        double feedNeeded = BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL;

        telemetry.addData("State", "SHOOTING%s", openLoop ? " (plain power — never reached speed)" : "");
        telemetry.addData("Launcher", "%.0f / %d ticks/s", vel, LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Feed", feedOn ? "ON" : "PAUSED (launcher < " + LAUNCHER_MIN_VELOCITY + ")");
        telemetry.addData("Fed", "%.1f / %.1f s (~%d / %d balls)", fedSeconds, feedNeeded,
                Math.min(BALLS_TO_SHOOT, (int) (fedSeconds / FEED_SECONDS_PER_BALL)), BALLS_TO_SHOOT);
        telemetry.addData("Feed pauses", "%d", feedPauses);

        boolean allFed   = fedSeconds >= feedNeeded;
        boolean tooLong  = stateTimer.seconds() > SHOOT_MAX_SEC;
        boolean deadline = autoTimer.seconds() > SHOOT_DEADLINE_SEC;
        if (allFed || tooLong || deadline) {
            startCrossField(String.format("shot (%s): fed %.1f / %.1f s, %d pauses",
                    allFed ? "all fed" : tooLong ? "SHOOT_MAX_SEC" : "shoot deadline",
                    fedSeconds, feedNeeded, feedPauses));
        }
    }

    /** Keeps the launcher running: velocity control, or plain power after a spin-up timeout. */
    private void commandLauncher() {
        if (openLoop) {
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
        } else {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }

    private void feed(boolean on) {
        feedOn = on;
        if (windmillOk) windmill.setPower(on ? 1.0 : 0.0);
        intake.setPower(on ? FEED_INTAKE_POWER : 0.0);
    }

    // ======== CROSS_FIELD — drive backward under the hive ========
    private void startCrossField(String result) {
        stopShooter();
        shotResult = result;
        state = State.CROSS_FIELD;
        stateTimer.reset();
    }

    private void runCrossField() {
        telemetry.addData("State", "CROSSING FIELD");
        telemetry.addData("Time", "%.1f / %.1f s", stateTimer.seconds(), CROSS_FIELD_SEC);

        moveRobot(TOWARD_HIVE * CROSS_FIELD_POWER, 0);   // backward, under the hive

        if (stateTimer.seconds() > CROSS_FIELD_SEC) {
            finish(shotResult + "; crossed the field");
        } else if (autoTimer.seconds() > FINISH_BY_SEC) {
            finish(String.format("%s; cross cut short by the 30 s budget (%.1f / %.1f s)",
                    shotResult, stateTimer.seconds(), CROSS_FIELD_SEC));
        }
    }

    // ======== DONE ========
    private void runDone() {
        stopMotors();
        telemetry.addData("State", "DONE — %s", doneReason);
    }

    /** Ends the run: motors and shooter off, then DONE shows why it ended. */
    private void finish(String reason) {
        stopMotors();
        stopShooter();
        doneReason = reason;
        state = State.DONE;
        stateTimer.reset();
        telemetry.addData("State", "DONE — %s", reason);
    }

    private String allianceName() {
        return blueAlliance ? "BLUE" : "RED";
    }

    // ======== Motor helpers ========
    private void moveRobot(double forward, double yaw) {
        double leftPower  = forward - yaw;
        double rightPower = forward + yaw;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower  /= max;
            rightPower /= max;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void stopShooter() {
        if (shooterOk) {
            launcher.setPower(0);
            intake.setPower(0);
        }
        if (windmillOk) windmill.setPower(0);
    }
}
