package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name = "StarterBotTeleop (Blocks to Java)")
public class StarterBotTeleop extends LinearOpMode {

  private DcMotor backLeft;
  private DcMotor right_drive;
  private DcMotor _3;
  private DcMotor left_drive;
  private ColorSensor sensor;

  /**
   * Describe this function...
   */
  private void createVariables() {
    String IDLE;
    String SPIN_UP;
    String LAUNCH;
    String LAUNCHING;
    String launchState;
    int LAUNCHER_TARGET_VELOCITY;
    int LAUNCHER_MIN_VELOCITY;
    ElapsedTime launchTime;

    IDLE = "IDLE";
    SPIN_UP = "SPIN_UP";
    LAUNCH = "LAUNCH";
    LAUNCHING = "LAUNCHING";
    launchState = IDLE;
    LAUNCHER_TARGET_VELOCITY = 1125;
    LAUNCHER_MIN_VELOCITY = 1075;
    launchTime = new ElapsedTime();
  }

  /**
   * Describe this function...
   */
  private void initMotors() {
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_drive.setDirection(DcMotor.Direction.REVERSE);
    _3.setDirection(DcMotor.Direction.REVERSE);
    backLeft.setDirection(DcMotor.Direction.REVERSE);
    left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    _3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /**
   * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
   * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
   * system for robot mobility, one high-speed motor driving two "launcher wheels",
   * and two servos which feed that launcher. Likely the most niche concept we'll
   * leverage in this example is closed-loop motor velocity control. This control
   * method reads the current speed as reported by the motor's encoder and applies
   * a varying amount of power to reach, and then hold a target velocity. The FTC
   * SDK calls this control method "RUN_USING_ENCODER". This contrasts to the
   * default "RUN_WITHOUT_ENCODER" where you control the power applied to the
   * motor directly. Since the dynamics of a launcher wheel system varies greatly
   * from those of most other FTC mechanisms, we will also need to adjust the
   * "PIDF" coefficients with some that are a better fit for our application.
   */
  @Override
  public void runOpMode() {
    NormalizedRGBA normalizedColor;

    backLeft = hardwareMap.get(DcMotor.class, " backLeft");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    _3 = hardwareMap.get(DcMotor.class, "_3");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    sensor = hardwareMap.get(ColorSensor.class, "sensor");

    // Put initialization blocks here.
    init_sensors___vison();
    createVariables();
    initMotors();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) sensor).getLightDetected());
        normalizedColor = ((NormalizedColorSensor) sensor).getNormalizedColors();
        telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColor.red, 5)));
        telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColor.green, 5)));
        telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColor.blue, 5)));
        telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(normalizedColor.alpha, 5)));
        telemetry.addData("Color", Double.parseDouble(JavaUtil.formatNumber(normalizedColor.toColor(), 5)));
        telemetry.update();
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        if (gamepad1.circle) {
          right_drive.setPower(1);
          _3.setPower(1);
        } else {
          right_drive.setPower(0);
          _3.setPower(0);
        }
      }
    }
  }

  /**
   * This takes input from the joysticks, and applies power to the left and right
   * drive motor to move the robot as requested by the driver. "arcade" refers to
   * the control style we're using here. Much like a classic arcade game, when you
   * move the left joystick forward both motors work to drive the robot forward, and
   * when you move the right joystick left and right both motors work to rotate the
   * robot. Combinations of these inputs can be used to create more complex maneuvers.
   */
  private void arcadeDrive(float forward, float rotate) {
    right_drive.setPower(forward - rotate);
    backLeft.setPower(forward - rotate);
  }

  /**
   * Describe this function...
   */
  private void init_sensors___vison() {
    PredominantColorProcessor.Builder myPredominantColorProcessorBuilder;
    ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder;
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    PredominantColorProcessor myPredominantColorProcessor;
    ColorBlobLocatorProcessor myColorBlobLocatorProcessor;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;

    // Create a new PredominantColorProcessor.Builder object and assign it to a variable.
    myPredominantColorProcessorBuilder = new PredominantColorProcessor.Builder();
    myPredominantColorProcessorBuilder.setRoi(ImageRegion.entireFrame());
    myPredominantColorProcessorBuilder.setSwatches(
        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
        PredominantColorProcessor.Swatch.BLUE);
    // Build the PredominantColorProcessor and assign it to a variable.
    myPredominantColorProcessor = myPredominantColorProcessorBuilder.build();
    // Create a new ColorBlobLocatorProcessor.Builder object and assign it to a variable.
    myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
    myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY);
    myColorBlobLocatorProcessorBuilder.setTargetColorRange(ColorRange.ARTIFACT_GREEN);
    // Build the ColorBlobLocatorProcessor and assign it to a variable.
    myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();
    // Create a new AprilTagProcessor.Builder object and assign it to a variable.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    // Build the AprilTag processor and assign it to a variable.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Create a VisionPortal.Builder object so you can specify attributes about the cameras.
    myVisionPortalBuilder = new VisionPortal.Builder();
    // Set the camera to the specified webcam name.
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    // Set the stream format.
    myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
    // Enable the live camera preview.
    myVisionPortalBuilder.enableLiveView(true);
    // Set whether to automatically stop the LiveView (RC preview) when all vision processors are disabled.
    myVisionPortalBuilder.setAutoStopLiveView(true);
    // Set whether the VisionPortal should automatically start streaming
    // when you issue a .build() call on this VisionPortal.Builder object.
    myVisionPortalBuilder.setAutoStartStreamOnBuild(true);
    // Set the camera resolution.
    myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
    // Add the AprilTag processor.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Add the color blob locator processor.
    myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
    // Add the predominant color processor.
    myVisionPortalBuilder.addProcessor(myPredominantColorProcessor);
    // Build the VisionPortal object and assign it to a variable.
    myVisionPortal = myVisionPortalBuilder.build();
  }
}
