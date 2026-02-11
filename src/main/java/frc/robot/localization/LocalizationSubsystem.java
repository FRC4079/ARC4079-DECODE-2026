package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;

public class LocalizationSubsystem extends StateMachine<LocalizationState> {
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  // NetworkTables publishers for pose telemetry
  private final DoubleArrayPublisher botposeBluePub;
  private final DoubleArrayPublisher robotPosePub;

  public LocalizationSubsystem(ImuSubsystem imu, VisionSubsystem vision, SwerveSubsystem swerve) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;

    // Set up NetworkTables publishers
    var nt = NetworkTableInstance.getDefault();
    botposeBluePub = nt.getTable("limelight").getDoubleArrayTopic("botpose_blue").publish();
    robotPosePub = nt.getTable("Localization").getDoubleArrayTopic("robot_pose").publish();

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo90",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo270",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
    }
  }

  public Pose2d getPose() {
    return swerve.getDrivetrainState().Pose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return swerve.drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  /** Expose the drivetrain field-relative speeds (vx, vy, omega). */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return swerve.getFieldRelativeSpeeds();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
  vision.getLeftTagResult().ifPresent(this::ingestTagResult);
  vision.getRightTagResult().ifPresent(this::ingestTagResult);

    // Publish the current pose to NetworkTables
    Pose2d pose = getPose();
    // Limelight-style botpose_blue: [X, Y, Z, Roll, Pitch, Yaw]
    botposeBluePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        0.0, // Z not tracked here
        0.0, // Roll
        0.0, // Pitch
        pose.getRotation().getDegrees()
    });

    // Custom pose array: [X, Y, ThetaRadians]
    robotPosePub.set(new double[] {
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians()
    });

  }

  private void ingestTagResult(TagResult result) {
    var visionPose = result.pose();

    if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      resetPose(visionPose);
    }
    swerve.drivetrain.addVisionMeasurement(
        visionPose, Utils.fpgaToCurrentTime(result.timestamp()), result.standardDevs());
  }

  private void resetGyro(Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle.getDegrees());
    swerve.drivetrain.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // Reset the gyro when requested in teleop
    // Otherwise, if we are in auto, only reset it if we aren't already at the correct heading
    if (DriverStation.isTeleop()
        || !MathUtil.isNear(
            estimatedPose.getRotation().getDegrees(), imu.getRobotHeading(), 1.5, -180, 180)) {
      imu.setAngle(estimatedPose.getRotation().getDegrees());
    }

    swerve.drivetrain.resetPose(estimatedPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
