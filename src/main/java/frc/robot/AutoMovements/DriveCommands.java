package frc.robot.AutoMovements;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

/**
 * Utility class with static helper methods for drive-related commands.
 * Adapted from Littleton Robotics / Mechanical Advantage.
 */
public class DriveCommands {
  public static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /** Apply deadband and squaring to joystick inputs, returning a Translation2d linear velocity. */
  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /** Apply deadband and squaring to a single omega joystick axis. */
  public static double getOmegaFromJoystick(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, DEADBAND);
    return omega * omega * Math.signum(omega);
  }

  /**
   * Creates a field-relative or robot-relative drive command using joystick suppliers.
   *
   * @param swerve         the swerve subsystem
   * @param localization   the localization subsystem
   * @param xSupplier      left-right joystick axis
   * @param ySupplier      forward-back joystick axis
   * @param omegaSupplier  rotation joystick axis
   * @param maxLinearSpeed max translational speed in m/s
   * @param maxAngularSpeed max rotational speed in rad/s
   * @param robotRelative  whether to drive robot-relative
   */
  public static Command joystickDrive(
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double maxLinearSpeed,
      double maxAngularSpeed,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(maxLinearSpeed);
          double omega = getOmegaFromJoystick(omegaSupplier.getAsDouble()) * maxAngularSpeed;

          ChassisSpeeds speeds =
              new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega);

          if (!robotRelative.getAsBoolean()) {
            Rotation2d rotation = localization.getPose().getRotation();
            if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red) {
              rotation = rotation.plus(Rotation2d.kPi);
            }
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation);
          }

          swerve.setRobotRelativeAutoSpeeds(speeds);
        },
        swerve);
  }
}
