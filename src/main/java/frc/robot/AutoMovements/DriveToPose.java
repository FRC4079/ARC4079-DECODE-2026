package frc.robot.AutoMovements;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class DriveToPose extends Command {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static double drivekP = 1.8;
  private static double drivekD = 0.0;
  private static double thetakP = 5.0;
  private static double thetakD = 0.5;

  private static double driveMaxVelocity = 4.0;
  private static double driveMaxAcceleration = 3.5;
  private static double thetaMaxVelocity = Units.degreesToRadians(500.0);
  private static double thetaMaxAcceleration = 8.0;

  private static double driveTolerance = 0.01;
  private static double thetaTolerance = Units.degreesToRadians(1.0);

  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final Supplier<Pose2d> target;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController =
      new PIDController(0.0, 0.0, 0.0, LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), LOOP_PERIOD_SECS);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;

  static {
    SmartDashboard.putNumber("DriveToPose/DrivekP", drivekP);
    SmartDashboard.putNumber("DriveToPose/DrivekD", drivekD);
    SmartDashboard.putNumber("DriveToPose/ThetakP", thetakP);
    SmartDashboard.putNumber("DriveToPose/ThetakD", thetakD);
    SmartDashboard.putNumber("DriveToPose/DriveMaxVelocity", driveMaxVelocity);
    SmartDashboard.putNumber("DriveToPose/DriveMaxAcceleration", driveMaxAcceleration);
    SmartDashboard.putNumber("DriveToPose/DriveTolerance", driveTolerance);
    SmartDashboard.putNumber("DriveToPose/ThetaToleranceDeg", Math.toDegrees(thetaTolerance));
  }

  public DriveToPose(SwerveSubsystem swerve, LocalizationSubsystem localization, Supplier<Pose2d> target) {
    this.swerve = swerve;
    this.localization = localization;
    this.target = target;
    addRequirements(swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    running = true;
    Pose2d currentPose = localization.getPose();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = swerve.getFieldRelativeSpeeds();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    drivekP = SmartDashboard.getNumber("DriveToPose/DrivekP", drivekP);
    drivekD = SmartDashboard.getNumber("DriveToPose/DrivekD", drivekD);
    thetakP = SmartDashboard.getNumber("DriveToPose/ThetakP", thetakP);
    thetakD = SmartDashboard.getNumber("DriveToPose/ThetakD", thetakD);
    driveMaxVelocity = SmartDashboard.getNumber("DriveToPose/DriveMaxVelocity", driveMaxVelocity);
    driveMaxAcceleration = SmartDashboard.getNumber("DriveToPose/DriveMaxAcceleration", driveMaxAcceleration);
    driveTolerance = SmartDashboard.getNumber("DriveToPose/DriveTolerance", driveTolerance);
    thetaTolerance = Units.degreesToRadians(
        SmartDashboard.getNumber("DriveToPose/ThetaToleranceDeg", Math.toDegrees(thetaTolerance)));

    driveProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    driveController.setPID(drivekP, 0, drivekD);
    driveController.setTolerance(driveTolerance);
    driveController.reset();
    thetaController.setPID(thetakP, 0, thetakD);
    thetaController.setTolerance(thetaTolerance);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    Pose2d currentPose = localization.getPose();
    Pose2d targetPose = target.get();

    driveProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double minDistCorrection = 0.01;
    double setpointVelocity = direction.norm() <= minDistCorrection
        ? lastSetpointVelocity.getNorm()
        : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, -0.5);

    State driveSetpoint = driveProfile.calculate(
        LOOP_PERIOD_SECS,
        new State(direction.norm(), -setpointVelocity),
        new State(0.0, 0.0));

    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position) + driveSetpoint.velocity;
    if (driveErrorAbs < driveTolerance) driveVelocityScalar = 0.0;

    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);

    lastSetpointTranslation = new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
        .transformBy(new Transform2d(driveSetpoint.position, 0.0, Rotation2d.kZero))
        .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    double thetaSetpointVelocity =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;
    double thetaVelocity = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
        + thetaController.getSetpoint().velocity;
    if (thetaErrorAbs < thetaTolerance) thetaVelocity = 0.0;

    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    swerve.setFieldRelativeAutoSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(),
            thetaVelocity, currentPose.getRotation()));

    SmartDashboard.putNumber("DriveToPose/DriveError", driveErrorAbs);
    SmartDashboard.putNumber("DriveToPose/ThetaErrorDeg", Math.toDegrees(thetaErrorAbs));
    SmartDashboard.putBoolean("DriveToPose/AtGoal",
        withinTolerance(driveTolerance, new Rotation2d(thetaTolerance)));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
  }

  @Override
  public boolean isFinished() {
    return withinTolerance(driveTolerance, new Rotation2d(thetaTolerance));
  }

  public boolean withinTolerance(double driveToleranceMeters, Rotation2d thetaToleranceRot) {
    return running
        && driveErrorAbs < driveToleranceMeters
        && thetaErrorAbs < thetaToleranceRot.getRadians();
  }

  public boolean isRunning() {
    return running;
  }
}
