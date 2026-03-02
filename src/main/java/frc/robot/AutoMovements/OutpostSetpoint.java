package frc.robot.AutoMovements;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;


public class OutpostSetpoint {
	public enum State {
		omwToOutpost,
		atOutpost
	}

	private final LocalizationSubsystem localization;
	private final SwerveSubsystem swerve;
	private State state = State.atOutpost;

	public OutpostSetpoint(LocalizationSubsystem localization, SwerveSubsystem swerve) {
		this.localization = localization;
		this.swerve = swerve;
	}

	public Pose2d getAllianceSetpoint() {
		return FmsSubsystem.isRedAlliance() ? FieldPoints.getOutpostRed() : FieldPoints.getOutpostBlue();
	}

	public void setRedSetpoint(Pose2d pose) { FieldPoints.setOutpostRed(pose); }
	public void setBlueSetpoint(Pose2d pose) { FieldPoints.setOutpostBlue(pose); }

	public State getState() {
		return state;
	}

	public Command travelToOutpost() {
		return Commands.runOnce(() -> state = State.omwToOutpost)
				.andThen(
						new DriveToPose(swerve, localization, this::getAllianceSetpoint)
								.withName("omwToOutpost"))
				.andThen(
						Commands.runOnce(
								() -> {
									state = State.atOutpost;
								}))
				.withName("travelToOutpost");
	}
}
