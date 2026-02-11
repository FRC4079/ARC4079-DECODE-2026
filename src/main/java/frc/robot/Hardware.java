package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final TalonFX flywheelA1 = new TalonFX(11);
  public final TalonFX flywheelA2 = new TalonFX(12);
  public final TalonFX flywheelB1 = new TalonFX(13);
  public final TalonFX flywheelB2 = new TalonFX(14);

  public final TalonFX hoodMotor = new TalonFX(21);
   public final Limelight leftLimelight =
    new Limelight("left", LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight =
    new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

  public final TalonFX intakeRollerMotor = new TalonFX(31);
  public final TalonFX intakePivotMotor = new TalonFX(32);
}
