package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.AutoMovements.HeadingLock;
import frc.robot.AutoMovements.OutpostSetpoint;
import frc.robot.FlywheelSubsystem.DistanceCalc;
import frc.robot.FlywheelSubsystem.LookupTable;
import frc.robot.Intake.IntakePosition;
import frc.robot.Intake.intaker;
import frc.robot.FlywheelSubsystem.Flywheel;
import frc.robot.FlywheelSubsystem.Hood;
import frc.robot.FlywheelSubsystem.FlywheelStateMachine;
import frc.robot.FlywheelSubsystem.HoodStateMachine;
import frc.robot.IndexerSubsystem.Indexer;
import frc.robot.IndexerSubsystem.Hopper;
import frc.robot.controller.TestController;
import frc.robot.autos.AutoPoint;
import frc.robot.autos.AutoSegment;
import frc.robot.autos.Points;
import frc.robot.autos.Trailblazer;
import frc.robot.autos.constraints.AutoConstraintOptions;
import frc.robot.util.ElasticLayoutUtil;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.vision.VisionSubsystem;
import frc.robot.AutoMovements.FieldPoints;
import frc.robot.fms.FmsSubsystem;
import frc.robot.currentPhase.phaseTimer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoMovements.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;


public class Robot extends TimedRobot {
  private static final boolean ASSUME_RED_ALLIANCE = false;
  private Command autonomousCommand = Commands.none();
  private final Hardware hardware = new Hardware();

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrainPigeon);

  private final VisionSubsystem vision = new VisionSubsystem(imu, hardware.leftLimelight, hardware.rightLimelight);
  private final LocalizationSubsystem localization = new LocalizationSubsystem(imu, vision, swerve);
  private final Trailblazer trailblazer = new Trailblazer(swerve, localization);
  private final HeadingLock headingLock = new HeadingLock(localization, swerve);
  private final OutpostSetpoint outpost = new OutpostSetpoint(localization, swerve);
  private final DistanceCalc distanceCalc = new DistanceCalc(localization, headingLock);
  private final Flywheel flywheel = new Flywheel(hardware.flywheelA1, hardware.flywheelA2);
  private final Hood hood = new Hood(hardware.hoodMotor);
  private final LookupTable turretLookup = new LookupTable(distanceCalc, flywheel, hood);
  private final intaker intakeRoller = new intaker(hardware.intakeRollerMotor);
  private final IntakePosition intakePosition = new IntakePosition(hardware.intakePivotMotor);
  private final FlywheelStateMachine flywheelSM = new FlywheelStateMachine(flywheel);
  private final HoodStateMachine hoodSM = new HoodStateMachine(hood);
  private final Indexer indexer = new Indexer(hardware.indexerMotor);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);
  private final EnterNeutralZone enterNeutralZone = new EnterNeutralZone(localization, trailblazer);
  private final TestController testController = new TestController(
    hardware.testController,
    hood,
    hoodSM,
    flywheel,
    flywheelSM,
    headingLock,
    intakePosition,
    indexer,
    hopper);



  private final phaseTimer phaseTimer = new phaseTimer();
  private double tuningRpm = 3200.0;
  
  public Robot() {
    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    LifecycleSubsystemManager.ready();

    headingLock.setRedTargetPoint(FieldPoints.getHeadingLockRedPoint());
    headingLock.setBlueTargetPoint(FieldPoints.getHeadingLockBluePoint());

    headingLock.setLookupTable(turretLookup);

    registerNamedCommands();

    try {
      var ppConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
        () -> localization.getPose(),                // Pose supplier
        (pose) -> localization.resetPose(pose),      // Pose reset
        () -> swerve.getRobotRelativeSpeeds(),       // Robot-relative speeds supplier
        (speeds, feedforwards) -> swerve.setRobotRelativeAutoSpeeds(speeds), // Drive robot-relative
        new PPHolonomicDriveController(
          new PIDConstants(5.0, 0.0, 0.0),           // Translation PID
          new PIDConstants(5.0, 0.0, 0.0)            // Rotation PID
        ),
        ppConfig,
        () -> FmsSubsystem.isRedAlliance(),          // Flip for red alliance
        swerve                                       // Drive subsystem requirement
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to configure PathPlanner: " + e.getMessage(), e.getStackTrace());
    }

    configureBindings();

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    PathPlannerAuto auto = new PathPlannerAuto("New Auto");

    // Reset pose and heading to the auto's starting pose
    localization.resetPose(auto.getStartingPose());

    autonomousCommand = auto;
    CommandScheduler.getInstance().schedule(autonomousCommand);

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
           intakePosition.retract();


    ElasticLayoutUtil.onEnable();

    phaseTimer.markTeleopStart();
   // intakePosition.requestUp();
   // intakeRoller.stop();

    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(localization.getZeroCommand());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void registerNamedCommands() {
    // Indexer states
    NamedCommands.registerCommand("IndexerOff", Commands.runOnce(() -> indexer.stop()));
    NamedCommands.registerCommand("IndexerIntake", Commands.runOnce(() -> indexer.intake()));
    NamedCommands.registerCommand("IndexerFeed", Commands.runOnce(() -> indexer.feed()));
    NamedCommands.registerCommand("IndexerReverse", Commands.runOnce(() -> indexer.reverse()));

    // Hopper states
    NamedCommands.registerCommand("HopperOff", Commands.runOnce(() -> hopper.stop()));
    NamedCommands.registerCommand("HopperIntake", Commands.runOnce(() -> hopper.intake()));
    NamedCommands.registerCommand("HopperFeed", Commands.runOnce(() -> hopper.feed()));
    NamedCommands.registerCommand("HopperReverse", Commands.runOnce(() -> hopper.reverse()));

    // Intaker (roller) states
    NamedCommands.registerCommand("IntakerOff", Commands.runOnce(() -> intakeRoller.stop()));
    NamedCommands.registerCommand("IntakerIntake", Commands.runOnce(() -> intakeRoller.intake()));
    NamedCommands.registerCommand("IntakerFeed", Commands.runOnce(() -> intakeRoller.feed()));
    NamedCommands.registerCommand("IntakerReverse", Commands.runOnce(() -> intakeRoller.reverse()));

    // Intake position states
    NamedCommands.registerCommand("IntakePositionDeploy", Commands.runOnce(() -> intakePosition.deploy()));
    NamedCommands.registerCommand("IntakePositionRetract", Commands.runOnce(() -> intakePosition.retract()));

    // Flywheel states
    NamedCommands.registerCommand("ShooterOff", Commands.runOnce(() -> flywheelSM.requestOff()));
    NamedCommands.registerCommand("ShooterSpin", Commands.runOnce(() -> flywheelSM.requestRpm(3200.0)));

    // Hood states
    NamedCommands.registerCommand("HoodOff", Commands.runOnce(() -> hoodSM.requestOff()));
  }

  private void configureBindings() {
    hardware.driverController.back().onTrue(
      Commands.runOnce(() -> {
        double heading = FmsSubsystem.isRedAlliance() ? 180.0 : 0.0;
        localization.resetGyro(Rotation2d.fromDegrees(heading));
      })
    );

    
    hardware.driverController.x().whileTrue(
      Commands.run(() -> {
        flywheel.spinFlywheel(tuningRpm);
        if (flywheel.isAtGoal()) {
          indexer.setDutyPercent(1.0);
          hopper.setDutyPercent(0.7);
          intakeRoller.reverse();
        }
      }).finallyDo(() -> {
        flywheel.stop();
        indexer.stop();
        hopper.stop();
        intakeRoller.stop();
      })
    );

    // D-pad Right → increase tuning RPM by 200
    hardware.driverController.povRight().onTrue(
      Commands.runOnce(() -> {
        tuningRpm += 50;
        SmartDashboard.putNumber("Tuning/RPM", tuningRpm);
      })
    );

    // D-pad Left → decrease tuning RPM by 200
    hardware.driverController.povLeft().onTrue(
      Commands.runOnce(() -> {
        tuningRpm = Math.max(0, tuningRpm - 50);
        SmartDashboard.putNumber("Tuning/RPM", tuningRpm);
      })
    );


    hardware.driverController.leftTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          
        intakePosition.deploy();
        intakeRoller.intake();
        hopper.feed();

  
        },
        () -> {
              
        intakePosition.retract();
        intakeRoller.stop();
        hopper.stop();
    
        }
      )
    );


    
/* 
    hardware.driverController.leftTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> hood.runDutyCycle(0.3),
        () -> hood.stopMotor()
      )
    );

      hardware.driverController.leftBumper().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> hood.runDutyCycle(-0.3),
        () -> hood.stopMotor()
      )
    );

    // D-pad Down → deploy elevator + turn intake rollers on
    hardware.driverController.povDown().onTrue(
      Commands.runOnce(() -> {
        intakePosition.deploy();
        intakeRoller.intake();
      })
    );

    // D-pad Up → retract elevator + turn intake rollers off
    hardware.driverController.povUp().onTrue(
      Commands.runOnce(() -> {
        intakePosition.retract();
        intakeRoller.stop();
      })
    );
    */

    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  if (DriverStation.isTeleop()) {
                    swerve.driveTeleop(
                        hardware.driverController.getLeftX(),
                        hardware.driverController.getLeftY(),
                        hardware.driverController.getRightX());
                  }
                })
            .withName("DefaultSwerveCommand"));
/* 
    hardware.driverController.rightTrigger().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          intakePosition.requestDown();
          intakeRoller.intake();
          SmartDashboard.putBoolean("Driver/IntakeIsDown", true);
        },
        () -> {
          intakePosition.requestUp();
          intakeRoller.stop();
          SmartDashboard.putBoolean("Driver/IntakeIsDown", false);
        }
      )
    );
*/
    hardware.driverController.rightTrigger(0.1).whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          turretLookup.enable();
          headingLock.enableForAlliance();
          intakeRoller.intake();
          hopper.feed();
          SmartDashboard.putBoolean("Driver/ShootingActive", true);
        },
        () -> {
          turretLookup.disable();
          headingLock.disableLock();
          flywheelSM.requestOff();
          hoodSM.requestOff();
          indexer.stop();
          hopper.stop();
          SmartDashboard.putBoolean("Driver/ShootingActive", false);
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          if (!turretLookup.hasCachedParameters()) return;
          double rpm = turretLookup.getCachedFlywheelRpm();
          double hoodRad = turretLookup.getCachedHoodAngleRad();
          flywheelSM.requestRpm(rpm);
          hoodSM.requestDegrees(Math.toDegrees(hoodRad));
          if (flywheel.isAtGoal() && headingLock.isSettled()) {
            indexer.feed();
            hopper.feed();
          }
        })
      )
    );

    hardware.driverController.y().whileTrue(
      Commands.defer(() ->
        outpost.travelToOutpost()
          .until(() -> {
            double stickMagnitude = Math.hypot(
              hardware.driverController.getLeftX(),
              hardware.driverController.getLeftY());
            return stickMagnitude > 0.3;
          })
          .withName("DriveToOutpost"),
        java.util.Set.of(swerve))
    );
/* 
    hardware.driverController.y().whileTrue(
      edu.wpi.first.wpilibj2.command.Commands.startEnd(
        () -> {
          turretLookup.enable();
          SmartDashboard.putBoolean("Driver/ShootingActive", true);
        },
        () -> {
          turretLookup.disable();
          flywheelSM.requestOff();
          hoodSM.requestOff();
          indexer.stop();
          hopper.stop();
          SmartDashboard.putBoolean("Driver/ShootingActive", false);
        }
      ).alongWith(
        edu.wpi.first.wpilibj2.command.Commands.run(() -> {
          if (!turretLookup.cachedParametersValid()) return;
          double rpm = turretLookup.getCachedFlywheelRpm();
          double hoodRad = turretLookup.getCachedHoodAngleRad();
          flywheelSM.requestRpm(rpm);
          hoodSM.requestDegrees(Math.toDegrees(hoodRad));
          indexer.feed();
          hopper.feed();
        })
      )
    );  */
  }
    
}