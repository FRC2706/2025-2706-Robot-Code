  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.lib2706.TunableNumber;
import frc.lib.lib2706.XBoxControllerUtil;
import frc.robot.Config;
import frc.robot.Config.ArmSetPoints;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.ClimberRPM;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralDepositorCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetArm;
import frc.robot.commands.SubwooferShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.Config.AutoConstants;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class Robot2025Container extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);

  /* Create Subsystems in a specific order */
  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
 
  /* Auto */
  private AutoRoutines m_autoRoutines;
  private AutoSelector m_autoSelector;
  private int m_analogSelectorIndex;
 
  /* Default Command */
  private Command m_swerveDefaultCommand;

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
  private TunableNumber armAngleDeg = new TunableNumber("Arm/ArmTuning/setAngleDeg", 5.0);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public Robot2025Container() {
    /*  Setup default commands */
    m_swerveDefaultCommand = new TeleopSwerve(driver);
    s_Swerve.setDefaultCommand(m_swerveDefaultCommand);
    
    // Setup auto
    m_autoRoutines = new AutoRoutines();
    m_autoSelector = new AutoSelector();
    m_analogSelectorIndex = m_autoSelector.getAnalogSelectorIndex();

    System.out.println("Analog Selector Index: " + m_analogSelectorIndex);

    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    // Set bling to for some events....

    // new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
    //                                               .onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
    //         new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
    //         new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false)));
      

    /**
     * Driver Controls
     * Driver button mapping: to add
     */
    // Core Swerve Buttons
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
    
    //slow mode
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    
    //??? 
    driver.rightBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(false)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(true)));

    //Sync Swerve
    driver.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().synchSwerve()));




    // Commands that take control of the rotation stick
    driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
    driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
    driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
    driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));   


    //for tuning the swerve
    // SwerveModuleState[] moduleStatesForwards = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    // };
    // driver.y().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesForwards, true, true)
    // ));

    // SwerveModuleState[] moduleStatesSideways = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
    // };
    // driver.x().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesSideways, true, true)
    // ));

    // SwerveModuleState[] moduleStatesBackwards = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
    // };
    // driver.a().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesBackwards, true, true)
    // ));

    // SwerveModuleState[] moduleStates270 = {
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
    //   new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
    // };
    // driver.b().whileTrue(Commands.run(
    //   () -> SwerveSubsystem.getInstance().setModuleStates(moduleStates270, true, true)
    // ));

    
    //vision-aid alignment: no timer
    // driver.leftTrigger().whileTrue(CombinedCommands.visionScoreLeftReef(driver, operator, PhotonPositions.REEF_LEFT))
    //         .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.VISION)))
    //         .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));


    //not working
    // driver.leftTrigger().whileTrue(CombinedCommands.visionScoreLeftReef(driver, operator, PhotonPositions.REEF_LEFT).withTimeout(0.9))
    // .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.VISION)))
    // .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    //This is good one: press one time: with timer
    //================================================
    driver.leftTrigger()
    .onTrue(Commands.sequence(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.VISION)), 
            CombinedCommands.visionScoreLeftReef(driver, operator).withTimeout(0.9)))
    .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));


  
    /**
     * 
     * 
     * Operator Controls
     * Operator button mapping: to add
     */
    // elevator
    // operator.y().onTrue(new SetArm(()->ArmSetPoints.AMP.angleDeg)).onTrue(new IntakeControl(false).withTimeout(0.25)); // Amp
    // operator.b().onTrue(new SetArm(()->ArmSetPoints.IDLE.angleDeg)); // Idle
    // operator.a().onTrue(new SetArm(()->ArmSetPoints.NO_INTAKE.angleDeg)); // Pickup
    // operator.x().onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));
    // Climber
    //operator.leftTrigger(0.10).and(operator.back()).whileTrue(new ClimberRPM(()-> MathUtil.applyDeadband(operator.getLeftTriggerAxis(), 0.35) * 0.5));

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int autoId = m_autoSelector.getAutoId();
    System.out.println("*********************** Auto Id"+autoId);

    return m_autoRoutines.getAutonomousCommand(autoId);
  }
}
