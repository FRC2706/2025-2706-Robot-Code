  
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
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.*;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.*;
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
    //operator.a().onTrue(new BlingCommand(BlingColour.PURPLE)).onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
    //                                               .onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
    //         new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
    //         new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false)));


    //Driver
    //=========================================================================
    /**
     * Driver Controls
     * Driver button mapping: to add
     */
    // Core Swerve Buttons
    //back is left side
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));

    //slow mode
    driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    //??? 
    driver.rightBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(false)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(true)));

    //Sync Swerve
    //start is right side
    driver.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().synchSwerve()));

    // Commands that take control of the rotation stick
    driver.y().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(0)));
    driver.x().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(90)));
    driver.a().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(180)));
    driver.b().whileTrue(new RotateToAngle(driver, Rotation2d.fromDegrees(270)));

    //for tuning the swerve
    /*SwerveModuleState[] moduleStatesForwards = {
       new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
    };
     driver.y().whileTrue(Commands.run(
       () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesForwards, true, true)
     ));

     SwerveModuleState[] moduleStatesSideways = {
       new SwerveModuleState(0, Rotation2d.fromDegrees(90)), new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
     };
     driver.x().whileTrue(Commands.run(
       () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesSideways, true, true)
     ));

     SwerveModuleState[] moduleStatesBackwards = {
       new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(180)),
     };
     driver.a().whileTrue(Commands.run(
       () -> SwerveSubsystem.getInstance().setModuleStates(moduleStatesBackwards, true, true)
     ));

     SwerveModuleState[] moduleStates270 = {
       new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
       new SwerveModuleState(0, Rotation2d.fromDegrees(270)),
     };
     driver.b().whileTrue(Commands.run(
       () -> SwerveSubsystem.getInstance().setModuleStates(moduleStates270, true, true)
     ));
     */

    // Right trigger because it is hard coded to work for the right corals (no camera for left)
    driver.rightTrigger().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.VISION)))
        .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    driver.rightTrigger().onTrue(Commands.runOnce(() -> PhotonSubsystem.getInstance().reset())); // Re-acquire target every time button is pressed
    driver.rightTrigger().and(() -> PhotonSubsystem.getInstance().hasData()) // Run vision command while button is pressed down AND a target is found
        .whileTrue(Commands.deadline(
            new PhotonMoveToTarget(false, false, false),
            new BlingCommand(BlingColour.BLUESTROBE)));
    driver.rightTrigger().onFalse(new BlingCommand(BlingColour.DISABLED));

    //Operator
    //===========================================================================
    //control Algae
    operator.rightTrigger().whileTrue(new AlgaeCommand(90, 1));
    //rescue: reverse the depositor
    operator.leftTrigger().whileTrue(new CoralDepositorCommand(false, false));
       
    //intake
    operator.leftBumper().whileTrue(CombinedCommands.getCoralForScore());
    //score the coral
    operator.rightBumper().whileTrue(new CoralDepositorCommand(true, false));   
    
    //elevator 
    operator.a().onTrue(new SetElevator(Config.ElevatorSetPoints.L1));
    operator.b().onTrue(new SetElevator(Config.ElevatorSetPoints.L2));
    operator.x().onTrue(new SetElevator(Config.ElevatorSetPoints.L3));
    operator.y().onTrue(new SetElevator(Config.ElevatorSetPoints.L4));
    //start is right side: going down
    operator.start().whileTrue(new ResetElevator(-0.15) );
    //back is left side: going up
    operator.back().whileTrue(new ResetElevator(0.15) );

    new Trigger(() -> CoralDepositorSubsystem.getInstance().isSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
                                                  .onFalse(new BlingCommand(BlingColour.DISABLED));

    //@todo: check the elevator position when free run


    // Algae remover
    //operator.y().whileTrue(new MoveAlgae(0.5));
    //operator.a().whileTrue(new MoveAlgae(-0.5));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int autoId = m_autoSelector.getAutoId();
    System.out.println("*********************** Auto Id"+autoId);
    //@todo: to put to the newwork table

    return m_autoRoutines.getAutonomousCommand(autoId);
  }
}
