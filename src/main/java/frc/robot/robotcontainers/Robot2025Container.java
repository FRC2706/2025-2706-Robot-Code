  
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
import frc.robot.commands.*;
import frc.robot.commands.BlingCommand.BlingColour;
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
    //operator.a().onTrue(new BlingCommand(BlingColour.PURPLE)).onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
    //                                               .onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
    //         new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
    //         new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false)));


    //Manipulator
    operator.rightTrigger().whileTrue(new CoralDepositorCommand(true));
    operator.leftTrigger().whileTrue(new CoralDepositorCommand(false));
    //intake
    operator.leftBumper().whileTrue(new CoralIntake(0.3,-0.3));
    operator.rightBumper().whileTrue(new CoralIntake(-0.3,0.3));


    // ELEVATOR PROTOTYPE
    operator.a().onTrue(new SetElevator(Config.ElevatorSetPoints.L1));
    operator.b().onTrue(new SetElevator(Config.ElevatorSetPoints.L2));
    operator.y().onTrue(new SetElevator(Config.ElevatorSetPoints.L3));
    operator.x().onTrue(new SetElevator(Config.ElevatorSetPoints.L4));

    operator.start().whileTrue(new ResetElevator() );
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
