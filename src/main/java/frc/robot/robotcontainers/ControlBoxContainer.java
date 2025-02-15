  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


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

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class ControlBoxContainer extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public ControlBoxContainer() {
    
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    
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
