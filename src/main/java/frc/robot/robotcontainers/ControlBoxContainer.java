  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;


import static frc.lib.lib2706.ErrorCheck.errSpark;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.lib2706.TunableNumber;
import frc.lib.lib2706.XBoxControllerUtil;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.Robot;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.commands.ClimberRPM;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.CoralDepositorCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.RotateAngleToVisionSupplier;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.RumbleJoystick;
import frc.robot.commands.SetArm;
import frc.robot.commands.SubwooferShot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.SetElevator;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.auto.AutoSelector;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.CombinedCommands;

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

    
    // Set bling to for some events....
    //operator.a().onTrue(new BlingCommand(BlingColour.PURPLE)).onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
    //                                               .onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
    //         new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
    //         new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false)));
      

    //Manipulator
    operator.rightTrigger().whileTrue(new CoralDepositorCommand(true, false)); 
    operator.leftTrigger().whileTrue(new CoralDepositorCommand(false, false));
    //intake
    // operator.leftBumper().whileTrue(new CoralIntake(0.3,-0.3));
    // operator.rightBumper().whileTrue(new CoralIntake(-0.3,0.3));


    // ELEVATOR PROTOTYPE
    operator.a().onTrue(new SetElevator(Config.ElevatorSetPoints.L1));
    operator.b().onTrue(new SetElevator(Config.ElevatorSetPoints.L2));
    operator.y().onTrue(new SetElevator(Config.ElevatorSetPoints.L3));
    operator.x().onTrue(new SetElevator(Config.ElevatorSetPoints.L4));

    //operator.start().whileTrue(new ResetElevator() );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *leop
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
