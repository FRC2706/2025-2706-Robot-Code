  
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    // new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
    //                                               .onFalse(new BlingCommand(BlingColour.DISABLED));

    // new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
    //         new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
    //         new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false)));
      

    /**
     * Driver Controls
     * Driver button mapping: to add
     */
    /*
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
  */  
    //vision-aid alignment    
    // driver.leftTrigger().whileTrue(CombinedCommands.centerSpeakerVisionShot(driver, PhotonPositions.FAR_SPEAKER_BLUE, PhotonPositions.FAR_SPEAKER_RED))
    //         .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
    //         .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    /**
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
    //Manipulator
    operator.y().onTrue(new CoralDepositorCommand(true)); 
    operator.x().onTrue(new CoralDepositorCommand(false));
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
