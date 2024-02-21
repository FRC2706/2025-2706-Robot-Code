  
  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import static frc.robot.subsystems.IntakeStates.Modes.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.TunableNumber;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.Robot;
import frc.robot.commands.Shooter_tuner;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.subsystems.ArmPneumaticsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods
 * (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class NewRobotContainer extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
    

  /* Create Subsystems in a specific order */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {

    /*  Setup default commands */
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            driver,
            TeleopSpeeds.MAX
        )
    );

    intake.setDefaultCommand(intake.autoIntake());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    /* --------------- Driver Controls -------------------- */
    driver.start().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));
  
    driver.back().whileTrue(SwerveSubsystem.getInstance().setLockWheelsInXCommand());
    driver.leftBumper().whileTrue(new TeleopSwerve(
        s_Swerve,
        driver,
        TeleopSpeeds.SLOW
    ));

    /* --------------- Operator Controls -------------------- */
    operator.y() //Manually turn on the shooter and get voltage from DS
      .whileTrue(new Shooter_tuner(shooterDesiredVoltage.get()));

    operator.a() //Intake the Note
      .whileTrue(Commands.runOnce(()-> intake.setMode(INTAKE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    

    operator.b() //Release the Note from the back
      .whileTrue(Commands.runOnce(()-> intake.setMode(RELEASE)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    

    operator.x() //Drives the note into the shooter
      .whileTrue(Commands.runOnce(()-> intake.setMode(SHOOT)))
      .whileFalse(Commands.runOnce(()->intake.setMode(STOP)));    

    operator.start() //Shoots the Note automatically 
      .onTrue(Commands.deadline(
        Commands.sequence(
          Commands.waitSeconds(2), 
          intake.shootNote())
          ,new Shooter_tuner(12)
      ));

    //turns brakes off
    operator.rightBumper().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(false, true)));

    //turns brakes on
    operator.rightTrigger().onTrue(Commands.runOnce(() -> ArmPneumaticsSubsystem.getInstance().controlBrake(true, true)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoRoutines().getAutonomousCommand(2);
  }
}