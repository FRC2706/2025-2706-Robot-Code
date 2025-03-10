// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.SetElevator;
import frc.robot.commands.BlingCommand.BlingColour;
import frc.robot.subsystems.DiffTalonSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class BeetleContainer extends RobotContainer {

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public BeetleContainer() {
    // Configure the button bindings
        configureButtonBindings();
  }

  /**    
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1); 

    //driver.a().onTrue(new BlingCommand(BlingColour.HONEYDEW));


    DiffTalonSubsystem.getInstance().setDefaultCommand(
        new ArcadeDrive(driver, XboxController.Axis.kLeftY.value, XboxController.Axis.kRightX.value));
  
    // ()-> is double supplier, this makes the code repeat and continue updating every time so the speed is not a single value
    //driver.rightTrigger().whileTrue(new ClimberRPM(()->  driver.getRightTriggerAxis()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return new InstantCommand(); 
  }

}
