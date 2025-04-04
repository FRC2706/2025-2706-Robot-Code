// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Config.ArmSetPoints;
import frc.robot.commands.SetArm;
import frc.robot.commands.Shooter_PID_Tuner;
import frc.robot.robotcontainers.BeetleContainer;
import frc.robot.robotcontainers.ContainerForTesting;
import frc.robot.robotcontainers.ControlBoxContainer;
import frc.robot.robotcontainers.NewRobotContainer;
import frc.robot.robotcontainers.Robot2025Container;
import frc.robot.robotcontainers.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    createRobotContainer();
  }


  private void createRobotContainer() {
    // Instantiate the RobotContainer based on the Robot ID.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    System.out.println(Config.getRobotId());
    switch (Config.getRobotId()) {
      case 0:
        // m_robotContainer = new ContainerForTesting(); break; // testing
        m_robotContainer = new Robot2025Container(); break; //competition

      case 1:
        m_robotContainer = new NewRobotContainer(); break; //Apollo
      case 2:
        m_robotContainer = new BeetleContainer(); break; //beetle

      case 3:
        System.out.println("ITS DEFINITELY ROBOT ID 3!!!");

        m_robotContainer = new ControlBoxContainer(); break; //ControlBox
       
      default:
        m_robotContainer = new Robot2025Container();
        DriverStation.reportError(
            String.format("ISSUE WITH CONSTRUCTING THE ROBOT CONTAINER. \n " +
                          "NewRobotContainer constructed by default. RobotID: %d", Config.getRobotId()), 
            true);
    }

     // Add CommandScheduler to shuffleboard so we can display what commands are scheduled
    ShuffleboardTab basicDebuggingTab = Shuffleboard.getTab("BasicDebugging");
    basicDebuggingTab
      .add("CommandScheduler", CommandScheduler.getInstance())
      .withPosition(3, 0)
      .withSize(3, 6);

    ShuffleboardLayout testingCommandList =
      basicDebuggingTab.getLayout("TestingCommands", BuiltInLayouts.kList)
      .withPosition(1, 0)
        .withSize(2, 6)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    if (Config.getRobotId() == 0 )
    {
      //@todo: add...
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link PoseidonContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //for robots which only have these subsystems
    if (Config.getRobotId() == 0)
    {
      SwerveSubsystem.getInstance().setVoltageCompensation(true);
      PhotonSubsystem.getInstance().resetTagAtBootup();
    }
    else if ( Config.getRobotId() == 1)
    {
      SwerveSubsystem.getInstance().setVoltageCompensation(true);
      ArmSubsystem.getInstance().resetProfiledPIDController();
      PhotonSubsystem.getInstance().resetTagAtBootup();
    }


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //ShooterSubsystem.getInstance().changeCurrentLimit(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if( Config.getRobotId()==0)
    {
      SwerveSubsystem.getInstance().setVoltageCompensation(false);
      PhotonSubsystem.getInstance().resetTagAtBootup();
    }
    else if (Config.getRobotId()==1)
    {
      SwerveSubsystem.getInstance().setVoltageCompensation(false);
      ArmSubsystem.getInstance().resetProfiledPIDController();
      PhotonSubsystem.getInstance().resetTagAtBootup();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}