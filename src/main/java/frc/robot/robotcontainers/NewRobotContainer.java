  
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BlingCommand.BlingColour;
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
import frc.robot.subsystems.BlingSubsystem;
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
public class NewRobotContainer extends RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final CommandXboxController testJoystick = new CommandXboxController(2);

  /* Create Subsystems in a specific order */
  private final SwerveSubsystem s_Swerve = SwerveSubsystem.getInstance();
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  /* Auto */
  private AutoRoutines m_autoRoutines;
  private AutoSelector m_autoSelector;
  private int m_analogSelectorIndex;
  //private boolean m_bDemoMode;

  /* Demo/Normal */
  private int m_subwooferShotRpm = 0;
  private int m_subwooferShotRpmTrigger = 0;

  /* Default Command */
  private Command m_swerveDefaultCommand;

  private TunableNumber shooterTargetRPM = new TunableNumber("Shooter/Target RPM", 0);
  private TunableNumber shooterDesiredVoltage = new TunableNumber("Shooter/desired Voltage", 0);
  private TunableNumber armAngleDeg = new TunableNumber("Arm/ArmTuning/setAngleDeg", 5.0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public NewRobotContainer() {
    /*  Setup default commands */
    m_swerveDefaultCommand = new TeleopSwerve(driver);
    s_Swerve.setDefaultCommand(m_swerveDefaultCommand);

    // Setup auto
    //m_autoRoutines = new AutoRoutines();
    m_autoSelector = new AutoSelector();
    m_analogSelectorIndex = m_autoSelector.getAnalogSelectorIndex();

    System.out.println("Analog Selector Index: " + m_analogSelectorIndex);

    if(Config.demoEnabled)
    {
      //Demo mode: 
      //difference between demo and normal modes: 
      //subwoofer shooter RPM and teleop swerve speeds are different
      m_subwooferShotRpm = Config.ShooterRPM.DEMO_SUBWOOFERSHOT;
      m_subwooferShotRpmTrigger = Config.ShooterRPM.DEMO_SUBWOOFERSHOT_TRIGGER;
      TeleopSwerve.setSpeeds(TeleopSpeeds.DEMO);
    }
    else
    {
      //Normal competition mode
      m_subwooferShotRpm = Config.ShooterRPM.NORMAL_SUBWOOFERSHOT;
      m_subwooferShotRpmTrigger = Config.ShooterRPM.NORMAL_SUBWOOFERSHOT_TRIGGER;
    }

    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link CommandXboxController} or other ways.
   */
  private void configureButtonBindings() { 
    // Set bling to purple when note is in

    new Trigger(() -> intake.isBackSensorActive()).onTrue(CombinedCommands.strobeToSolidBlingCommand())
                                                  .onFalse(new BlingCommand(BlingColour.DISABLED));

    new Trigger(() -> intake.isBackSensorLongActive() && DriverStation.isTeleopEnabled()).onTrue(Commands.parallel(
            new RumbleJoystick(driver, RumbleType.kBothRumble, 0.75, 0.4, false),
            new RumbleJoystick(operator, RumbleType.kBothRumble, 0.75, 0.4, false))
    );          

    /**
     * Driver Controls
     * KingstonV1: https://drive.google.com/file/d/1gDgxnz-agWGoYmTTRfViVPwR7O2H80mh
     */
    // Core Swerve Buttons
    driver.back().onTrue(SwerveSubsystem.getInstance().setHeadingCommand(new Rotation2d(0)));

    if ( Config.demoEnabled )
    {
        driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.DEMO)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.DEMO)));

    }
    else
    {
        driver.leftBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
                            .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));
    }   

    driver.rightBumper().onTrue(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(false)))
                       .onFalse(Commands.runOnce(() -> TeleopSwerve.setFieldRelative(true)));

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



    driver.rightTrigger().whileTrue(new RotateAngleToVisionSupplier(driver, "/photonvision/" + PhotonConfig.apriltagCameraName));
    
    // Vision scoring commands with no intake, shooter, arm
    // driver.leftTrigger().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_BLUE, driver), 
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.AMP_RED, driver)));

    // driver.rightTrigger().whileTrue(new SelectByAllianceCommand( // Implement command group that also controls the arm, intake, shooter
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.RIGHT_SPEAKER_BLUE, driver), 
    //   PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.LEFT_SPEAKER_RED, driver)));

    driver.leftTrigger().whileTrue(CombinedCommands.centerSpeakerVisionShot(driver, PhotonPositions.FAR_SPEAKER_BLUE, PhotonPositions.FAR_SPEAKER_RED))
            .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
            .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

    // driver.leftTrigger().whileTrue(CombinedCommands.podiumSourceSideSpeakerVisionShot(driver, PhotonPositions.PODIUM_SOURCESIDE_BLUE, PhotonPositions.PODIUM_SOURCESIDE_RED))
    //         .onTrue(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.SLOW)))
    //         .onFalse(Commands.runOnce(() -> TeleopSwerve.setSpeeds(TeleopSpeeds.MAX)));

     /**
     * Operator Controls
     * KingstonV1: https://drive.google.com/file/d/18HyIpIeW08CC6r6u-Z74xBWRv9opHnoZ
     */
    // Arm
    operator.y().onTrue(new SetArm(()->ArmSetPoints.AMP.angleDeg)).onTrue(new IntakeControl(false).withTimeout(0.25)); // Amp
    operator.b().onTrue(new SetArm(()->ArmSetPoints.IDLE.angleDeg)); // Idle
    operator.a().onTrue(new SetArm(()->ArmSetPoints.NO_INTAKE.angleDeg)); // Pickup
    operator.x().onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));
   // Climber
    operator.leftTrigger(0.10).and(operator.back()).whileTrue(new ClimberRPM(()-> MathUtil.applyDeadband(operator.getLeftTriggerAxis(), 0.35) * 0.5));

    // Eject the note from the front with start
    operator.start()
      .whileTrue(Commands.run(() -> intake.setVoltage(-12), intake))
      .onFalse(Commands.runOnce(() -> intake.stop()));
  
       
    //operator.leftTrigger(0.3).whileTrue(
    operator.leftBumper()
      .whileTrue(CombinedCommands.armIntake())
      .onFalse(new SetArm(()->ArmSetPoints.NO_INTAKE.angleDeg))
      .onFalse(new MakeIntakeMotorSpin(9.0,0).withTimeout(1).until(() -> intake.isBackSensorActive()));

    //right trigger for shooter with speaker RPM
    operator.rightTrigger(0.3).whileTrue(CombinedCommands.simpleShootNoteSpeaker(0.4));
    //operator.rightTrigger(0.3).whileTrue(CombinedCommands.simpleShootNoteAmp());
    // Shoot note with leftBumper
    // operator.rightBumper().whileTrue(CombinedCommands.simpleShootNoteSpeaker(1))
    //                       .onTrue(new SetArm(()->ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg));

      operator.rightBumper().onTrue(new SubwooferShot(
      operator.rightBumper(), 
      ArmSetPoints.SPEAKER_KICKBOT_SHOT.angleDeg, 
      m_subwooferShotRpm, 
      m_subwooferShotRpmTrigger));
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
