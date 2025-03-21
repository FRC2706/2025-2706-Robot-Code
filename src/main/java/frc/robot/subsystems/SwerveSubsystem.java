package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
//mport com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.AdvantageUtil;
import frc.lib.lib2706.PoseBuffer;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.lib.lib2706.UpdateSimpleFeedforward;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.Swerve;

public class SwerveSubsystem extends SubsystemBase {
  private final Pigeon2 gyro;
  private Pigeon2Configuration pigeonConfig; 
  private StatusSignal<Angle> gyroYaw;
  private StatusSignal<Angle> gyroPitch;
  private StatusSignal<Angle> gyroRoll;
  

  private BuiltInAccelerometer rioAccelerometer;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  String tableName = "SwerveChassis";
  private NetworkTable swerveTable = NetworkTableInstance.getDefault().getTable(tableName);
  private DoublePublisher pubCurrentAngle = swerveTable.getDoubleTopic("Current angle (deg)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionX = swerveTable.getDoubleTopic("Current positionX (m) ").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubCurrentPositionY = swerveTable.getDoubleTopic("Current positionY (m) ").publish(PubSubOption.periodic(0.02));
  private DoubleArrayPublisher pubCurrentPose = swerveTable.getDoubleArrayTopic("Pose ").publish(PubSubOption.periodic(0.02));
  private UpdateSimpleFeedforward updateFeedforward;
  private DoublePublisher pubGyroRate = swerveTable.getDoubleTopic("Gyro Rate (degps)").publish(PubSubOption.periodic(0.02));

  private NetworkTable visionPidTable = swerveTable.getSubTable("VisionPid");
  private DoublePublisher pubMeasuredSpeedX = visionPidTable.getDoubleTopic("MeasuredSpeedX (mps)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubMeasuredSpeedY = visionPidTable.getDoubleTopic("MeasuredSpeedY (mps)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubMeasuredSpeedRot = visionPidTable.getDoubleTopic("MeasuredSpeedRot (radps)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubDesiredX = visionPidTable.getDoubleTopic("DesiredX (m)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubDesiredY = visionPidTable.getDoubleTopic("DesiredY (m)").publish(PubSubOption.periodic(0.02));
  private DoublePublisher pubDesiredRot = visionPidTable.getDoubleTopic("DesiredRot (deg)").publish(PubSubOption.periodic(0.02));

  // DataLog log = DataLogManager.getLog(); 
  // private DoubleLogEntry pubPigeonAccelX = new DoubleLogEntry(log, "Acceleratometers/Pigeon1/X (G)");
  // private DoubleLogEntry pubPigeonAccelY = new DoubleLogEntry(log, "Acceleratometers/Pigeon1/Y (G)");
  // private DoubleLogEntry pubPigeonAccelZ = new DoubleLogEntry(log, "Acceleratometers/Pigeon1/Z (G)");

  // private DoubleLogEntry pubRioAccelX = new DoubleLogEntry(log, "Acceleratometers/RoboRio/X (G)");
  // private DoubleLogEntry pubRioAccelY = new DoubleLogEntry(log, "Acceleratometers/RoboRio/Y (G)");
  // private DoubleLogEntry pubRioAccelZ = new DoubleLogEntry(log, "Acceleratometers/RoboRio/Z (G)");

  // ProfiledPIDControllers for the pid control
  ProfiledPIDController pidControlX;
  double currentX;
  double desiredX;
  ProfiledPIDController pidControlY;
  double currentY;
  double desiredY;
  ProfiledPIDController pidControlRotation;
  double currentRotation;
  double desiredRotation;
  int tempSynchCounter = 0;
  boolean recievedPidInstruction = false;

  /**
   * Counter to synchronize the modules relative encoder with absolute encoder when not moving.
   */
  private int moduleSynchronizationCounter = 0;
  
  private Field2d field;

  private PoseBuffer poseBuffer;

  private static SwerveSubsystem instance;
  public static SwerveSubsystem getInstance(){
      if(instance == null){
        SubsystemChecker.subsystemConstructed(SubsystemType.SwerveSubsystem);
        instance = new SwerveSubsystem();
      }
      return instance;
  }

  private SwerveSubsystem() {
    gyro = new Pigeon2(Swerve.pigeonID);
    pigeonConfig = new Pigeon2Configuration();
    //todo: This Pigeon is mounted position
    //todo: calibration for roll and pitch
    //todo: check yaw inverted or not
    pigeonConfig.MountPose.MountPoseYaw = 0;
    pigeonConfig.MountPose.MountPosePitch = 0;
    pigeonConfig.MountPose.MountPoseRoll = 0;
    // This Pigeon has no need to trim the gyro
    pigeonConfig.GyroTrim.GyroScalarX = 0;
    pigeonConfig.GyroTrim.GyroScalarY = 0;
    pigeonConfig.GyroTrim.GyroScalarZ = 0;
    // We want the thermal comp and no-motion cal enabled, with the compass disabled for best behavior
    pigeonConfig.Pigeon2Features.DisableNoMotionCalibration = false;
    pigeonConfig.Pigeon2Features.DisableTemperatureCompensation = false;
    pigeonConfig.Pigeon2Features.EnableCompass = false;
     // Write these configs to the Pigeon2
    gyro.getConfigurator().apply(pigeonConfig);
    //set the yaw to 0 degrees for intiial use
    //gyro.setYaw(0);
    gyroYaw = gyro.getYaw();
    gyroPitch = gyro.getPitch();
    gyroRoll = gyro.getRoll();

    rioAccelerometer = new BuiltInAccelerometer();    

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Swerve.Mod0.constants,"FL"),
          new SwerveModule(1, Swerve.Mod1.constants,"FR"),
          new SwerveModule(2, Swerve.Mod2.constants,"BL"),
          new SwerveModule(3, Swerve.Mod3.constants,"BR")
        };

    swerveOdometry = new SwerveDriveOdometry(Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d() );


    //Auto pathplanner configuration
     RobotConfig config = null;
     try {
       config = RobotConfig.fromGUISettings();
     } catch (Exception e) {
       // Handle exception as needed
      e.printStackTrace();
     }
      

    //Please make sure these numbers are good. CUrrent values are dummy values.
    // nominalVoltageVolts,  stallTorqueNewtonMeters,  stallCurrentAmps,  freeCurrentAmps, double freeSpeedRadPerSec, int numMotors
   /* DCMotor dcMotor = new DCMotor(12.0, 1.0, 1.0, 1.0, 1.0, 1);
    ModuleConfig moduleConfig = new ModuleConfig(0.049,3.0,1.20,dcMotor,50,1);
    Translation2d[] offsets = new Translation2d[4];
    //in the order of FL, FR, BL, BR, refer to Swerve.swerveKinematics. todo: double check the order
    offsets[0] = new Translation2d(Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0);//FL
    offsets[1] = new Translation2d(Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0);//FR
    offsets[2] = new Translation2d(-Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0);//BL
    offsets[3] = new Translation2d(-Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0);//BR

    RobotConfig config = new RobotConfig(50, 6.88, moduleConfig, offsets ); */

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier (MUST BE ROBOT RELATIVE)
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method to drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // Path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    poseBuffer = new PoseBuffer();

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    pidControlX = new ProfiledPIDController(7, 0.5, 0.2,
            new TrapezoidProfile.Constraints(1.5, 2.0));
    pidControlY = new ProfiledPIDController(7, 0.5, 0.2,
            new TrapezoidProfile.Constraints(1.5, 2.0));
    pidControlRotation = new ProfiledPIDController(5.0, 0.5, 0.3,
            new TrapezoidProfile.Constraints(8 * Math.PI, 8 * Math.PI));
            pidControlRotation.enableContinuousInput(-Math.PI, Math.PI);


    pidControlX.setIZone(0.3);
    pidControlY.setIZone(0.3);
    pidControlRotation.setIZone(Math.toRadians(3));

    updateFeedforward = new UpdateSimpleFeedforward((ff) -> updateModuleFeedforward(ff), swerveTable, Swerve.driveKS, Swerve.driveKV, Swerve.driveKA);
  }

  public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop, boolean flipForAlliance) {
    Rotation2d heading = flipForAlliance ? rotateForAlliance(getHeading()) : getHeading();

    SwerveModuleState[] swerveModuleStates =
    Swerve.swerveKinematics.toSwerveModuleStates(
      // ChassisSpeeds.discretize(
        fieldRelative ? 
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading) :
            speeds
            // , 0.02)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void drive(
      ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop) {
        drive(speeds, fieldRelative, isOpenLoop, true);
      }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    setModuleStates(desiredStates, isOpenLoop, false);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean isDisableAntiJitter) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop, isDisableAntiJitter);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void updateModuleFeedforward(SimpleMotorFeedforward ff) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setFeedforward(ff);
    }
  }

  //@todo: installation of Pigeon2
  private Rotation2d getYaw() {
    return (Swerve.invertGyro)
        ? Rotation2d.fromDegrees(180+ gyroYaw.refresh().getValueAsDouble())
        : Rotation2d.fromDegrees(gyroYaw.refresh().getValueAsDouble());
  }
  
  /**
   * Returns a command to set the given angle as the heading.
   * Rotates the angle by 180 degrees if on the red alliance.
   * 
   * @param angle to set for the blue alliance
   * @return Command to reset the heading
   */
  public Command setHeadingCommand(Rotation2d angle) {
    return Commands.runOnce(
      () -> resetOdometry(
        new Pose2d(
          getPose().getTranslation(), 
          rotateForAlliance(angle)))
    );
  }

  public Command setOdometryCommand(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose));
  }
  public Command setLockWheelsInXCommand() {
    return run(() -> setModuleStates(
      new SwerveModuleState[]{
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
      }, true, true)
    );
  }
  public Command getDriveToPoseCommand(Pose2d desiredPose) {
    return runOnce(() -> resetDriveToPose())
          .andThen(run(() -> driveToPose(desiredPose)))
          .until(() -> isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE));
  }

  // Swerve actual driving methods
  public void resetDriveToPose() {
    recievedPidInstruction = false;

    currentX = getPose().getX();
    currentY = getPose().getY();
    currentRotation = getPose().getRotation().getRadians();

    desiredX = getPose().getX();
    desiredY = getPose().getY();
    desiredRotation = getPose().getRotation().getRadians();

    ChassisSpeeds speeds = getFieldRelativeSpeeds();
    pidControlX.reset(getPose().getX(), speeds.vxMetersPerSecond);
    pidControlY.reset(getPose().getY(), speeds.vyMetersPerSecond);
    pidControlRotation.reset(getPose().getRotation().getRadians(), speeds.omegaRadiansPerSecond);
  }

  /**
   * Calculate the pid value for rotating the chassis to the desired angle
   * 
   * @param desiredAngle Desired angle for the chassis
   * @return The pid value to pass to rotation in the drive method
   */
  public double calculateRotation(Rotation2d desiredAngle) {
    return pidControlRotation.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), desiredAngle.getRadians());
  }

  public void driveToPose(Pose2d pose) {
    //update the currentX and currentY
    
    currentX = getPose().getX();
    currentY = getPose().getY();
    currentRotation = getPose().getRotation().getRadians();

    desiredX = pose.getX();
    desiredY = pose.getY();
    desiredRotation = pose.getRotation().getRadians();

    double xSpeed = 0;
    double ySpeed = 0;
    double rotSpeed = 0;

    if (Math.abs(currentX - desiredX) > Swerve.translationAllowableError) {
      xSpeed = pidControlX.calculate(currentX, desiredX);
    }

    if (Math.abs(currentY - desiredY) > Swerve.translationAllowableError) {
      ySpeed = pidControlY.calculate(currentY, desiredY);
    }

    if (Math.abs(currentRotation - desiredRotation) > Swerve.rotationAllowableError) {
      rotSpeed = pidControlRotation.calculate(currentRotation, desiredRotation);
    }

    pubDesiredX.accept(pidControlX.getSetpoint().position);
    pubDesiredY.accept(pidControlY.getSetpoint().position);
    pubDesiredRot.accept(Math.toDegrees(pidControlRotation.getSetpoint().position));

    recievedPidInstruction = true;
    drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed), true, true, false);
  }

  public boolean isAtPose(double tol, double angleTol) {
    return recievedPidInstruction 
        && Math.abs(currentX - desiredX) < tol && Math.abs(currentY - desiredY) < tol
        && Math.abs(MathUtil.angleModulus(currentRotation - desiredRotation)) < angleTol;
  }

  public boolean isAtTargetPose(Translation2d targetPos)
  {
    return (Math.abs(currentX - targetPos.getX()) < PhotonConfig.POS_TOLERANCE && Math.abs(currentY - targetPos.getY()) < PhotonConfig.POS_TOLERANCE);
  }
  /**
   * Get a pose at the given timestamp. 
   * Returns an empty Optional if the buffer is empty or doesn't go back far enough.
   * 
   * @param timestampSeconds The timestamp for the pose to get, matching WPILib PoseEstimator's 
   *                         timestamps (which matches PhotonVision and Limelight)
   * @return An Optional of the Pose2d or an empty Optional.
   */
  public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
    return(poseBuffer.getPoseAtTimestamp(timestampSeconds));
  }

  @Override
  public void periodic() {
    for (SwerveModule mod : mSwerveMods) {
        mod.periodic();
    }
    SwerveModulePosition[] tempGetPositions = getPositions();
    SwerveModuleState[] tempGetStates = getStates();
    swerveOdometry.update(getYaw(), tempGetPositions);
    field.setRobotPose(getPose());

  
    // If the robot isn't moving synchronize the encoders every 100ms (Inspired by democrat's SDS
    // lib)
    // To ensure that everytime we initialize it works.
    if (DriverStation.isDisabled() && !isChassisMoving(0.01) && !areModulesRotating(2)) {
      if (++moduleSynchronizationCounter > 6 && isSwerveNotSynched()) {
        synchSwerve();
        System.out.println("Resynced" + ++tempSynchCounter);
        moduleSynchronizationCounter = 0;
      }
    }
    else {
      moduleSynchronizationCounter = 0;
    }

    poseBuffer.addPoseToBuffer(getPose());

    pubCurrentAngle.accept(getPose().getRotation().getDegrees());
    pubCurrentPositionX.accept(getPose().getX());
    pubCurrentPositionY.accept(getPose().getY());
    pubCurrentPose.accept(AdvantageUtil.deconstruct(getPose()));
    
    pubGyroRate.accept(getAngularRate());

    updateFeedforward.checkForUpdates();
    ChassisSpeeds speeds = getFieldRelativeSpeeds();
    pubMeasuredSpeedX.accept(speeds.vxMetersPerSecond);
    pubMeasuredSpeedY.accept(speeds.vyMetersPerSecond);
    pubMeasuredSpeedRot.accept(Math.toDegrees(speeds.omegaRadiansPerSecond));

    logAcceleratometerData();
  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    ChassisSpeeds targetSpeeds = robotRelativeSpeeds;

    SwerveModuleState[] targetStates = Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates, false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public double getAngularRate() {
    double[] xyz_dps = new double[3];
    //todo
    //gyro.getRawGyro(xyz_dps);
    
    return xyz_dps[2];
  }

  public ChassisSpeeds getFieldRelativeSpeeds()
  {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
  }

  public boolean isChassisMoving(double velToleranceMPS)
  {
    double sumVelocity = 0;
    for (SwerveModule mod : mSwerveMods) {
      sumVelocity += Math.abs(mod.getState().speedMetersPerSecond);
    }

    if (sumVelocity <= velToleranceMPS) {
      return false;
    }

    else
    {
      return true;
    }
  }

  public boolean areModulesRotating(double angleTolerance) {
    double angularVelocitySum = 0;
    for (SwerveModule module : mSwerveMods) {
      angularVelocitySum += module.getSteeringVelocity();
    }

    return angularVelocitySum > Math.toRadians(angleTolerance);
  }

  public boolean isSwerveNotSynched() {
    for (SwerveModule module : mSwerveMods) {
      if (!module.isModuleSynced()) {
        return(true);
      }
    }
    return(false);
  }
  public void synchSwerve() {
    for (SwerveModule module : mSwerveMods) {
      module.resetToAbsolute();
    }
  }

  public void stopMotors() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopMotors();
    }
  }

  /**
   * Enable/disable voltage compensation on the drive motors. 
   * 
   * @param enable True to enable, false to disable.
   */
  public void setVoltageCompensation(boolean enable) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setVoltageCompensation(enable);
    }
  }

  public void logAcceleratometerData() {
    short[] xyz_dps = new short[]{0, 0, 0};
    //gyro.getBiasedAccelerometer(xyz_dps);

    // pubPigeonAccelX.append(xyz_dps[0] / 16384);
    // pubPigeonAccelY.append(xyz_dps[1] / 16384);
    // pubPigeonAccelZ.append(xyz_dps[2] / 16384);

    // pubRioAccelX.append(rioAccelerometer.getX());
    // pubRioAccelY.append(rioAccelerometer.getY());
    // pubRioAccelZ.append(rioAccelerometer.getZ());
  }

  /**
   * Rotates the given angle by 180 if the red alliance or 0 if blue and returns it.
   * Aka defaults to assuming we are on the blue alliance.
   * 
   * @param angle to rotate.
   * @return The angle rotated for the alliance.
   */
  public static Rotation2d rotateForAlliance(Rotation2d angle){
    var alliance = DriverStation.getAlliance();

    // Default to blue alliance
    if (alliance.isEmpty()) {
      DriverStation.reportWarning("Unable to detect alliance color.", false);
      return angle;
    }

    if (alliance.get() == DriverStation.Alliance.Blue) {
      return angle;
    } else {
      return angle.rotateBy(new Rotation2d(Math.PI));
    }
  }
}
