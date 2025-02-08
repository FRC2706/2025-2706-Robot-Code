package frc.robot.subsystems;

//import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.lib3512.config.SwerveModuleConstants;
import frc.lib.lib3512.util.CANCoderUtil;
import frc.lib.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib.lib3512.util.CANSparkMaxUtil;
import frc.lib.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.Config;
import frc.robot.Robot;

public class SwerveModule {

  private NetworkTable swerveModuleTable;
  private NetworkTable swerveTable;
  private DoublePublisher currentSpeedEntry;
  private DoublePublisher currentAngleEntry;
  private DoublePublisher speedError;
  private DoublePublisher angleError;
  private DoublePublisher desiredSpeedEntry;
  private DoublePublisher desiredAngleEntry;
  private DoubleEntry entryAngleOffset;
  private DoublePublisher canCoderAngleEntry;

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkMax angleMotor;
  private SparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;
  private CANcoderConfiguration angleEncoderConfig;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;
  SparkMaxConfig driveMotorConfig;
  SparkMaxConfig angleMotorConfig;

  private boolean synchronizeEncoderQueued = false;

  private SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
        Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, String ModuleName) {
    this.moduleNumber = moduleNumber;

    angleOffset = moduleConstants.angleOffset;

    String tableName = "SwerveChassis/SwerveModule" + ModuleName;
    swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);
    swerveTable = NetworkTableInstance.getDefault().getTable("SwerveChassis");
  
    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    angleEncoderConfig = new CANcoderConfiguration();
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    configDriveMotor();

    lastAngle = getState().angle;

    desiredSpeedEntry = swerveModuleTable.getDoubleTopic("Desired speed (mps)").publish(PubSubOption.periodic(0.02));
    desiredAngleEntry = swerveModuleTable.getDoubleTopic("Desired angle (rad)").publish(PubSubOption.periodic(0.02));
    currentSpeedEntry = swerveModuleTable.getDoubleTopic("Current speed (mps)").publish(PubSubOption.periodic(0.02));
    currentAngleEntry = swerveModuleTable.getDoubleTopic("Current angle (rad)").publish(PubSubOption.periodic(0.02));
    speedError = swerveModuleTable.getDoubleTopic("Speed error (mps)").publish(PubSubOption.periodic(0.02));
    angleError = swerveModuleTable.getDoubleTopic("Angle error (deg)").publish(PubSubOption.periodic(0.02));
    entryAngleOffset = swerveModuleTable.getDoubleTopic("Angle Offset (deg)").getEntry(angleOffset.getDegrees());
    canCoderAngleEntry = swerveModuleTable.getDoubleTopic("Cancoder (deg)").publish(PubSubOption.periodic(0.02));
    
    entryAngleOffset.accept(angleOffset.getDegrees());

    resetToAbsolute();

    ErrorTrackingSubsystem.getInstance().register(angleMotor);
    ErrorTrackingSubsystem.getInstance().register(driveMotor);

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    setDesiredState(desiredState, isOpenLoop, false);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isDisableAntiJitter) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    
    setAngle(desiredState, isDisableAntiJitter);
    setSpeed(desiredState, isOpenLoop);


    desiredAngleEntry.accept(desiredState.angle.getRadians());
    desiredSpeedEntry.accept(desiredState.speedMetersPerSecond);
    speedError.accept((desiredState.speedMetersPerSecond)-(driveEncoder.getVelocity()));
    angleError.accept((desiredState.angle.getRadians())-(getAngle().getRadians()));
  }

  /**
   * Resets Position Encoder
   */
  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - angleOffset.getRadians();
    integratedAngleEncoder.setPosition(absolutePosition);
    lastAngle = getAngle();
 
    System.out.println("ModuleName resetAngle (rad)"+ absolutePosition);
  }

  private void configAngleEncoder() {
   
    //CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
   
    MagnetSensorConfigs magnetCfg = new MagnetSensorConfigs()
                                    .withMagnetOffset(0)
                                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) //Config.Swerve.canCoderInvert
                                    .withAbsoluteSensorDiscontinuityPoint(1);//Unsigned_0_to_360
        
    angleEncoderConfig.withMagnetSensor(magnetCfg);
    angleEncoder.getConfigurator().apply(angleEncoderConfig);
  }

  private void configAngleMotor() {
    angleMotorConfig = new SparkMaxConfig();
    angleMotor.setCANTimeout(Config.CANTIMEOUT_MS);

    angleMotorConfig.smartCurrentLimit(Config.Swerve.angleContinuousCurrentLimit);
    angleMotorConfig.inverted(Config.Swerve.angleInvert);
    angleMotorConfig.idleMode(Config.Swerve.angleNeutralMode);
    angleMotorConfig.voltageCompensation(Config.Swerve.voltageComp);
    
    angleMotorConfig.encoder.positionConversionFactor(Config.Swerve.angleConversionFactor);
    angleMotorConfig.encoder.velocityConversionFactor(Config.Swerve.angleVelocityConversionFactor);

    angleMotorConfig.closedLoop.p(Config.Swerve.angleKP);
    angleMotorConfig.closedLoop.i(Config.Swerve.angleKI);
    angleMotorConfig.closedLoop.d(Config.Swerve.angleKD);
    angleMotorConfig.closedLoop.velocityFF(Config.Swerve.angleKFF);
    angleMotorConfig.closedLoop.positionWrappingMinInput(0);
    angleMotorConfig.closedLoop.positionWrappingMaxInput(2 * Math.PI);
    angleMotorConfig.closedLoop.positionWrappingEnabled(true);
    angleMotor.configure(angleMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    angleMotor.setCANTimeout(0);
  }

  private void configDriveMotor() {
    driveMotorConfig = new SparkMaxConfig();
    driveMotor.setCANTimeout(Config.CANTIMEOUT_MS);

    driveMotorConfig.smartCurrentLimit(Config.Swerve.driveContinuousCurrentLimit);
    driveMotorConfig.inverted(Config.Swerve.driveInvert);
    driveMotorConfig.idleMode(Config.Swerve.driveNeutralMode);
    driveMotorConfig.encoder.velocityConversionFactor(Config.Swerve.driveConversionVelocityFactor);
    driveMotorConfig.encoder.positionConversionFactor(Config.Swerve.driveConversionPositionFactor);
    driveMotorConfig.closedLoop.p(Config.Swerve.driveKP);
    driveMotorConfig.closedLoop.i(Config.Swerve.driveKI);
    driveMotorConfig.closedLoop.d(Config.Swerve.driveKD);
    driveMotorConfig.closedLoop.velocityFF(Config.Swerve.driveKFF);
    driveMotorConfig.closedLoop.positionWrappingMinInput(0);
    driveMotorConfig.closedLoop.positionWrappingMaxInput(2 * Math.PI);
    driveMotorConfig.closedLoop.positionWrappingEnabled(true);
    driveMotorConfig.voltageCompensation(Config.Swerve.voltageComp);

    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    driveEncoder.setPosition(0.0);

    driveMotor.setCANTimeout(0);
  }

 
  /**
   * Enable/disable voltage compensation on the drive motors. 
   * 
   * @param enable True to enable, false to disable.
   */
  public void setVoltageCompensation(boolean enable) {
    if (enable) {
      driveMotorConfig.voltageCompensation(Config.Swerve.voltageComp);
    } else {
      driveMotorConfig.disableVoltageCompensation();
    }

    driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }

  /*
   * Sets Speed
   * 
   * @param desiredState
   * @param isOpenLoop Op
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double speed = desiredState.speedMetersPerSecond * desiredState.angle.minus(getAngle()).getCos();

    if (isOpenLoop) {
      // original implementation
      // double percentOutput = desiredState.speedMetersPerSecond / Config.Swerve.maxSpeed;
      double percentOutput = speed / Config.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      errSpark("Drive set FF", 
        driveController.setReference(
          speed,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(speed)));
    }
  }

  /**
   * Sets Angle
   * 
   * @param desiredState 
   */
  private void setAngle(SwerveModuleState desiredState, boolean isDisableAntiJitter) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle;
    if (isDisableAntiJitter) {
      angle = desiredState.angle;

    // Anti jitter check
    } else if (Math.abs(desiredState.speedMetersPerSecond) <= (Config.Swerve.maxSpeed * 0.01)) {
      angle = lastAngle;

    } else {
      angle = desiredState.angle;
    }

    errSpark("Angle set reference", angleController.setReference(angle.getRadians(), SparkBase.ControlType.kPosition));
    lastAngle = angle;
  }

  /**
   * Returns Angle
   * 
   * @return Angle
   */
  private Rotation2d getAngle() {
    return (new Rotation2d(integratedAngleEncoder.getPosition()));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble()*360);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public double getSteeringVelocity() {
    return integratedAngleEncoder.getVelocity();  
  }

  public void setFeedforward(SimpleMotorFeedforward newFeedforward) {
    feedforward = newFeedforward;
  }

  public void periodic() {
    //update network tables
    currentSpeedEntry.accept(driveEncoder.getVelocity());
    currentAngleEntry.accept(getAngle().getRadians());
    
    canCoderAngleEntry.accept(getCanCoder().getDegrees());

    if (Config.swerveTuning) {  
      angleOffset = Rotation2d.fromDegrees(entryAngleOffset.get());
    }
  }

  public boolean isModuleSynced(){
    // Calculate the angle error between the NEO encoder and cancoder
    double angleError = getAngle().getDegrees() - (getCanCoder().getDegrees() - angleOffset.getDegrees());

    // Wrap the angle to (-180, 180], get the absolute value, then check if the error is less than the tolerance
    if (Math.abs(MathUtil.inputModulus(angleError, -180, 180)) < Config.Swerve.synchTolerance) {
      return true;
    }
    else{
      return false;
    }
  }

  public void stopMotors() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }
}
