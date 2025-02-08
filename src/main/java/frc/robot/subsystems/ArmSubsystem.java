package frc.robot.subsystems;

//import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ProfiledPIDFFController;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;

public class ArmSubsystem extends SubsystemBase {
  private static ArmSubsystem instance = null; // static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype
  private final SparkMax m_arm; // bottom SparkMax motor controller
  private SparkMaxConfig m_arm_config;

  // network table entry
  private final String m_tuningTable = "Arm/ArmTuning";
  private final String m_dataTable = "Arm/ArmData";

  // network table entries
  private DoubleEntry m_armPSubs;
  private DoubleEntry m_armISubs;
  private DoubleEntry m_armDSubs;
  private DoubleEntry m_armIzSubs;
  private DoubleEntry m_armFFSubs;
  private DoublePublisher m_armSetpointPub;
  private DoublePublisher m_armVelPub;
  private DoublePublisher m_armFFTestingVolts;
  private DoubleEntry m_armOffset;
  private DoublePublisher m_targetAngle;
  private DoublePublisher m_armPosPub;

  // for arm ff
  private DoubleEntry m_armMomentToVoltage;

  //spark absolute encoder
  private SparkAbsoluteEncoder m_absEncoder;  
  //embedded relative encoder
  private SparkClosedLoopController m_pidControllerArm;    

  private final TrapezoidProfile.Constraints m_constraints = 
    new TrapezoidProfile.Constraints(Config.ArmConfig.MAX_VEL, Config.ArmConfig.MAX_ACCEL);
  private final ProfiledPIDController m_ProfiledPIDController = 
    new ProfiledPIDController(1.6,0.002,40, m_constraints, 0.02);


  public static ArmSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
  }

  private ArmSubsystem() {
    m_arm = new SparkMax(Config.ArmConfig.ARM_SPARK_CAN_ID, motorType); // creates SparkMax motor controller
    m_arm_config = new SparkMaxConfig();

    m_arm.setCANTimeout(Config.CANTIMEOUT_MS);

    m_arm_config.smartCurrentLimit(Config.ArmConfig.CURRENT_LIMIT);
    m_arm_config.inverted(Config.ArmConfig.SET_INVERTED);
    m_arm_config.idleMode(IdleMode.kBrake);
    m_arm_config.voltageCompensation(6);
    m_arm_config.softLimit.forwardSoftLimit(Config.ArmConfig.arm_forward_limit);
    m_arm_config.softLimit.reverseSoftLimit(Config.ArmConfig.arm_reverse_limit);
    m_arm_config.softLimit.forwardSoftLimitEnabled(Config.ArmConfig.SOFT_LIMIT_ENABLE);
    m_arm_config.softLimit.reverseSoftLimitEnabled(Config.ArmConfig.SOFT_LIMIT_ENABLE);
    m_arm_config.signals.primaryEncoderPositionPeriodMs(20);
    m_arm_config.signals.primaryEncoderVelocityPeriodMs(20);

    m_absEncoder = m_arm.getAbsoluteEncoder();
    m_arm_config
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(Config.ArmConfig.INVERT_ENCODER)
          .positionConversionFactor(Config.ArmConfig.armPositionConversionFactor) // radians
          .velocityConversionFactor(Config.ArmConfig.armVelocityConversionFactor); // radians per second
          //.zeroOffset(Math.toRadians(Config.ArmConfig.armAbsEncoderOffset)); 

    m_arm_config
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, Config.ArmConfig.armPositionConversionFactor);


    NetworkTable ArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_armPSubs = ArmTuningTable.getDoubleTopic("P").getEntry(Config.ArmConfig.arm_kP);
    m_armISubs = ArmTuningTable.getDoubleTopic("I").getEntry(Config.ArmConfig.arm_kI);
    m_armDSubs = ArmTuningTable.getDoubleTopic("D").getEntry(Config.ArmConfig.arm_kD);
    m_armIzSubs = ArmTuningTable.getDoubleTopic("IZone").getEntry(Config.ArmConfig.arm_kIz);
    m_armFFSubs = ArmTuningTable.getDoubleTopic("FF").getEntry(Config.ArmConfig.arm_kFF);
    // m_topArmOffset =
    // topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    m_armMomentToVoltage = ArmTuningTable.getDoubleTopic("MomentToVoltage")
        .getEntry(Config.ArmConfig.MOMENT_TO_VOLTAGE);

    m_armFFSubs.setDefault(Config.ArmConfig.arm_kFF);
    m_armPSubs.setDefault(Config.ArmConfig.arm_kP);
    m_armISubs.setDefault(Config.ArmConfig.arm_kI);
    m_armDSubs.setDefault(Config.ArmConfig.arm_kD);
    m_armIzSubs.setDefault(Config.ArmConfig.arm_kIz);

    NetworkTable ArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

    m_armPosPub = ArmDataTable.getDoubleTopic("MeasuredAngleDeg").publish(PubSubOption.periodic(0.02));
    m_armVelPub = ArmDataTable.getDoubleTopic("MeasuredVelocity").publish(PubSubOption.periodic(0.02));
    m_armFFTestingVolts= ArmDataTable.getDoubleTopic("FFTestingVolts").publish(PubSubOption.periodic(0.02));
    m_targetAngle = ArmDataTable.getDoubleTopic("TargetAngleDeg").publish(PubSubOption.periodic(0.02));

    updatePID0Settings();
    updatePID1Settings();

    m_arm.configure(m_arm_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    
    //burnFlash();
    m_arm.setCANTimeout(0);

    ErrorTrackingSubsystem.getInstance().register(m_arm);
  }

  public void updatePID0Settings() {
    
    m_arm_config.closedLoop.velocityFF(m_armFFSubs.get(), ClosedLoopSlot.kSlot0);
    m_arm_config.closedLoop.p(m_armPSubs.get(), ClosedLoopSlot.kSlot0);
    m_arm_config.closedLoop.i(m_armPSubs.get(), ClosedLoopSlot.kSlot0);
    m_arm_config.closedLoop.d(m_armDSubs.get(), ClosedLoopSlot.kSlot0);
    m_arm_config.closedLoop.iZone(m_armIzSubs.get(), ClosedLoopSlot.kSlot0);
    m_arm_config.closedLoop.outputRange(Config.ArmConfig.min_output, Config.ArmConfig.max_output);

  }

  public void updatePID1Settings() { 
    m_arm_config.closedLoop.velocityFF(ArmConfig.arm_far_kFF, ClosedLoopSlot.kSlot1);
    m_arm_config.closedLoop.p(ArmConfig.arm_far_kP, ClosedLoopSlot.kSlot1);
    m_arm_config.closedLoop.i(ArmConfig.arm_far_kI, ClosedLoopSlot.kSlot1);
    m_arm_config.closedLoop.d(ArmConfig.arm_far_kD, ClosedLoopSlot.kSlot1);
    m_arm_config.closedLoop.iZone(ArmConfig.arm_far_iZone, ClosedLoopSlot.kSlot1);

  }

  @Override
  public void periodic() {
    m_armPosPub.accept(Math.toDegrees(getPosition()));
    m_armVelPub.accept(Math.toDegrees(m_absEncoder.getVelocity()));
  }

    // input angle_bottom in radians(
  public void setJointAngle(double angle) {
    double clampedAngle = MathUtil.clamp(angle, Math.toRadians(Config.ArmConfig.MIN_ARM_ANGLE_DEG),
        Math.toRadians(Config.ArmConfig.MAX_ARM_ANGLE_DEG));

    // pidSlot 1 is tuned well for setpoints between 25 deg and 45 deg
    double angleDeg = Math.toDegrees(angle);
    ClosedLoopSlot pidSlot = ClosedLoopSlot.kSlot0;
    if (angleDeg < 25) {
      pidSlot = ClosedLoopSlot.kSlot0;
    } else if (angleDeg >= 25 && angleDeg < 55) {
      pidSlot = ClosedLoopSlot.kSlot1;
    } else if (angleDeg >= 55) {
      pidSlot = ClosedLoopSlot.kSlot0;
    }

    m_ProfiledPIDController.calculate(getPosition(), clampedAngle);
    double targetPos = m_ProfiledPIDController.getSetpoint().position;

    //m_pidControllerArm.setReference((targetPos), ControlType.kPosition, 0, calculateFF(clampedAngle));
    m_pidControllerArm.setReference(targetPos + Math.toRadians(ArmConfig.shiftEncoderRange), ControlType.kPosition, pidSlot, 0);

     m_targetAngle.accept(Math.toDegrees(targetPos));
  }

  public void resetProfiledPIDController() {
     m_ProfiledPIDController.reset(getPosition(), m_absEncoder.getVelocity());
  }


  
    //return radius
    public double getPosition() {
      return m_absEncoder.getPosition() - Math.toRadians(ArmConfig.shiftEncoderRange);
    }
  
    public void stopMotors() {
      m_arm.stopMotor();
    }

    public void burnFlash() {
      try {
        Thread.sleep(200);
      } 
      catch (Exception e) {}

      m_arm.configure(m_arm_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      //errSpark("Arm burn flash", m_arm.burnFlash());
    }

    private double calculateFF(double encoder1Rad) {
      //double ArmMoment = Config.ArmConfig.ARM_FORCE * (Config.ArmConfig.LENGTH_ARM_TO_COG*Math.cos(encoder1Rad));
      //return (ArmMoment) * m_armMomentToVoltage.get();

      double toTunedConst = m_armMomentToVoltage.get();
      return toTunedConst*Math.cos(encoder1Rad);
    }

    public void isAtSetpoint() {
    }

    public void setArmIdleMode(IdleMode mode) {
  
      m_arm_config.idleMode(mode);

    }

    public void testFeedForward(double additionalVoltage) {
      double voltage = additionalVoltage + calculateFF(getPosition());
      m_pidControllerArm.setReference(voltage, ControlType.kVoltage);
      m_armFFTestingVolts.accept(voltage);
    }

}
