import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;

import frc.robot.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

   private static AlgaeManipulator instance = null;

  private final SparkMax m_algeaManipulator;
  private final SparkMaxConfig m_algeaManipulator_config;
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;
  private int algeaManipulatorCanID = 1;


    public static AlgaeManipulator getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.CoralIntakeSubsystem);
      instance = new AlgaeManipulator();
    }
    return instance;
  }

  public AlgaeManipulator() {
    m_algeaManipulator = new SparkMax(algeaManipulatorCanID, MotorType.kBrushless);
    m_algeaManipulator_config = new SparkMaxConfig(); 
    m_encoder = m_algeaManipulator.getEncoder();
    m_pidController = m_algeaManipulator.getClosedLoopController();

    // Configure the motor
    m_algeaManipulator.setCANTimeout(Config.CANTIMEOUT_MS);

    m_algeaManipulator_config
                        .inverted(Config.ElevatorConfig.SET_INVERTED)
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT)
                        .voltageCompensation(12);

    // Configure the PID controller
    m_algeaManipulator_config.closedLoop
    .velocityFF(0.003)
    .outputRange(-1,1)
    .maxMotion.maxVelocity(1000)
    .maxAcceleration(1000)
    .allowedClosedLoopError(0.25);
    
  }
  
  public void startIntakePercent(double percentOutput) {
    m_algeaManipulator.set(percentOutput);
  }

  public void moveToAngle(double targetAngleDegrees) {
    double targetPosition = targetAngleDegrees / 360.0; 
    m_pidController.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }
  public void stop() 
  {
    m_algeaManipulator.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}