package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class AlgaeManipulator extends SubsystemBase {

  private final CANSparkMax m_algeaManipulator;
  private final SparkMaxPIDController m_pidController;
  private final CANEncoder m_encoder;

  public AlgaeManipulator() {
    m_algeaManipulator = new CANSparkMax(Config.algeaManipulatorCanID, MotorType.kBrushless);
    m_pidController = m_algeaManipulator.getPIDController();
    m_encoder = m_algeaManipulator.getEncoder();

    // Configure the motor
    m_algeaManipulator.setCANTimeout(Config.CANTIMEOUT_MS);
    m_algeaManipulator.setInverted(Config.ALGAE_MANIPULATOR_INVERTED);
    m_algeaManipulator.setIdleMode(IdleMode.kBrake);
    m_algeaManipulator.setSmartCurrentLimit(Config.ALGAE_MANIPULATOR_CURRENT_LIMIT);
    m_algeaManipulator.enableVoltageCompensation(6);

    // Configure the PID controller
    m_pidController.setP(Config.algaeManipulator_kP);
    m_pidController.setI(Config.algaeManipulator_kI);
    m_pidController.setD(Config.algaeManipulator_kD);
    m_pidController.setFF(Config.algaeManipulator_kFF);
    m_pidController.setIZone(Config.algaeManipulator_kIz);
    m_pidController.setOutputRange(Config.algaeManipulator_min_output, Config.algaeManipulator_max_output);

    // Configure the motor with the PID controller settings
    m_algeaManipulator.burnFlash();
  }

  public static final int algeaManipulatorCanID = 5; // Example CAN ID, change as needed
  
  public void startIntakePercent(double percentOutput) {
    m_algeaManipulator.set(percentOutput);
  }

  public void moveToAngle(double targetAngleDegrees) {
    // Convert the target angle from degrees to encoder units (rotations)
    double targetPosition = targetAngleDegrees / 360.0; // Assuming 1 rotation = 360 degrees
    m_pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  public void startingPosition() {
    // Move the arm to the starting position (e.g., 0 degrees)
    moveToAngle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}