package frc.robot.subsystems;

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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;

import frc.robot.Config;

public class AlgaeMainpulatorSubsystem extends SubsystemBase {

   private static AlgaeMainpulatorSubsystem instance = null;

  private final SparkMax m_AlgaeMainpulatorSubsystem;
  private final SparkMaxConfig m_AlgaeMainpulatorSubsystem_config;
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;
  private int AlgaeMainpulatorSubsystemCanID = 5;
    // network table entry
    private final String m_tuningTable = "Algae/AlgaeTuning";
    private final String m_dataTable = "Algae/AlgaeData";
    // network table entries
    private DoubleEntry m_AlgaePSubs;
    private DoubleEntry m_AlgaeISubs;
    private DoubleEntry m_AlgaeDSubs;
    private DoubleEntry m_AlgaeIzSubs;
    private DoubleEntry m_AlgaeFFSubs;
    private DoublePublisher m_targetPositionPub;
    private DoublePublisher m_currentPositionPub;
    private BooleanPublisher m_switchPressedPub;
    private BooleanPublisher m_servoBrakeOnPub;

    public static AlgaeMainpulatorSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.AlgaeSubsystem);
      instance = new AlgaeMainpulatorSubsystem();
    }
    return instance;
  }

  public AlgaeMainpulatorSubsystem() {
    m_AlgaeMainpulatorSubsystem = new SparkMax(AlgaeMainpulatorSubsystemCanID, MotorType.kBrushless);
    m_AlgaeMainpulatorSubsystem_config = new SparkMaxConfig();
    m_pidController = m_AlgaeMainpulatorSubsystem.getClosedLoopController();
    m_encoder = m_AlgaeMainpulatorSubsystem.getEncoder();

    m_AlgaeMainpulatorSubsystem.setCANTimeout(Config.CANTIMEOUT_MS);

    m_AlgaeMainpulatorSubsystem_config.softLimit.reverseSoftLimit(-160)
    .reverseSoftLimitEnabled(false)
    //forward limit is tbd
    .forwardSoftLimit(0)
    .forwardSoftLimitEnabled(false);

    // Get pid values from network tables
        NetworkTable AlgaeTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_AlgaePSubs = AlgaeTuningTable.getDoubleTopic("P").getEntry(Config.AlgaeConfig.algae_kP);
        m_AlgaeISubs = AlgaeTuningTable.getDoubleTopic("I").getEntry(Config.AlgaeConfig.algae_kI);
        m_AlgaeDSubs = AlgaeTuningTable.getDoubleTopic("D").getEntry(Config.AlgaeConfig.algae_kD);
        m_AlgaeIzSubs = AlgaeTuningTable.getDoubleTopic("IZone").getEntry(Config.AlgaeConfig.algae_kIz);
    //@todo: to be tuned
        m_AlgaePSubs.setDefault(Config.AlgaeConfig.algae_kP);
        m_AlgaeISubs.setDefault(Config.AlgaeConfig.algae_kI);
        m_AlgaeDSubs.setDefault(Config.AlgaeConfig.algae_kD);
        m_AlgaeIzSubs.setDefault(Config.AlgaeConfig.algae_kIz);
    // Send telemetry thru networktables
        NetworkTable AlgaeDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_targetPositionPub = AlgaeDataTable.getDoubleTopic("TargetPosition").publish(PubSubOption.periodic(0.02));
        m_currentPositionPub = AlgaeDataTable.getDoubleTopic("CurrentPosition").publish(PubSubOption.periodic(0.02));
        m_switchPressedPub =AlgaeDataTable.getBooleanTopic("IsSwitchPressed").publish(PubSubOption.periodic(0.02));
        m_servoBrakeOnPub = AlgaeDataTable.getBooleanTopic("IsBrakeOn").publish(PubSubOption.periodic(0.02));
    // Configure the motor controller
    m_AlgaeMainpulatorSubsystem_config
                        .inverted(Config.ElevatorConfig.SET_INVERTED)
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT)
                        .voltageCompensation(12);

    // Configure the PID controller
    m_AlgaeMainpulatorSubsystem_config.closedLoop
    .velocityFF(0.003)
    .outputRange(-1,1)
    .maxMotion.maxVelocity(1000)
    .maxAcceleration(1000)
    .allowedClosedLoopError(0.25);
    m_encoder.setPosition(0);
    
  }
  
  public void startIntakePercent(double percentOutput) {
    m_AlgaeMainpulatorSubsystem.set(percentOutput);
  }

  public void moveToAngle(double targetAngleDegrees) {
    double targetPosition = targetAngleDegrees / 360.0; 
    m_pidController.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }
  public void stop() 
  {
    
    m_AlgaeMainpulatorSubsystem.stopMotor();
  }
  @Override
  public void periodic() {
  
  }
}