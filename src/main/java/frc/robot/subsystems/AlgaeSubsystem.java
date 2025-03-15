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

public class AlgaeSubsystem extends SubsystemBase {

   private static AlgaeSubsystem instance = null;

  private final SparkMax m_algaeSubsystem;
  private final SparkMaxConfig m_algaeSubsystem_config;
  private final SparkClosedLoopController m_pidController;
  private final RelativeEncoder m_encoder;
  private int algaeSubsystemCanID = 5;
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

    public static AlgaeSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.CoralIntakeSubsystem);
      instance = new AlgaeSubsystem();
    }
    return instance;
  }

  public AlgaeSubsystem() {
    m_algaeSubsystem = new SparkMax(algaeSubsystemCanID, MotorType.kBrushless);
    m_algaeSubsystem_config = new SparkMaxConfig();
    m_pidController = m_algaeSubsystem.getClosedLoopController();
    m_encoder = m_algaeSubsystem.getEncoder();

    m_algaeSubsystem.setCANTimeout(Config.CANTIMEOUT_MS);

    m_algaeSubsystem_config.softLimit.reverseSoftLimit(0)
    .reverseSoftLimitEnabled(false)
    //forward limit is tbd
    .forwardSoftLimit(500)
    .forwardSoftLimitEnabled(false);

            // Get pid values from network tables
        NetworkTable AlgaeTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_AlgaePSubs = AlgaeTuningTable.getDoubleTopic("P").getEntry(Config.ElevatorConfig.elevator_kP);
        m_AlgaeISubs = AlgaeTuningTable.getDoubleTopic("I").getEntry(Config.ElevatorConfig.elevator_kI);
        m_AlgaeDSubs = AlgaeTuningTable.getDoubleTopic("D").getEntry(Config.ElevatorConfig.elevator_kD);
        m_AlgaeIzSubs = AlgaeTuningTable.getDoubleTopic("IZone").getEntry(Config.ElevatorConfig.elevator_kIz);
        m_AlgaeFFSubs = AlgaeTuningTable.getDoubleTopic("FF").getEntry(Config.ElevatorConfig.elevator_kFF);

        //@todo: to be tuned
        m_AlgaeFFSubs.setDefault(Config.ElevatorConfig.elevator_kFF);
        m_AlgaePSubs.setDefault(2.5);//Config.AlgaeConfig.elevator_kP
        m_AlgaeISubs.setDefault(Config.ElevatorConfig.elevator_kI);
        m_AlgaeDSubs.setDefault(Config.ElevatorConfig.elevator_kD);
        m_AlgaeIzSubs.setDefault(Config.ElevatorConfig.elevator_kIz);

        // Send telemetry thru networktables
        NetworkTable AlgaeDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_targetPositionPub = AlgaeDataTable.getDoubleTopic("TargetPosition").publish(PubSubOption.periodic(0.02));
        m_currentPositionPub = AlgaeDataTable.getDoubleTopic("CurrentPosition").publish(PubSubOption.periodic(0.02));
        m_switchPressedPub =AlgaeDataTable.getBooleanTopic("IsSwitchPressed").publish(PubSubOption.periodic(0.02));
        m_servoBrakeOnPub = AlgaeDataTable.getBooleanTopic("IsBrakeOn").publish(PubSubOption.periodic(0.02));
        
    m_algaeSubsystem_config
                        .inverted(Config.ElevatorConfig.SET_INVERTED)
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT)
                        .voltageCompensation(12);

    

    // Configure the PID controller
    m_algaeSubsystem_config.closedLoop
    .velocityFF(0.003)
    .outputRange(-1,1)
    .maxMotion.maxVelocity(1000)
    .maxAcceleration(1000)
    .allowedClosedLoopError(0.25);

    m_encoder.setPosition(0);
    
  }
  
  public void startIntakePercent(double percentOutput) {
    m_algaeSubsystem.set(percentOutput);
  }

  public void moveToAngle(double targetAngleDegrees) {
    double targetPosition = targetAngleDegrees / 360.0; 
    m_pidController.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }
  public void stop() 
  {
    
    m_algaeSubsystem.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}