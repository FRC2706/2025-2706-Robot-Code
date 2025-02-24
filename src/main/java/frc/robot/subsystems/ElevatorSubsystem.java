package frc.robot.subsystems;

//import static frc.lib.lib2706.ErrorCheck.configureSpark;
import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.lib2706.ProfiledPIDFFController;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.ElevatorConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance = null; // static object that contains all movement controls

    private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype
    private final SparkMax m_elevator; // bottom SparkMax motor controller
    private SparkMaxConfig m_elevator_config;

    //embedded relative encoder
    private RelativeEncoder m_elevator_encoder; 
    private SparkClosedLoopController m_pidControllerElevator;

    //limit switch
    private final SparkLimitSwitch m_elevatorSwitch;
    private Boolean bWasResetbyLimit;

    //Servo for brake
    Servo servoBrake;

    // network table entry
    private final String m_tuningTable = "Elevator/ElevatorTuning";
    private final String m_dataTable = "Elevator/ElevatorData";

    // network table entries
    private DoubleEntry m_elevatorPSubs;
    private DoubleEntry m_elevatorISubs;
    private DoubleEntry m_elevatorDSubs;
    private DoubleEntry m_elevatorIzSubs;
    private DoubleEntry m_elevatorFFSubs;
    private DoublePublisher m_targetPositionPub;
    private DoublePublisher m_currentPositionPub;
    private BooleanPublisher m_switchPressedPub;

    //elevator target position
    public double elevatorTargetPos = 0;
 
    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.ElevatorSubsystem);
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    private ElevatorSubsystem() {

        servoBrake = new Servo(0);

        // Initialize elevator and other stuff
        m_elevator = new SparkMax(Config.ElevatorConfig.ELEVATOR_SPARK_CAN_ID, motorType); // creates SparkMax motor controller
        m_elevator_config = new SparkMaxConfig();

        m_elevator_encoder = m_elevator.getEncoder();
        m_pidControllerElevator = m_elevator.getClosedLoopController();

        //@todo: forware or reverse?
        m_elevatorSwitch = m_elevator.getForwardLimitSwitch();
        //m_elevatorSwitch = m_elevator.getReverseLimitSwitch();
        bWasResetbyLimit = false;

        // Config elevator
        m_elevator.setCANTimeout(Config.CANTIMEOUT_MS);

        m_elevator_config.inverted(Config.ElevatorConfig.SET_INVERTED)
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT)
                        .voltageCompensation(12);

        // Hard limit via limit switch
        m_elevator_config.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true); 
        // m_elevator_config.limitSwitch.reverseLimitSwitchEnabled(true)
        //         .reverseLimitSwitchType(Type.kNormallyOpen);

        // Soft limit of position
        //@todo: to determine the value and reverse or forward limit
        m_elevator_config.softLimit.reverseSoftLimit(500)
                .reverseSoftLimitEnabled(true);

        // Get pid values from network tables
        NetworkTable ElevatorTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_elevatorPSubs = ElevatorTuningTable.getDoubleTopic("P").getEntry(Config.ElevatorConfig.elevator_kP);
        m_elevatorISubs = ElevatorTuningTable.getDoubleTopic("I").getEntry(Config.ElevatorConfig.elevator_kI);
        m_elevatorDSubs = ElevatorTuningTable.getDoubleTopic("D").getEntry(Config.ElevatorConfig.elevator_kD);
        m_elevatorIzSubs = ElevatorTuningTable.getDoubleTopic("IZone").getEntry(Config.ElevatorConfig.elevator_kIz);
        m_elevatorFFSubs = ElevatorTuningTable.getDoubleTopic("FF").getEntry(Config.ElevatorConfig.elevator_kFF);

        m_elevatorFFSubs.setDefault(Config.ElevatorConfig.elevator_kFF);
        m_elevatorPSubs.setDefault(Config.ElevatorConfig.elevator_kP);
        m_elevatorISubs.setDefault(Config.ElevatorConfig.elevator_kI);
        m_elevatorDSubs.setDefault(Config.ElevatorConfig.elevator_kD);
        m_elevatorIzSubs.setDefault(Config.ElevatorConfig.elevator_kIz);

        // Send telemetry thru networktables
        NetworkTable ElevatorDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_targetPositionPub = ElevatorDataTable.getDoubleTopic("TargetPosition").publish(PubSubOption.periodic(0.02));
        m_currentPositionPub = ElevatorDataTable.getDoubleTopic("CurrentPosition").publish(PubSubOption.periodic(0.02));
        m_switchPressedPub = ElevatorDataTable.getBooleanTopic("IsSwitchPressed").publish(PubSubOption.periodic(0.02));

        // PID config
        m_elevator_config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pid(m_elevatorPSubs.get(), m_elevatorISubs.get(), m_elevatorDSubs.get())
                .velocityFF(0.003)
                .outputRange(-1,1)
                .maxMotion.maxVelocity(1000)
                .maxAcceleration(1000)
                .allowedClosedLoopError(0.25);

        // configure elevator motor
        m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        m_elevator.setCANTimeout(0);

        // reset encoder
        m_elevator_encoder.setPosition(0);

        ErrorTrackingSubsystem.getInstance().register(m_elevator);
    }

    @Override
    public void periodic() {
        // update networktables
        m_currentPositionPub.accept(m_elevator_encoder.getPosition());
        m_targetPositionPub.accept(elevatorTargetPos);
        m_switchPressedPub.accept(m_elevatorSwitch.isPressed());

        resetEncoderbyLimit();

    }

    public void resetEncoderbyLimit()
    {
      //to reset position when the switch is hit
      if(bWasResetbyLimit == false && m_elevatorSwitch.isPressed() == true)
      {
        resetEncoderPosition();
        bWasResetbyLimit = true;
      }
      else if (m_elevatorSwitch.isPressed() == false)
      {
        bWasResetbyLimit = false;
      }

    }
    public void setElevatorHeight(double height) {
        ClosedLoopSlot pidSlot = ClosedLoopSlot.kSlot0;
        m_pidControllerElevator.setReference(height, ControlType.kPosition, pidSlot, 0);

    }

    //return positon
    public double getCurrPosition() {
        return m_elevator_encoder.getPosition();
    }

    public double getCurrVelocity() {
        return m_elevator_encoder.getVelocity();
    }

    public void stopMotor() {
      m_elevator.stopMotor();
    }

    public void setTargetPos( double targetPos)
    {
      elevatorTargetPos = targetPos;
    }

    public void resetEncoderPosition()
    {
      m_elevator_encoder.setPosition(0.0);
    }

    public void updateEncoderPosition(double newPosition)
    {
      m_elevator_encoder.setPosition(newPosition);
    }

    public boolean isAtTargetPos() {
      if( Math.abs(elevatorTargetPos - m_elevator_encoder.getPosition() ) < Config.ElevatorConfig.ELEVATOR_POS_TH )
        return true;
      else
        return false;
    }

    public void setServoBrake(boolean bBrakeOn)
    {
      //@todo: the range of servo?
      if (bBrakeOn == true)
      {
        //servoBrake.setAngle(90);
        servoBrake.set(-0.5);
      }
      else
      {
        //servoBrake.setAngle(0);
        servoBrake.set(+0.5);
      }
    }

}
