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
import frc.robot.Config.ElevatorConfig;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem instance = null; // static object that contains all movement controls

    private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype
    private final SparkMax m_elevator; // bottom SparkMax motor controller
    private SparkMaxConfig m_elevator_config;

    private RelativeEncoder m_elevator_encoder;

    // network table entry
    private final String m_tuningTable = "Elevator/ElevatorTuning";
    private final String m_dataTable = "Elevator/ElevatorData";

    // network table entries
    private DoubleEntry m_elevatorPSubs;
    private DoubleEntry m_elevatorISubs;
    private DoubleEntry m_elevatorDSubs;
    private DoubleEntry m_elevatorIzSubs;
    private DoubleEntry m_elevatorFFSubs;
    private DoublePublisher m_targetPosition;
    private DoublePublisher m_currentPosition;

    public double elevatorTargetPos = 0;
    public boolean controlOverride = false;

    //embedded relative encoder
    private SparkClosedLoopController m_pidControllerElevator;


    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.ElevatorSubsystem);
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    private ElevatorSubsystem() {
        // Initialize elevator and other stuff
        m_elevator = new SparkMax(Config.ElevatorConfig.ELEVATOR_SPARK_CAN_ID, motorType); // creates SparkMax motor controller
        m_elevator_config = new SparkMaxConfig();

        m_elevator_encoder = m_elevator.getEncoder();
        m_pidControllerElevator = m_elevator.getClosedLoopController();


        // Config elevator
        m_elevator.setCANTimeout(Config.CANTIMEOUT_MS);

        m_elevator_config.inverted(Config.ElevatorConfig.SET_INVERTED)
                        .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT)
                                        .voltageCompensation(12);

        // Hard limit via limit switch
        m_elevator_config.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true);

        // Soft limit
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
        m_elevatorPSubs.setDefault(Config.ElevatorConfig.elevator_kPDefault);
        m_elevatorISubs.setDefault(Config.ElevatorConfig.elevator_kI);
        m_elevatorDSubs.setDefault(Config.ElevatorConfig.elevator_kD);
        m_elevatorIzSubs.setDefault(Config.ElevatorConfig.elevator_kIz);


        // Send telemetry thru networktables
        NetworkTable ElevatorDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

        m_targetPosition = ElevatorDataTable.getDoubleTopic("TargetPosition").publish(PubSubOption.periodic(0.02));
        m_currentPosition = ElevatorDataTable.getDoubleTopic("CurrentPosition").publish(PubSubOption.periodic(0.02));


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
        m_currentPosition.accept(m_elevator_encoder.getPosition());
        m_targetPosition.accept(elevatorTargetPos);

        if (!controlOverride) {
            // set elevator height
            setElevatorHeight(elevatorTargetPos);
        }
    }

    public void setElevatorHeight(double height) {
        ClosedLoopSlot pidSlot = ClosedLoopSlot.kSlot0;
        m_pidControllerElevator.setReference(height, ControlType.kPosition, pidSlot, 0);
    }



    //return positon
    public double getPosition() {
        return m_elevator.getEncoder().getPosition() - ElevatorConfig.shiftEncoderRange;
    }


    public double getVelocity() {
        return m_elevator.getEncoder().getVelocity();
    }


    public void raiseMotor() {
        m_elevator.set(1);
        controlOverride = true;
    }

    public void lowerMotor() {
        m_elevator.set(-1);
        controlOverride = true;
    }

    public void stopMotors() {
        m_elevator.stopMotor();
        controlOverride = true;
    }

}
