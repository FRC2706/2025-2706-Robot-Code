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
    private DoublePublisher m_elevatorSetpointPub;
    private DoublePublisher m_elevatorVelPub;
    private DoublePublisher m_elevatorFFTestingVolts;
    private DoubleEntry m_elevatorOffset;
    private DoublePublisher m_targetAngle;
    private DoublePublisher m_elevatorPosPub;

    // for elevator ff
    private DoubleEntry m_elevatorMomentToVoltage;

    //embedded relative encoder
    private SparkClosedLoopController m_pidControllerElevator;

    private final TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(Config.ElevatorConfig.MAX_VEL, Config.ElevatorConfig.MAX_ACCEL);
    private final ProfiledPIDController m_ProfiledPIDController =
            new ProfiledPIDController(1.6,0.002,40, m_constraints, 0.02);


    public static ElevatorSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.ElevatorSubsystem);
            instance = new ElevatorSubsystem();
        }
        return instance;
    }

    private ElevatorSubsystem() {
        m_elevator = new SparkMax(Config.ElevatorConfig.ELEVATOR_SPARK_CAN_ID, motorType); // creates SparkMax motor controller
        m_elevator_config = new SparkMaxConfig();
        m_elevator_encoder = m_elevator.getEncoder();

        m_pidControllerElevator = m_elevator.getClosedLoopController();

        m_elevator.setCANTimeout(Config.CANTIMEOUT_MS);

        m_elevator_config.smartCurrentLimit(Config.ElevatorConfig.CURRENT_LIMIT);
        m_elevator_config.inverted(Config.ElevatorConfig.SET_INVERTED);
        m_elevator_config.idleMode(IdleMode.kBrake);
        m_elevator_config.voltageCompensation(6);
        m_elevator_config.softLimit.forwardSoftLimit(Config.ElevatorConfig.elevator_up_limit);
        m_elevator_config.softLimit.reverseSoftLimit(Config.ElevatorConfig.elevator_down_limit);
        //m_elevator_config.softLimit.forwardSoftLimitEnabled(Config.ElevatorConfig.SOFT_LIMIT_ENABLE);
        //m_elevator_config.softLimit.reverseSoftLimitEnabled(Config.ElevatorConfig.SOFT_LIMIT_ENABLE);
        m_elevator_config.softLimit.forwardSoftLimitEnabled(false);
        m_elevator_config.softLimit.reverseSoftLimitEnabled(false);
        m_elevator_config.signals.primaryEncoderPositionPeriodMs(20);
        m_elevator_config.signals.primaryEncoderVelocityPeriodMs(20);
        //m_elevator_config.encoder.inverted(Config.ElevatorConfig.INVERT_ENCODER);
        m_elevator_config.encoder.positionConversionFactor(Config.ElevatorConfig.elevatorPositionConversionFactor);
        m_elevator_config.encoder.velocityConversionFactor(Config.ElevatorConfig.elevatorVelocityConversionFactor);
        m_elevator_config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        //m_elevator_config.limitSwitch.reverseLimitSwitchEnabled(true);
        //m_elevator_config.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        NetworkTable ElevatorTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
        m_elevatorPSubs = ElevatorTuningTable.getDoubleTopic("P").getEntry(Config.ElevatorConfig.elevator_kP);
        m_elevatorISubs = ElevatorTuningTable.getDoubleTopic("I").getEntry(Config.ElevatorConfig.elevator_kI);
        m_elevatorDSubs = ElevatorTuningTable.getDoubleTopic("D").getEntry(Config.ElevatorConfig.elevator_kD);
        m_elevatorIzSubs = ElevatorTuningTable.getDoubleTopic("IZone").getEntry(Config.ElevatorConfig.elevator_kIz);
        m_elevatorFFSubs = ElevatorTuningTable.getDoubleTopic("FF").getEntry(Config.ElevatorConfig.elevator_kFF);
        // m_topElevatorOffset =
        // topElevatorTuningTable.getDoubleTopic("Offset").getEntry(ElevatorConfig.top_elevator_offset);
        m_elevatorMomentToVoltage = ElevatorTuningTable.getDoubleTopic("MomentToVoltage")
                .getEntry(Config.ElevatorConfig.MOMENT_TO_VOLTAGE);

        m_elevatorFFSubs.setDefault(Config.ElevatorConfig.elevator_kFF);
        m_elevatorPSubs.setDefault(Config.ElevatorConfig.elevator_kP);
        m_elevatorISubs.setDefault(Config.ElevatorConfig.elevator_kI);
        m_elevatorDSubs.setDefault(Config.ElevatorConfig.elevator_kD);
        m_elevatorIzSubs.setDefault(Config.ElevatorConfig.elevator_kIz);

        NetworkTable ElevatorDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);

        m_elevatorPosPub = ElevatorDataTable.getDoubleTopic("MeasuredAngleDeg").publish(PubSubOption.periodic(0.02));
        m_elevatorVelPub = ElevatorDataTable.getDoubleTopic("MeasuredVelocity").publish(PubSubOption.periodic(0.02));
        m_elevatorFFTestingVolts= ElevatorDataTable.getDoubleTopic("FFTestingVolts").publish(PubSubOption.periodic(0.02));
        m_targetAngle = ElevatorDataTable.getDoubleTopic("TargetAngleDeg").publish(PubSubOption.periodic(0.02));


        updatePID0Settings();
        updatePID1Settings();

        // configure elevator motor
        m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        burnFlash();
        m_elevator.setCANTimeout(0);
        m_elevator_encoder.setPosition(0);

        ErrorTrackingSubsystem.getInstance().register(m_elevator);
    }

    public void updatePID0Settings() {
        m_elevator_config.closedLoop.velocityFF(m_elevatorFFSubs.get(), ClosedLoopSlot.kSlot0);
        m_elevator_config.closedLoop.p(m_elevatorPSubs.get(), ClosedLoopSlot.kSlot0);
        m_elevator_config.closedLoop.i(m_elevatorPSubs.get(), ClosedLoopSlot.kSlot0);
        m_elevator_config.closedLoop.d(m_elevatorDSubs.get(), ClosedLoopSlot.kSlot0);
        m_elevator_config.closedLoop.iZone(m_elevatorIzSubs.get(), ClosedLoopSlot.kSlot0);
        m_elevator_config.closedLoop.outputRange(Config.ElevatorConfig.min_output, Config.ElevatorConfig.max_output);
    }

    public void updatePID1Settings() {
        m_elevator_config.closedLoop.velocityFF(ElevatorConfig.elevator_far_kFF, ClosedLoopSlot.kSlot1);
        m_elevator_config.closedLoop.p(ElevatorConfig.elevator_far_kP, ClosedLoopSlot.kSlot1);
        m_elevator_config.closedLoop.i(ElevatorConfig.elevator_far_kI, ClosedLoopSlot.kSlot1);
        m_elevator_config.closedLoop.d(ElevatorConfig.elevator_far_kD, ClosedLoopSlot.kSlot1);
        m_elevator_config.closedLoop.iZone(ElevatorConfig.elevator_far_iZone, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void periodic() {
        m_elevatorPosPub.accept(Math.toDegrees(getPosition()));
        m_elevatorVelPub.accept(Math.toDegrees(getVelocity()));
    }

    public void setElevatorHeight(double height) {

        // pidSlot 1 is tuned well for setpoints between 25 deg and 45 deg
        //double angleDeg = Math.toDegrees(angle);
        ClosedLoopSlot pidSlot = ClosedLoopSlot.kSlot0;
        /*
        if (angleDeg < 25) {
            pidSlot = ClosedLoopSlot.kSlot0;
        } else if (angleDeg >= 25 && angleDeg < 55) {
            pidSlot = ClosedLoopSlot.kSlot1;
        } else if (angleDeg >= 55) {
            pidSlot = ClosedLoopSlot.kSlot0;
        }*/

        //m_pidControllerElevator.setReference((targetPos), ControlType.kPosition, 0, calculateFF(clampedAngle));
        m_pidControllerElevator.setReference(height, ControlType.kPosition, pidSlot, 0);
    }

    public void resetProfiledPIDController() {
        m_ProfiledPIDController.reset(getPosition(), getVelocity());
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
    }

    public void lowerMotor() {
        m_elevator.set(-1);
    }

    public void stopMotors() {
        m_elevator.stopMotor();
    }

    public void burnFlash() {
        try {
            Thread.sleep(200);
        }
        catch (Exception e) {}

        //errSpark("Elevator burn flash", m_elevator.burnFlash());
    }

    private double calculateFF(double encoder1Rad) {
        //double ElevatorMoment = Config.ElevatorConfig.Elevator_FORCE * (Config.ElevatorConfig.LENGTH_Elevator_TO_COG*Math.cos(encoder1Rad));
        //return (ElevatorMoment) * m_elevatorMomentToVoltage.get();

        double toTunedConst = m_elevatorMomentToVoltage.get();
        return toTunedConst*Math.cos(encoder1Rad);
    }

    public void isAtSetpoint() {
    }

    public void setElevatorIdleMode(IdleMode mode) {
        SparkMaxConfig m_elevator_config = new SparkMaxConfig();

        m_elevator_config.idleMode(mode);

        m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void testFeedForward(double additionalVoltage) {
        double voltage = additionalVoltage + calculateFF(getPosition());
        m_pidControllerElevator.setReference(voltage, ControlType.kVoltage);
        m_elevatorFFTestingVolts.accept(voltage);
    }

}
