package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.ErrorCheck;
import frc.lib.lib2706.TunableNumber;
import frc.robot.Config;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;
    private CANSparkMax m_motor2;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private boolean closedLoopControl = false;
    private boolean stateFulControl = false;


    private TunableNumber kP = new TunableNumber("Shooter/PID0/kP", Config.ShooterConstants.kP);
    private TunableNumber kI = new TunableNumber("Shooter/PID0/kI", Config.ShooterConstants.kI);
    private TunableNumber kD = new TunableNumber("Shooter/PID0/kD", Config.ShooterConstants.kD);
    private TunableNumber kFF = new TunableNumber("Shooter/PID0/kFF", Config.ShooterConstants.kFF);

    private TunableNumber kP1 = new TunableNumber("Shooter/PID1/kP", Config.ShooterConstants.kP1);
    private TunableNumber kI1 = new TunableNumber("Shooter/PID1/kI", Config.ShooterConstants.kI1);
    private TunableNumber kD1 = new TunableNumber("Shooter/PID1/kD", Config.ShooterConstants.kD1);
    private TunableNumber kFF1 = new TunableNumber("Shooter/PID1/kFF", Config.ShooterConstants.kFF1);
    private TunableNumber shooterTreshHold = new TunableNumber("Shooter/tresh hold", 100);
    
    private DoublePublisher velocityPub;
    private StringPublisher statePub;
    private BooleanPublisher shooterReadyPub;

    private GenericEntry pubMotorTemp;
    private GenericEntry pubMotorTemp2;

    private static ShooterSubsystem shooter;
    public static ShooterSubsystem getInstance() {
        if (shooter == null)
            shooter = new ShooterSubsystem();
        return shooter;
    }

    public ShooterSubsystem() {
        System.out.println("[Init] Creating Shooter");
        m_motor = new CANSparkMax(Config.ShooterConstants.MOTOR_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();

        m_motor.setCANTimeout(500);//Units in miliseconds
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);

        m_motor2 = new CANSparkMax(Config.ShooterConstants.MOTOR_ID2, MotorType.kBrushless);
        m_motor2.restoreFactoryDefaults();

        m_motor2.setCANTimeout(500);//Units in miliseconds
        m_motor2.setIdleMode(IdleMode.kBrake);
        m_motor2.setInverted(false);

        //set the second motor as a follower of the first motor
        m_motor2.follow(m_motor);

        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();

        //Voltage compensation
        // m_motor.enableVoltageCompensation(10); //adjust on final robot
        m_motor.setSmartCurrentLimit(60);//Change this back to 70
        setBrake(true);

        m_pidController.setOutputRange(Config.ShooterConstants.kMinOutput, Config.ShooterConstants.kMaxOutput);
        setPIDGains(kP.get(), kI.get(), kD.get(), 0);
        setFFGains(kFF.get(), 0);

        setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1);
        setFFGains(kFF1.get(), 1);

        ErrorCheck.sparkBurnFlash("Shooter", m_motor);

        NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        velocityPub = shooterTable.getDoubleTopic("Shooter Velocity RPM").publish(PubSubOption.periodic(0.02));
        shooterReadyPub = shooterTable.getBooleanTopic("Shooter is Ready to shoot").publish(PubSubOption.periodic(0.02));
        statePub = shooterTable.getStringTopic("Shooter state").publish(PubSubOption.periodic(0.02));

        // Publish motor temperature on Shuffleboard alongside sparkmax logging for hardware to access everything they want in once place.
        pubMotorTemp = ErrorTrackingSubsystem.getInstance().getStatusTab().add("ShooterMotorTemp", -99)
            .withPosition(0, 2).withSize(2, 1).getEntry();

        // Publish the second motor temperature on Shufferboard
        pubMotorTemp2 = ErrorTrackingSubsystem.getInstance().getStatusTab().add("ShooterMotorTemp2", -99)
            .withPosition(0, 3).withSize(2, 1).getEntry();

        ErrorTrackingSubsystem.getInstance().register(m_motor);
        ErrorTrackingSubsystem.getInstance().register(m_motor2);

    }

    public double getVelocityRPM() {
        return m_encoder.getVelocity();
    }

    /**
     * Get the temperature of the motor in Celsius as reported by the sparkmax.
     * 
     * @return Celsius
     */
    public double getMotorTemperature() {
        return m_motor.getMotorTemperature();
    }

    public double getMotorTemperature2() {
        return m_motor2.getMotorTemperature();
    }

    public void setRPM(double setPoint) {
        int slotID = 1;
        m_pidController.setReference(setPoint, ControlType.kVelocity, slotID);
    }

    public void setVoltage(double setVolt) {
        m_motor.setVoltage(setVolt);
    }

    public void stop(){
        m_motor.stopMotor();
    }

    public void setBrake(boolean enableBreak){
        m_motor.setIdleMode(enableBreak ? IdleMode.kBrake: IdleMode.kCoast);
    }

    private void setPIDGains(double kP, double kI, double kD, int slotID){
        m_pidController.setP(kP, slotID);
        m_pidController.setI(kI, slotID);
        m_pidController.setD(kD, slotID);
    }   

    private void setFFGains(double kFF, int slotID){
        m_pidController.setFF(kFF, slotID);
    }   

    /*---------------------------Commands---------------------------*/

    
    @Override
    public void periodic() {
        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP.get(), kI.get(), kD.get(), 0), kP, kI, kD);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF.get(), 0), kFF);

        TunableNumber.ifChanged(hashCode(), ()->setPIDGains(kP1.get(), kI1.get(), kD1.get(), 1), kP1, kI1, kD1);
        TunableNumber.ifChanged(hashCode(), ()->setFFGains(kFF1.get(), 1), kFF1);

        velocityPub.accept(getVelocityRPM());
        pubMotorTemp.setDouble(getMotorTemperature());
        pubMotorTemp2.setDouble(getMotorTemperature2());

    }
}
