package frc.lib.lib3512.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a SparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setSparkMaxBusUsage(
      SparkMax motor, Usage usage, boolean enableFollowing) {
    SparkMaxConfig motor_config = new SparkMaxConfig();

    if (enableFollowing) {
      motor_config.signals.primaryEncoderPositionPeriodMs(10);
    } else {
      motor_config.signals.primaryEncoderPositionPeriodMs(500);
    }

    if (usage == Usage.kAll) {
      motor_config.signals.primaryEncoderPositionPeriodMs(20);
      motor_config.signals.primaryEncoderVelocityPeriodMs(20);
      motor_config.signals.analogVoltagePeriodMs(50);
    } else if (usage == Usage.kPositionOnly) {
      motor_config.signals.primaryEncoderPositionPeriodMs(500);
      motor_config.signals.primaryEncoderVelocityPeriodMs(20);
      motor_config.signals.analogVoltagePeriodMs(500);
    } else if (usage == Usage.kVelocityOnly) {
      motor_config.signals.primaryEncoderPositionPeriodMs(20);
      motor_config.signals.primaryEncoderVelocityPeriodMs(500);
      motor_config.signals.analogVoltagePeriodMs(500);
    } else if (usage == Usage.kMinimal) {
      motor_config.signals.primaryEncoderPositionPeriodMs(500);
      motor_config.signals.primaryEncoderVelocityPeriodMs(500);
      motor_config.signals.analogVoltagePeriodMs(500);
    }

    motor.configure(motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a SparkMax is
   *     constructed.
   */
  public static void setSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setSparkMaxBusUsage(motor, usage, false);
  }
}
