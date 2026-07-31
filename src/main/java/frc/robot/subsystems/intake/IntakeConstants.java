package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

final class IntakeConstants {
  static final int ROLLER_LEAD_ID = 21;
  static final int ROLLER_FOLLOWER_ID = 20;
  static final int ARM_MOTOR_ID = 10;
  static final int ARM_CANCODER_ID = 0;
  static final int ARM_FAST_SLOT = 0;
  static final int ARM_SLOW_SLOT = 1;
  static final double ARM_DEPLOYED_ROTATIONS = 0.0;
  static final double ARM_PRE_JUICE_ROTATIONS = 0.15;
  static final double ARM_SQUEEZE_ROTATIONS = 0.25;
  static final double ARM_STOWED_ROTATIONS = 0.37;
  static final double ARM_POSITION_TOLERANCE = 0.025;
  static final Current ROLLER_STATOR_LIMIT = Amps.of(80.0);
  static final Current ROLLER_SUPPLY_LIMIT = Amps.of(40.0);
  static final Current ROLLER_SUPPLY_LOWER_LIMIT = Amps.of(30.0);
  static final Time ROLLER_SUPPLY_LOWER_TIME = Seconds.of(0.2);
  static final Current ARM_STATOR_LIMIT = Amps.of(50.0);
  static final Current ARM_SUPPLY_LIMIT = Amps.of(30.0);
  static final Current INTAKE_ROLLER_CURRENT = Amps.of(80.0);
  static final Current OUTTAKE_ROLLER_CURRENT = Amps.of(-80.0);
  static final Current ARM_INTAKE_TENSION_CURRENT = Amps.of(-15.0);
  static final double ROLLER_STATE_MAX_DUTY = 0.80;
  static final double ROLLER_JUICER_MAX_DUTY = 0.50;
  static final Voltage ROLLER_MAX_VOLTAGE = Volts.of(12.0);
  static final Voltage ARM_MAX_VOLTAGE = Volts.of(10.0);
  static final Voltage AUTONOMOUS_DEPLOY_ROLLER_VOLTAGE = Volts.of(-6.0);
  static final double CONTROL_HZ = 100.0;
  static final double MECHANISM_HZ = 50.0;
  static final double SLOW_HZ = 4.0;

  static TalonFXConfiguration createRollerLeadConfig() {
    return createRollerConfig(InvertedValue.Clockwise_Positive);
  }

  static TalonFXConfiguration createRollerFollowerConfig() {
    return createRollerConfig(InvertedValue.Clockwise_Positive);
  }

  static TalonFXConfiguration createArmConfig() {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ARM_STATOR_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ARM_SUPPLY_LIMIT))
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(ARM_MAX_VOLTAGE)
                .withPeakReverseVoltage(ARM_MAX_VOLTAGE.unaryMinus()))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
        .withSlot0(
            new Slot0Configs()
                .withKP(14.0)
                .withKV(2.4)
                .withKG(0.5)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSlot1(
            new Slot1Configs()
                .withKP(8.0)
                .withKV(2.4)
                .withKG(0.5)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withFeedback(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ARM_CANCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1.0)
                .withRotorToSensorRatio(40.0));
  }

  static CANcoderConfiguration createArmCancoderConfig() {
    return new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(0.881064453125));
  }

  private static TalonFXConfiguration createRollerConfig(InvertedValue inversion) {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLER_SUPPLY_LIMIT)
                .withSupplyCurrentLowerLimit(ROLLER_SUPPLY_LOWER_LIMIT)
                .withSupplyCurrentLowerTime(ROLLER_SUPPLY_LOWER_TIME))
        .withVoltage(
            new VoltageConfigs()
                .withPeakForwardVoltage(ROLLER_MAX_VOLTAGE)
                .withPeakReverseVoltage(ROLLER_MAX_VOLTAGE.unaryMinus()))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(inversion));
  }

  private IntakeConstants() {}
}
