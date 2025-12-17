package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

  private final GenericMotorSim motor =
      new GenericMotorSim(DCMotor.getNEO(1), ElevatorConstants.GEAR_RATIO, ElevatorConstants.MOI);

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    motor.update(Constants.defaultPeriod);
    inputs.heightInches =
        motor.getPosition().in(Units.Rotation) / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.velocityInchesPerSecond =
        motor.getVelocity().in(Units.RotationsPerSecond) / ElevatorConstants.INCHES_TO_MOTOR_ROT;
    inputs.appliedOutput = motor.getVoltage();
    inputs.currentAmps = motor.getCurrent().in(Units.Amp);
    inputs.tempCelsius = 0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(Units.Volt.of(volts));
  }

  @Override
  public void setHeight(Distance height) {
    motor.setPosition(
        Units.Rotation.of(height.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT));
  }

  @Override
  public void setSensorPosition(Distance position) {
    motor.setState(
        Units.Rotations.of(position.in(Units.Inches) * ElevatorConstants.INCHES_TO_MOTOR_ROT)
            .in(Units.Radian),
        0);
  }

  @Override
  public void configMotor(
      double kP, double kD, double kG, double maxVelocity, double maxAcceleration) {
    motor.setConfig(kP, 0, kD, 0, kG);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    return true;
  }
}
