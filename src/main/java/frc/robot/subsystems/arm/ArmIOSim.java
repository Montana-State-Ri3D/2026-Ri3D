package frc.robot.subsystems.arm;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmIOSim implements ArmIO {

  private final GenericMotorSim motor =
      new GenericMotorSim(DCMotor.getNEO(1), ArmConstants.GEAR_RATIO, ArmConstants.MOI);

  @Override
  public void updateInputs(ArmInputs inputs) {
    motor.update(Constants.defaultPeriod);
    inputs.angleDegrees = motor.getPosition().in(Units.Degree) * ArmConstants.GEAR_RATIO;
    inputs.velocityDegreesPerSecond =
        motor.getVelocity().in(Units.DegreesPerSecond) * ArmConstants.GEAR_RATIO;
    inputs.appliedOutput = motor.getVoltage();
    inputs.currentAmps = motor.getCurrent().in(Units.Amp);
    inputs.tempCelsius = 0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setAngle(Angle angle) {
    motor.setPosition(angle.div(ArmConstants.GEAR_RATIO));
  }

  @Override
  public void setSensorPosition(Angle angle) {
    motor.setState(angle.div(ArmConstants.GEAR_RATIO).in(Units.Radian), 0);
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
