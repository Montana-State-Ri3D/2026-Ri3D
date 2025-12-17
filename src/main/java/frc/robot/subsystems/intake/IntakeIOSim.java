package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private final GenericMotorSim motor =
      new GenericMotorSim(DCMotor.getNEO(1), IntakeConstants.GEAR_RATIO, IntakeConstants.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    motor.update(Constants.defaultPeriod);
    inputs.velocityRPM = motor.getVelocity().in(Units.RPM) * ArmConstants.GEAR_RATIO;
    inputs.appliedOutput = motor.getVoltage();
    inputs.currentAmps = motor.getCurrent().in(Units.Amp);
    inputs.tempCelsius = 0;
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setVel(AngularVelocity vel) {
    motor.setVelocity(vel.div(ArmConstants.GEAR_RATIO));
  }

  @Override
  public void configMotor(double kV, double kP, double maxAcceleration) {
    motor.setConfig(kP, 0, 0, kV, 0);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    return true;
  }
}
