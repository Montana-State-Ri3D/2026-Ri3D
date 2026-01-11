package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private final GenericMotorSim motor =
      new GenericMotorSim(
          DCMotor.getNEO(1),
          IntakeConstants.FrontRollers.GEAR_RATIO,
          IntakeConstants.FrontRollers.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    motor.update(Constants.defaultPeriod);
    inputs.frontRollersVelocityRPM =
        motor.getVelocity().in(Units.RPM) * IntakeConstants.FrontRollers.GEAR_RATIO;
    inputs.frontRollersAppliedOutput = motor.getVoltage();
    inputs.frontRollersCurrentAmps = motor.getCurrent().in(Units.Amp);
    inputs.frontRollersTempCelsius = 0;
  }

  @Override
  public void setFrontVoltage(double volts) {
    motor.setVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setFrontVel(AngularVelocity vel) {
    motor.setVelocity(vel.div(IntakeConstants.FrontRollers.GEAR_RATIO));
  }

  @Override
  public void configFrontRollers(double kV, double kP, double maxAcceleration) {
    motor.setConfig(kP, 0, 0, kV, 0);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    return true;
  }
}
