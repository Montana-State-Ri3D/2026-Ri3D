package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DriveModuleIOSim implements DriveModuleIO {

  private final DriveModuleSparkInputsAutoLogged inputs = new DriveModuleSparkInputsAutoLogged();

  private final GenericMotorSim motor =
      new GenericMotorSim(DCMotor.getNEO(1), DriveConstants.GEAR_RATIO, DriveConstants.MOI);
  private final int id;

  private LinearVelocity desiredVel = MetersPerSecond.of(0);

  public DriveModuleIOSim(int id) {
    this.id = id;
  }

  @Override
  public void updateInputs() {
    motor.update(Constants.defaultPeriod);
    inputs.voltage = motor.getVoltage();
    inputs.relativePosMeters = getRelativePosition().in(Units.Meter);
    inputs.relativePosRotations =
        motor.getPosition().in(Units.Rotation) * DriveConstants.GEAR_RATIO;
    inputs.velMetersPerSecond = getVelocity().in(Units.MetersPerSecond);

    inputs.desiredVelMetersPerSecond = desiredVel.in(Units.MetersPerSecond);
    inputs.current = motor.getCurrent().in(Units.Amps);

    Logger.processInputs(DriveConstants.ROOT_TABLE + "/Modules/Module" + id, inputs);
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setVelocity(LinearVelocity vel) {
    desiredVel = vel;
    motor.setVelocity(
        Units.RadiansPerSecond.of(vel.div(DriveConstants.WHEEL_RAD).magnitude())
            .div(DriveConstants.GEAR_RATIO));
  }

  @Override
  public Distance getRelativePosition() {
    return Units.Meter.of(
        motor.getPosition().in(Units.Radians)
            * DriveConstants.WHEEL_RAD.in(Units.Meter)
            * DriveConstants.GEAR_RATIO);
  }

  @Override
  public LinearVelocity getVelocity() {
    return Units.MetersPerSecond.of(
        motor.getVelocity().in(Units.RadiansPerSecond)
            * DriveConstants.WHEEL_RAD.in(Units.Meter)
            * DriveConstants.GEAR_RATIO);
  }

  @Override
  public void updateMotorConfig(double kV, double kP) {
    motor.setConfig(kP, 0, 0, kV, 0);
  }
}
