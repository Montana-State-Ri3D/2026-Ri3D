package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.teamBSR.GenericMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private final GenericMotorSim frontRoller =
      new GenericMotorSim(
          DCMotor.getNEO(1),
          IntakeConstants.FrontRollers.GEAR_RATIO,
          IntakeConstants.FrontRollers.MOI);
  private final GenericMotorSim backRoller =
      new GenericMotorSim(
          DCMotor.getNEO(1),
          IntakeConstants.FrontRollers.GEAR_RATIO,
          IntakeConstants.FrontRollers.MOI);
  private final GenericMotorSim extender =
      new GenericMotorSim(
          DCMotor.getNEO(1),
          IntakeConstants.FrontRollers.GEAR_RATIO,
          IntakeConstants.FrontRollers.MOI);

  @Override
  public void updateInputs(IntakeInputs inputs) {
    frontRoller.update(Constants.defaultPeriod);
    backRoller.update(Constants.defaultPeriod);
    extender.update(Constants.defaultPeriod);
    inputs.frontRollersVelocityRPM =
        frontRoller.getVelocity().in(Units.RPM) * IntakeConstants.FrontRollers.GEAR_RATIO;
    inputs.frontRollersAppliedOutput = frontRoller.getVoltage();
    inputs.frontRollersCurrentAmps = frontRoller.getCurrent().in(Units.Amp);
    inputs.backRollersVelocityRPM =
        backRoller.getVelocity().in(Units.RPM) * IntakeConstants.FrontRollers.GEAR_RATIO;
    inputs.backRollersAppliedOutput = backRoller.getVoltage();
    inputs.backRollersCurrentAmps = backRoller.getCurrent().in(Units.Amp);
    inputs.extenderAppliedOutput = extender.getVoltage();
    inputs.extenderPositionInches =
        extender.getPosition().in(Units.Rotation) / IntakeConstants.Extender.INCHES_TO_MOTOR_ROT;
    inputs.extenderVelocityInchesPerSecond =
        extender.getVelocity().in(Units.RotationsPerSecond)
            / IntakeConstants.Extender.INCHES_TO_MOTOR_ROT;
    inputs.extenderCurrentAmps = extender.getCurrent().in(Units.Amp);
  }

  @Override
  public void setFrontVoltage(double volts) {
    frontRoller.setVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setFrontVel(AngularVelocity vel) {
    frontRoller.setVelocity(vel.div(IntakeConstants.FrontRollers.GEAR_RATIO));
  }

  @Override
  public void configFrontRollers(double kV, double kP, double maxAcceleration) {
    frontRoller.setConfig(kP, 0, 0, kV, 0);
  }

  @Override
  public void setBackVoltage(double volts) {
    backRoller.setVoltage(Units.Volts.of(volts));
  }

  @Override
  public void setBackVel(AngularVelocity vel) {
    backRoller.setVelocity(vel.div(IntakeConstants.FrontRollers.GEAR_RATIO));
  }

  @Override
  public void configBackRollers(double kV, double kP, double maxAcceleration) {
    backRoller.setConfig(kP, 0, 0, kV, 0);
  }

  @Override
  public void setExtenderPos(Distance length) {
    extender.setPosition(
        Units.Rotation.of(length.in(Units.Inch) * IntakeConstants.Extender.INCHES_TO_MOTOR_ROT));
  }

  @Override
  public void configExtender(double kP, double kD) {
    extender.setConfig(kP, 0, kD, 0, 0);
  }

  @Override
  public void setExtenderSensorPosition(Distance position) {
    extender.setState(
        Units.Rotations.of(position.in(Units.Inch) * IntakeConstants.Extender.INCHES_TO_MOTOR_ROT)
            .in(Units.Radian),
        0);
  }

  @Override
  public boolean setIdleMode(IdleMode value) {
    return true;
  }
}
