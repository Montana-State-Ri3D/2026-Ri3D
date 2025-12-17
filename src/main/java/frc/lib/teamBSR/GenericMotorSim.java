package frc.lib.teamBSR;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class GenericMotorSim {

  private DCMotorSim motor;
  private double gearing;

  private ControlMode control = ControlMode.VOLTAGE;

  private double volts;

  private double targetOutput;

  private double kP;
  private double kI;
  private double kD;
  private double kV;
  private double kG;

  private double lastError;

  private double posSum;
  private double lastPos;

  public GenericMotorSim(DCMotor motorType, double gearing, double MOI) {
    motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, MOI, gearing), motorType);
    this.gearing = gearing;
  }

  public void update(double dtSeconds) {
    motor.update(dtSeconds);
    double error;
    double deltaError;

    double adjustedVelocity = getVelocity().in(Units.RPM);
    double adjustedPosition = getPosition().in(Units.Radian);
    switch (control) {
      case VOLTAGE:
        volts = targetOutput;
        break;
      case VELOCITY:
        error = targetOutput - adjustedVelocity;
        deltaError = (error - lastError) / dtSeconds;
        volts = kV * targetOutput + kP * deltaError;
        lastError = error;
        break;
      case POSITION:
        error = targetOutput - adjustedPosition;
        deltaError = (error - lastError) / dtSeconds;
        posSum += (adjustedPosition + lastPos) / 2.0 * dtSeconds;
        volts = kP * error + kI * posSum + kD * deltaError + kG;
        lastError = error;
        break;
      default:
        volts = 0;
        break;
    }

    volts =
        MathUtil.clamp(
            volts, -Constants.MAX_VOLTAGE.in(Units.Volt), Constants.MAX_VOLTAGE.in(Units.Volt));

    motor.setInputVoltage(volts);

    lastPos = adjustedPosition;

    if (volts == 0.0)
      motor.setState(motor.getAngularPositionRad(), motor.getAngularVelocityRadPerSec() / 2.0);
  }

  public void setVoltage(Voltage volts) {
    targetOutput = volts.in(Units.Volts);
    control = ControlMode.VOLTAGE;
  }

  public void setVelocity(AngularVelocity vel) {
    targetOutput = vel.in(Units.RPM);
    control = ControlMode.VELOCITY;
  }

  public void setPosition(Angle angle) {
    targetOutput = angle.in(Units.Radian);
    control = ControlMode.POSITION;
  }

  public void setConfig(double kP, double kI, double kD, double kV, double kG) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kV = kV;
    this.kG = kG;
  }

  public double getVoltage() {
    return volts;
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(motor.getAngularVelocityRPM() * gearing);
  }

  public Angle getPosition() {
    return Units.Radian.of(motor.getAngularPositionRad() * gearing);
  }

  public Current getCurrent() {
    return Units.Amps.of(motor.getCurrentDrawAmps());
  }

  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    motor.setState(angularPositionRad, angularVelocityRadPerSec);
  }

  public enum ControlMode {
    VOLTAGE,
    VELOCITY,
    POSITION
  }
}
