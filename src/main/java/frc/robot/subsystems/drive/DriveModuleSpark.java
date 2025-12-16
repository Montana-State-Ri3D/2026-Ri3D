package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class DriveModuleSpark extends SubsystemBase {

  @AutoLog
  public static class DriveModuleSparkInputs {
    public double voltage;
    public double relativePosMeters;
    public double relativePosRotations;
    public double velMetersPerSecond;
    public double desiredVelMetersPerSecond;
    public double current;
  }

  private final DriveModuleSparkInputsAutoLogged inputs = new DriveModuleSparkInputsAutoLogged();

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final int id;

  private LinearVelocity desiredVel = MetersPerSecond.of(0);

  public DriveModuleSpark(int id) {
    this.id = id;
    motor = new SparkMax(CanIDs.DRIVE_CAN_IDS[id], MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  @Override
  public void periodic() {
    inputs.voltage = motor.getAppliedOutput();
    inputs.relativePosMeters = getRelativePosition().in(Units.Meter);
    inputs.relativePosRotations = encoder.getPosition();
    inputs.velMetersPerSecond = getVel().in(Units.MetersPerSecond);
    inputs.desiredVelMetersPerSecond = desiredVel.in(Units.MetersPerSecond);
    inputs.current = motor.getOutputCurrent();

    Logger.processInputs("Drive/Modules/Module" + id, inputs);
  }

  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  public void setVelocity(LinearVelocity vel) {
    desiredVel = vel;
    motor
        .getClosedLoopController()
        .setReference(
            Units.RadiansPerSecond.of(vel.div(DriveConstants.WHEEL_RAD).magnitude()).in(Units.RPM)
                / DriveConstants.GEAR_RATIO,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0);
  }

  public LinearVelocity getVel() {
    return Units.MetersPerSecond.of(
        Units.RPM.of(encoder.getVelocity()).in(Units.RadiansPerSecond)
            * DriveConstants.WHEEL_RAD.in(Units.Meter));
  }

  public Distance getRelativePosition() {
    return Units.Meter.of(
        Units.Rotations.of(encoder.getPosition()).in(Units.Radians)
            * DriveConstants.WHEEL_RAD.in(Units.Meter));
  }

  public void updateMotorConfig(double v, double p) {
    SparkMaxConfig config = DriveConstants.MOTOR_CONFIG(id);
    config.closedLoop.pidf(p, 0, 0, v);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
