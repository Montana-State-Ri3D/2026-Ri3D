package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DriveModuleSpark extends SubsystemBase {

  private SparkMax motor;
  private final int id;

  public DriveModuleSpark(int id) {
    this.id = id;
    motor = new SparkMax(CanIDs.DRIVE_CAN_IDS[id], MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.absoluteEncoder.inverted(Constants.DriveConstants.MOTOR_INVERTS[id]);
    config.inverted(Constants.DriveConstants.MOTOR_INVERTS[id]);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Drive/Module" + id + "/voltage", motor.getAppliedOutput());
    Logger.recordOutput("Drive/Module" + id + "/vel", getVel());
    Logger.recordOutput("Drive/Module" + id + "/current", motor.getOutputCurrent());
  }

  public void setVoltage(Voltage volts) {
    motor.setVoltage(volts);
  }

  public void setVelocity(LinearVelocity vel) {
    motor.set(vel.div(DriveConstants.MAX_LINEAR_SPEED).magnitude());
  }

  public LinearVelocity getVel() {
    return DriveConstants.MAX_LINEAR_SPEED.times(motor.get());
  }

  public Distance getAbsolutePosition() {
    return DriveConstants.WHEEL_CIRC.times(
        motor.getAbsoluteEncoder().getPosition() / DriveConstants.GEAR_RATIO);
  }
}
