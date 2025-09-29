package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DriveModuleSpark extends SubsystemBase {

  private final SparkMax motor;
  private final int id;

  public DriveModuleSpark(int id) {
    this.id = id;
    motor = new SparkMax(CanIDs.DRIVE_CAN_IDS[id], MotorType.kBrushless);
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
    motor.getClosedLoopController().setReference(vel.div(DriveConstants.MAX_LINEAR_SPEED).magnitude(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public LinearVelocity getVel() {
    return DriveConstants.MAX_LINEAR_SPEED.times(motor.get());
  }

  public Distance getAbsolutePosition() {
    return DriveConstants.WHEEL_CIRC.times(
        motor.getAbsoluteEncoder().getPosition() / DriveConstants.GEAR_RATIO);
  }

  public void updateMotorConfig(double p, double i, double d, double ff){
    SparkMaxConfig config = DriveConstants.MOTOR_CONFIG(id);
    config.closedLoop.pidf(p, i, d, ff);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
