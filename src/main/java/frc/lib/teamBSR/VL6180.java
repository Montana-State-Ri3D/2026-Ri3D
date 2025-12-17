package frc.lib.teamBSR;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VL6180 {

  private final I2C i2c;
  private static final int ADDRESS = 0x29;

  public VL6180(I2C.Port port) {
    i2c = new I2C(port, ADDRESS);
    initializeSensor();
  }

  private void initializeSensor() {
    // Minimal init (from Adafruit Arduino library)
    // Normally you'd port the full init for reliability
    writeByte(0x0207, (byte) 0x01);
    writeByte(0x0208, (byte) 0x01);
    writeByte(0x0096, (byte) 0x00);
    // Additional init registers omitted for brevity
  }

  private void writeByte(int register, byte value) {
    i2c.write(register, value);
  }

  private byte readByte(int register) {
    byte[] buffer = new byte[1];
    i2c.read(register, 1, buffer);
    return buffer[0];
  }

  public Distance getDistance() {
    return Units.Millimeters.of(readByte(0x62) & 0xFF); // Distance in mm
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("VL6180 Distance Inches", getDistance().in(Units.Inches));
  }
}
