package frc.lib.teamBSR;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class VL6180 {

    /* =========================
     * Registers
     * ========================= */
    private static final int REG_IDENTIFICATION_MODEL_ID = 0x000;

    private static final int REG_SYSTEM_HISTORY_CTRL = 0x012;
    private static final int REG_SYSTEM_INTERRUPT_CONFIG = 0x014;
    private static final int REG_SYSTEM_INTERRUPT_CLEAR = 0x015;
    private static final int REG_SYSTEM_FRESH_OUT_OF_RESET = 0x016;

    private static final int REG_SYSRANGE_START = 0x018;
    private static final int REG_SYSRANGE_INTERMEASUREMENT_PERIOD = 0x01B;
    private static final int REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET = 0x024;

    private static final int REG_SYSALS_START = 0x038;
    private static final int REG_SYSALS_ANALOGUE_GAIN = 0x03F;
    private static final int REG_SYSALS_INTEGRATION_PERIOD_HI = 0x040;
    private static final int REG_SYSALS_INTEGRATION_PERIOD_LO = 0x041;

    private static final int REG_RESULT_RANGE_STATUS = 0x04D;
    private static final int REG_RESULT_INTERRUPT_STATUS_GPIO = 0x04F;
    private static final int REG_RESULT_ALS_VAL = 0x050;
    private static final int REG_RESULT_HISTORY_BUFFER_0 = 0x052;
    private static final int REG_RESULT_RANGE_VAL = 0x062;

    private static final int DEFAULT_I2C_ADDR = 0x29;

    /* =========================
     * ALS Gains
     * ========================= */
    public static final int ALS_GAIN_1 = 0x06;
    public static final int ALS_GAIN_1_25 = 0x05;
    public static final int ALS_GAIN_1_67 = 0x04;
    public static final int ALS_GAIN_2_5 = 0x03;
    public static final int ALS_GAIN_5 = 0x02;
    public static final int ALS_GAIN_10 = 0x01;
    public static final int ALS_GAIN_20 = 0x00;
    public static final int ALS_GAIN_40 = 0x07;

    /* =========================
     * Instance fields
     * ========================= */
    private final I2C i2c;
    private int offset = 0;

    /* =========================
     * Constructor
     * ========================= */
    public VL6180(Port port) {
        this(port, DEFAULT_I2C_ADDR);
    }

    public VL6180(Port port, int address) {
        i2c = new I2C(port, address);

        int model = read8(REG_IDENTIFICATION_MODEL_ID);
        if (model != 0xB4) {
            throw new RuntimeException("VL6180X not found (Model ID = " + model + ")");
        }

        loadSettings();
        write8(REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);

        // Enable range history
        write8(REG_SYSTEM_HISTORY_CTRL, 0x01);
    }

    /* =========================
     * Public API
     * ========================= */

    public Distance getRange() {
        return Units.Millimeters.of(isContinuousModeEnabled() ? readRangeContinuous() : readRangeSingle());
    }

    public void startContinuous(int periodMs) {
        if (periodMs < 20 || periodMs > 2550) {
            throw new IllegalArgumentException("Period must be 20–2550 ms");
        }
        int regVal = (periodMs / 10) - 1;
        write8(REG_SYSRANGE_INTERMEASUREMENT_PERIOD, regVal);
        write8(REG_SYSRANGE_START, 0x03);
    }

    public void stopContinuous() {
        write8(REG_SYSRANGE_START, 0x01);
    }

    public boolean isContinuousModeEnabled() {
        return (read8(REG_SYSRANGE_START) & 0x01) == 1;
    }

    public void setOffsetMm(int offset) {
        this.offset = offset;
        write8(REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset & 0xFF);
    }

    public int getRangeStatus() {
        return read8(REG_RESULT_RANGE_STATUS) >> 4;
    }

    public double readLux(int gain) {
        int reg = read8(REG_SYSTEM_INTERRUPT_CONFIG);
        reg &= ~0x38;
        reg |= (0x4 << 3);
        write8(REG_SYSTEM_INTERRUPT_CONFIG, reg);

        write8(REG_SYSALS_INTEGRATION_PERIOD_HI, 0);
        write8(REG_SYSALS_INTEGRATION_PERIOD_LO, 100);

        gain = Math.min(gain, ALS_GAIN_40);
        write8(REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain);
        write8(REG_SYSALS_START, 0x01);

        while (((read8(REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7) != 4) {}

        int als = read16(REG_RESULT_ALS_VAL);
        write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

        double lux = als * 0.32;

        switch (gain) {
            case ALS_GAIN_1_25: lux /= 1.25; break;
            case ALS_GAIN_1_67: lux /= 1.67; break;
            case ALS_GAIN_2_5: lux /= 2.5; break;
            case ALS_GAIN_5: lux /= 5; break;
            case ALS_GAIN_10: lux /= 10; break;
            case ALS_GAIN_20: lux /= 20; break;
            case ALS_GAIN_40: lux /= 40; break;
            default: break;
        }

        return lux;
    }

    /* =========================
     * Internal helpers
     * ========================= */

    private int readRangeSingle() {
        while ((read8(REG_RESULT_RANGE_STATUS) & 0x01) == 0) {}
        write8(REG_SYSRANGE_START, 0x01);
        return readRangeContinuous();
    }

    private int readRangeContinuous() {
        while ((read8(REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04) == 0) {}
        int range = read8(REG_RESULT_RANGE_VAL);
        write8(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
        return range;
    }

    private void loadSettings() {
        int[][] init = {
            {0x0207,0x01},{0x0208,0x01},{0x0096,0x00},{0x0097,0xFD},
            {0x00E3,0x00},{0x00E4,0x04},{0x00E5,0x02},{0x00E6,0x01},
            {0x00E7,0x03},{0x00F5,0x02},{0x00D9,0x05},{0x00DB,0xCE},
            {0x00DC,0x03},{0x00DD,0xF8},{0x009F,0x00},{0x00A3,0x3C},
            {0x00B7,0x00},{0x00BB,0x3C},{0x00B2,0x09},{0x00CA,0x09},
            {0x0198,0x01},{0x01B0,0x17},{0x01AD,0x00},{0x00FF,0x05},
            {0x0100,0x05},{0x0199,0x05},{0x01A6,0x1B},{0x01AC,0x3E},
            {0x01A7,0x1F},{0x0030,0x00}
        };

        for (int[] r : init) write8(r[0], r[1]);

        write8(0x0011, 0x10);
        write8(0x010A, 0x30);
        write8(0x003F, 0x46);
        write8(0x0031, 0xFF);
        write8(0x0040, 0x63);
        write8(0x002E, 0x01);
        write8(0x001B, 0x09);
        write8(0x003E, 0x31);
        write8(0x0014, 0x24);
    }

    private void write8(int reg, int data) {
        byte[] buffer = {
            (byte)(reg >> 8),
            (byte)(reg & 0xFF),
            (byte)(data & 0xFF)
        };
        i2c.writeBulk(buffer);
    }

    private int read8(int reg) {
        byte[] addr = {(byte)(reg >> 8), (byte)(reg & 0xFF)};
        byte[] data = new byte[1];
        i2c.writeBulk(addr);
        i2c.readOnly(data, 1);
        return data[0] & 0xFF;
    }

    private int read16(int reg) {
        byte[] addr = {(byte)(reg >> 8), (byte)(reg & 0xFF)};
        byte[] data = new byte[2];
        i2c.writeBulk(addr);
        i2c.readOnly(data, 2);
        return ((data[0] & 0xFF) << 8) | (data[1] & 0xFF);
    }
}
