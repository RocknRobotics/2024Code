package frc.robot;

public class LauncherInfo {
    public int voltage;
    public double distance;
    public double angles[];
    public double speeds[];
    public boolean has[];

    public LauncherInfo() {
        voltage = 0;
        distance = 0.0;
        angles = new double[3];
        speeds = new double[3];
        has = new boolean[3];
    }

    public LauncherInfo(int voltage, double distance, double[] angles, double[] speeds, boolean has[]) {
        this.voltage = voltage;
        this.distance = distance;
        this.angles = angles;
        this.speeds = speeds;
        this.has = has;
    }

    public LauncherInfo interpolateVoltage(LauncherInfo other, int voltage) {
        LauncherInfo output = new LauncherInfo();
        output.voltage = voltage;
        output.distance = Math.abs(voltage - this.voltage) / Math.abs(this.voltage - other.voltage) * distance + 
            Math.abs(voltage - other.voltage) / Math.abs(this.voltage - other.voltage) * other.distance;
        output.has[0] = this.has[0] && other.has[0];
        output.has[1] = this.has[1] && other.has[1];
        output.has[2] = this.has[2] && other.has[2];

        for(int i = 0; i < 3; i++) {
            output.angles[i] = Math.abs(voltage - this.voltage) / Math.abs(this.voltage - other.voltage) * angles[i] + 
                Math.abs(voltage - other.voltage) / Math.abs(this.voltage - other.voltage) * other.angles[i];
            output.speeds[i] = Math.abs(voltage - this.voltage) / Math.abs(this.voltage - other.voltage) * speeds[i] + 
                Math.abs(voltage - other.voltage) / Math.abs(this.voltage - other.voltage) * other.speeds[i];
        }

        return output;
    }

    public LauncherInfo interpolateDistance(LauncherInfo other, double distance) {
        LauncherInfo output = new LauncherInfo();
        output.distance = distance;
        output.has[0] = this.has[0] && other.has[0];
        output.has[1] = this.has[1] && other.has[1];
        output.has[2] = this.has[2] && other.has[2];

        for(int i = 0; i < 3; i++) {
            output.angles[i] = Math.abs(distance - this.distance) / Math.abs(this.distance - other.distance) * angles[i] + 
                Math.abs(distance - other.distance) / Math.abs(this.distance - other.distance) * other.angles[i];
            output.speeds[i] = Math.abs(distance - this.distance) / Math.abs(this.distance - other.distance) * speeds[i] + 
                Math.abs(distance - other.distance) / Math.abs(this.distance - other.distance) * other.speeds[i];
        }

        return output;
    }
}