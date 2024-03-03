package frc.robot.Laptop;

public class LaptopMain {
    static FileUpdater updater = new FileUpdater();

    public static void main(String[] args) {
        initialize();
    }
    
    public static void initialize() {
        updater.initialization(true);
    }
}
