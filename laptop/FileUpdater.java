package frc.robot.Laptop;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class is necessary since I don't think the roboRio can communicate the updated files back to the computer
public class FileUpdater {
    public static void initialization(boolean runThis) {
        if(runThis) {
            NetworkTableInstance inst = NetworkTableInstance.create();
            inst.startClient4("laptopFileClient");
            inst.setServer("10.36.92.20", NetworkTableInstance.kDefaultPort4);
            SmartDashboard.setNetworkTableInstance(inst);

            SmartDashboard.putString("File Write Contents", "");
            SmartDashboard.putString("File Write Path", "");

            Thread fileHandler = new Thread(() -> {
                while(true) {
                    String writePath = SmartDashboard.getString("File Write Path", "");

                    if(!writePath.equals("")) {
                        String writeContents = SmartDashboard.getString("File Write Contents", "");
                        
                        try {
                            FileWriter author = new FileWriter(writePath, false);
                            author.write(writeContents, 0, writeContents.length());
                            author.close();

                        } catch(IOException e) {
                            e.printStackTrace();
                        } finally {
                            SmartDashboard.putString("File Write Path", "");
                            SmartDashboard.putString("File Write Contents", "");
                        }
                    }

                    try {
                        Thread.sleep(500);
                    } catch(InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            });
            fileHandler.start();
        }
    }
}
