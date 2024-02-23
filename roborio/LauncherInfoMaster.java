package frc.robot;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherInfoMaster {
    public List<String> launcherInfoStrings;
    public List<LauncherInfo> launcherInfoList;

    public LauncherInfoMaster() {
        launcherInfoStrings = new ArrayList<String>();
        launcherInfoList = new ArrayList<LauncherInfo>();
        
        try {
            FileReader george = new FileReader("launcherExperimentData.txt");
            String fullFileString = "";
            int currChar = 0;

            while(currChar != -1) {
                currChar = george.read();
                fullFileString += String.valueOf(currChar);
            }

            launcherInfoStrings = new ArrayList<String>(Arrays.asList(fullFileString.split("?")));

            for(int i = 0; i < launcherInfoStrings.size(); i++) {
                String distanceString = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("d:") + 2, launcherInfoStrings.get(i).indexOf("a0:"));
                String angle0String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a0:") + 3, launcherInfoStrings.get(i).indexOf("a1:"));
                String angle1String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a1:") + 3, launcherInfoStrings.get(i).indexOf("a2:"));
                String angle2String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a2:") + 3, launcherInfoStrings.get(i).indexOf("s0:"));
                String speed0String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s0:") + 3, launcherInfoStrings.get(i).indexOf("s1:"));
                String speed1String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s1:") + 3, launcherInfoStrings.get(i).indexOf("s2:"));
                String speed2String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s2:") + 3, launcherInfoStrings.get(i).indexOf("?"));

                launcherInfoList.set(i, new LauncherInfo(Double.valueOf(distanceString), 
                    new double[]{Double.valueOf(angle0String), Double.valueOf(angle1String), Double.valueOf(angle2String)}, 
                    new double[]{Double.valueOf(speed0String), Double.valueOf(speed1String), Double.valueOf(speed2String)}));
            }

            george.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    public LauncherInfo get(double distance) {
        if(launcherInfoList.get(0).distance > distance) {
            SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            return launcherInfoList.get(0);
        } else if(launcherInfoList.get(launcherInfoList.size() - 1).distance < distance) {
            SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            return launcherInfoList.get(launcherInfoList.size() - 1);
        }

        int bottomIndex = 0;

        for(int i = 0; i < launcherInfoList.size(); i++) {
            if(launcherInfoList.get(i).distance < distance) {
                bottomIndex++;
            }
        }

        if(launcherInfoList.get(bottomIndex).distance == distance) {
            SmartDashboard.putString("Distance Range Status: ", "DISTANCE RANGE READY");
            return launcherInfoList.get(bottomIndex);
        } else {
            SmartDashboard.putString("Distance Range Status: ", "DISTANCE RANGE READY");
            return launcherInfoList.get(bottomIndex).interpolate(launcherInfoList.get(bottomIndex + 1), distance);
        }
    }

    public void storeSpeaker(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:" + angle + "a1:0a2:0s0:" + speed + "s1:0s2:0?";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{angle, 0d, 0d}, new double[]{speed, 0d, 0d});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[0] = angle;
                actualInfo.speeds[0] = speed;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a0:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a1:"), launcherInfoStrings.get(i).indexOf("s0:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s1:")));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void storeAmp(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:0a1:" + angle + "a2:0s0:0s1:" + speed + "s2:0?";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{0d, angle, 0d}, new double[]{0d, speed, 0d});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[1] = angle;
                actualInfo.speeds[1] = speed;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a1:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a2:"), launcherInfoStrings.get(i).indexOf("s1:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s2:")));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void storeTrap(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:0a1:0a2:" + angle + "s0:0s1:0s2:" + speed + "?";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{0d, 0d, angle}, new double[]{0d, 0d, speed});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[2] = angle;
                actualInfo.speeds[2] = speed;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a2:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s0:"), launcherInfoStrings.get(i).indexOf("s2:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?")));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void updateDataFile() {
        try {
            FileWriter george = new FileWriter("launcherExperimentData.txt", false);
            String fullFileString = "";

            for(int i = 0; i < launcherInfoStrings.size(); i++) {
                fullFileString += launcherInfoStrings.get(i);
            }

            george.write(fullFileString);
            george.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }
}
