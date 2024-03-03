package frc.robot;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherInfoMaster {
    public List<String> launcherInfoStrings;
    public List<LauncherInfo> launcherInfoList;

    public LauncherInfoMaster() {
        launcherInfoStrings = new ArrayList<String>();
        launcherInfoList = new ArrayList<LauncherInfo>();
        
        try {
            FileReader george = new FileReader(Filesystem.getDeployDirectory().toString() + "/launcherExperimentData.txt");
            String fullFileString = "";
            int currChar = 0;

            while(currChar != -1) {
                currChar = george.read();
                fullFileString += String.valueOf((char) currChar);
            }

            fullFileString = fullFileString.substring(0, fullFileString.length() - 1);

            launcherInfoStrings = new ArrayList<String>(Arrays.asList(fullFileString.split("!")));

            for(int i = 0; i < launcherInfoStrings.size(); i++) {
                String distanceString = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("d:") + 2, launcherInfoStrings.get(i).indexOf("a0:"));
                String angle0String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a0:") + 3, launcherInfoStrings.get(i).indexOf("a1:"));
                String angle1String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a1:") + 3, launcherInfoStrings.get(i).indexOf("a2:"));
                String angle2String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a2:") + 3, launcherInfoStrings.get(i).indexOf("s0:"));
                String speed0String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s0:") + 3, launcherInfoStrings.get(i).indexOf("s1:"));
                String speed1String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s1:") + 3, launcherInfoStrings.get(i).indexOf("s2:"));
                String speed2String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s2:") + 3, launcherInfoStrings.get(i).indexOf("?"));
                String has0String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 1, launcherInfoStrings.get(i).indexOf("?") + 2);
                String has1String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 2, launcherInfoStrings.get(i).indexOf("?") + 3);
                String has2String = launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 3, launcherInfoStrings.get(i).indexOf("?") + 4);

                launcherInfoList.add(new LauncherInfo(Double.valueOf(distanceString), 
                    new double[]{Double.valueOf(angle0String), Double.valueOf(angle1String), Double.valueOf(angle2String)}, 
                    new double[]{Double.valueOf(speed0String), Double.valueOf(speed1String), Double.valueOf(speed2String)},
                    new boolean[]{has0String.equals("Y"), has1String.equals("Y"), has2String.equals("Y")}));
            }

            george.close();
        } catch(IOException e) {
            e.printStackTrace();
        }
    }

    public LauncherInfo get(double distance, int index) {
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
            if(launcherInfoList.get(bottomIndex).has[index]) {
                SmartDashboard.putString("Distance Range Status: ", "DISTANCE RANGE READY");
            } else {
                SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            }

            return launcherInfoList.get(bottomIndex);
        } else {
            if(launcherInfoList.get(bottomIndex).has[index]) {
                SmartDashboard.putString("Distance Range Status: ", "DISTANCE RANGE READY");
            } else {
                SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            }

            return launcherInfoList.get(bottomIndex).interpolate(launcherInfoList.get(bottomIndex + 1), distance);
        }
    }

    public void storeSpeaker(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:" + angle + "a1:0a2:0s0:" + speed + "s1:0s2:0?YNN!";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{angle, 0d, 0d}, new double[]{speed, 0d, 0d}, new boolean[]{true, false, false});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[0] = angle;
                actualInfo.speeds[0] = speed;
                actualInfo.has[0] = true;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a0:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a1:"), launcherInfoStrings.get(i).indexOf("s0:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s1:"), launcherInfoStrings.get(i).indexOf("?") + 1) + 
                "Y" + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 2));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void storeAmp(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:0a1:" + angle + "a2:0s0:0s1:" + speed + "s2:0?NYN!";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{0d, angle, 0d}, new double[]{0d, speed, 0d}, new boolean[]{false, true, false});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[1] = angle;
                actualInfo.speeds[1] = speed;
                actualInfo.has[1] = true;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a1:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("a2:"), launcherInfoStrings.get(i).indexOf("s1:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s2:"), launcherInfoStrings.get(i).indexOf("?") + 2) + 
                "Y" + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 3));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void storeTrap(double distance, double angle, double speed) {
        String launcherString = "d:" + distance + "a0:0a1:0a2:" + angle + "s0:0s1:0s2:" + speed + "?NNY!";
        LauncherInfo actualInfo = new LauncherInfo(distance, new double[]{0d, 0d, angle}, new double[]{0d, 0d, speed}, new boolean[]{false, false, true});
        int index = 0;

        for(int i = 0; i < launcherInfoStrings.size() && launcherInfoList.get(i).distance <= distance; i++) {
            if(launcherInfoList.get(i).distance == distance) {
                actualInfo = launcherInfoList.get(i);
                actualInfo.angles[2] = angle;
                actualInfo.speeds[2] = speed;

                launcherInfoStrings.set(i, launcherInfoStrings.get(i).substring(0, launcherInfoStrings.get(i).indexOf("a2:") + 3) + 
                angle + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("s0:"), launcherInfoStrings.get(i).indexOf("s2:" + 3)) + 
                speed + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?"), launcherInfoStrings.get(i).indexOf("?") + 3) + 
                "Y" + launcherInfoStrings.get(i).substring(launcherInfoStrings.get(i).indexOf("?") + 4));

                return;
            }

            index++;
        }

        launcherInfoStrings.add(index, launcherString);
        launcherInfoList.add(index, actualInfo);
    }

    public void updateDataFile() {
        String fullFileString = "";
        String filePath = "C:/robotData/launcherExperimentData.txt";
        for(int i = 0; i < launcherInfoStrings.size(); i++) {
            fullFileString += launcherInfoStrings.get(i);
        }

        SmartDashboard.putString("File Write Path", filePath);
        SmartDashboard.putString("File Write Contents", fullFileString);
        /* 
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
        */
    }
}
