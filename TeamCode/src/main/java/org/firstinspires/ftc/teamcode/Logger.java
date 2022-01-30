package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.nio.Buffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

// Written by Lucas
// Oct. 12, 2019

public class Logger {
    static PrintWriter pw;

    public static void init() {
        try {
            String timestamp = new SimpleDateFormat("yyyyMMdd-HHmm", Locale.US).format(new Date());
            File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/driver_" + timestamp + ".txt");

            pw = new PrintWriter(new BufferedWriter(new FileWriter(file)));
        } catch(IOException e) {
            throw new RuntimeException("cannot write to file", e);
        }
    }
    public static void logFile(String message) {
        try {
            if (pw == null) {
                init();
            }
            String timestamp = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date());

            pw.println(timestamp + " " + message);
            pw.flush();     // for now
        } catch (Exception e) {
            throw new RuntimeException("cannot write to file");
        }
    }

    public static void flushToFile()
    {
        try {
            pw.flush();
        } catch(Exception e) {
            throw new RuntimeException("cannot flush");
        }
    }

    public static void folderCleanup(String startsWith, String endsWith, int count) {
        // could use wildcardfilter, but not want to include another Library
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/");
        ArrayList<File> fileList = new ArrayList<File>();
        for(File f : file.listFiles()) {
            if (f.isFile() && f.getName().startsWith(startsWith) && f.getName().endsWith(endsWith)) {
                fileList.add(f);
            }
        }
    }
}
