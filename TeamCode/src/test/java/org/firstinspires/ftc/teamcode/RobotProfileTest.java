package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.testng.annotations.Test;

import java.io.File;
import java.util.HashMap;


public class RobotProfileTest {
    @Test
    public void robotProfileTest() {
        RobotProfile profile = new RobotProfile();
        profile.populateInitValue();

        GsonBuilder builder = new GsonBuilder();
        Gson gson = new GsonBuilder().setPrettyPrinting().create();
        String json = gson.toJson(profile);
        System.out.println(json);

        try {
            File f = new File("../../profile.json");
            profile.saveToFile(f);
            RobotProfile profile2 = RobotProfile.loadFromFile(f);
        } catch (Exception ex) {
            System.out.println(ex);
        }
    }

}
