// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.paths;

import java.util.ArrayList;

import raiderlib.path.WayPoint;
import raiderlib.path.Path;

/** Add your docs here. */
public class Forward extends Path {
    @Override
    public ArrayList<WayPoint> get_waypoints() {
        ArrayList<WayPoint> points = new ArrayList<>();
        points.add(new WayPoint(0, 0, 0));
        points.add(new WayPoint(45, 0, 0));
        points.add(new WayPoint(90, 0, 0));
        return points;
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
