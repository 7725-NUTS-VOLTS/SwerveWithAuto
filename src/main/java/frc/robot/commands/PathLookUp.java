// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class PathLookUp {

    public static PathContainer getContainer(String path){
        PathContainer container = null;

        switch(path){
            case "New Path":
            container = new PathContainer("New Path", getSpeeds(SPEEDS.ONE), 1.25, true, true);
            break;
        }
        return container;
    }

    private enum SPEEDS{
        ONE,
        TWO,
        THREE,
        FOUR, 
        FIVE
    }

    private static double[] getSpeeds(SPEEDS speeds){
        double[] speed = new double[6];

        switch(speeds){
            case ONE:
            speed = new double[]{1,1};
            break;
            case TWO:
            speed = new double[]{2,2};
            break;
            case THREE:
            speed = new double[]{3,3};
            break;
            case FOUR:
            speed = new double[]{4,4};
            break;

            case FIVE:
            speed = new double[]{5,5};
            break;
        }

        return speed;

    }
}
