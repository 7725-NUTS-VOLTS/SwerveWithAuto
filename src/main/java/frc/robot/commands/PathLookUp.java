// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class PathLookUp {

    public static PathContainer getContainer(String path){
        PathContainer container = null;

        switch(path){
            case "path0":
            container = new PathContainer(path, null, 1.25, true, true);
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
        double[] speed;

        switch(speeds){
            case ONE:
            return speed = new double[]{1,1};
            

            case TWO:
            return speed = new double[]{2,2};
            
            case THREE:
            return speed = new double[]{3,3};
           


            case FOUR:
            speed = new double[]{4,4};
            break;

            case FIVE:
            speed = new double[]{5,5};
            break;
        }

        return new double[] {1,1};

    }
}
