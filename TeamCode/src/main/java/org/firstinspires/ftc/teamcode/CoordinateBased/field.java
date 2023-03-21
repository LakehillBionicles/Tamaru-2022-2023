package org.firstinspires.ftc.teamcode.CoordinateBased;

import org.checkerframework.checker.units.qual.C;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Set;

public class field {

    public double tile = 23.5;
    public double halfTile = 23.5 / 2;

    public Height targetHeight;
    public double targetX;
    public double targetY;

    public Coordinates currentCoordinates = new Coordinates(0, 0, Height.GROUND);

    public enum ObjectType {
        GROUND_JUNCTION, CONE_STACK, POLE, PARKING_SPOT, TERMINAL
    }

    public enum Location {
        RED_CORNER, BLUE_CORNER
    }

    public enum Height {
        GROUND(0),
        TWO_CONES(300), THREE_CONES(400), FOUR_CONES(500), FIVE_CONES(600),
        LOW_POLE(1400), MID_POLE(2000), HIGH_POLE(2900);

        public final int height;

        Height(int height) {
            this.height = height;
        }

        public int getHeight() {
            return height;
        }
    }

    public enum Heading {
        NORTH, SOUTH, EAST, WEST
    }

    public field(Location location) {
        if (location == Location.BLUE_CORNER) {
            //setScoreMap(1);
        } else {
            //setScoreMap(-1);
        }
    }

    public HashMap<String, Coordinates> scoreMap = new HashMap<>();

    public static class Coordinates {
        public double x;
        public double y;
        public Height z;

        public boolean equals(Object o) {
            Coordinates c = (Coordinates) o;
            return c.x == x && c.y == y && c.z == z;
        }

        public Coordinates(double x, double y, Height z) {
            super();
            this.x = x;
            this.y = y;
            this.z = z;
        }

    }

    public void armScore(String poleID, Coordinates currentCoordinates) {
        Coordinates targetCoordinates = scoreMap.get(poleID);
        targetHeight = targetCoordinates.z;
        if (targetCoordinates.y < currentCoordinates.y) {
            //turret star
            //extend star
        } else {
            //turret port
            //extend port
        }
    }

    public void setScoreMap(int multiplier, Heading heading) {
        switch (heading) {
            case NORTH:
                scoreMap.clear();
                scoreMap.put("T1", new Coordinates(4 * tile, multiplier * (halfTile + tile), Height.HIGH_POLE));
                scoreMap.put("T2", new Coordinates(3 * tile, multiplier * (halfTile + (2 * tile)), Height.HIGH_POLE));
                scoreMap.put("T3", new Coordinates(2 * tile, multiplier * (halfTile + tile), Height.HIGH_POLE));
                scoreMap.put("T4", new Coordinates(3 * tile, multiplier * halfTile, Height.HIGH_POLE));
                scoreMap.put("M1", new Coordinates(4 * tile, multiplier * (halfTile + (2 * tile)), Height.MID_POLE));
                scoreMap.put("M2", new Coordinates(2 * tile, multiplier * (halfTile + (2 * tile)), Height.MID_POLE));
                scoreMap.put("M3", new Coordinates(2 * tile, multiplier * (halfTile), Height.MID_POLE));
                scoreMap.put("M4", new Coordinates(4 * tile, multiplier * (halfTile), Height.MID_POLE));
                scoreMap.put("L1", new Coordinates(2 * tile, -multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L2", new Coordinates(4 * tile, -multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L3", new Coordinates(1 * tile, multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L4", new Coordinates(5 * tile, multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L5", new Coordinates(tile, multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L6", new Coordinates(5 * tile, multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L7", new Coordinates(2 * tile, multiplier * (halfTile), Height.LOW_POLE));
                scoreMap.put("L8", new Coordinates(4 * tile, multiplier * (halfTile), Height.LOW_POLE));
            case EAST:
                scoreMap.clear();
            case SOUTH:
                scoreMap.clear();
            case WEST:
                scoreMap.clear();
        }

    /*public void setJunctionsRedCorner(){
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, -(halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, -(halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, -(halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, -(halfTile+(2*tile)), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, -(halfTile+(2*tile)), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, -(halfTile+(2*tile)), 0.0));
    }

    public void setJunctionsBlueCorner(){
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, -halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, -halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, -halfTile, 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, (halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, (halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, (halfTile+tile), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(tile, (halfTile+(2*tile)), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(3*tile, (halfTile+(2*tile)), 0.0));
        fieldMap.put(ObjectType.GROUND_JUNCTION, new Coordinates(5*tile, (halfTile+(2*tile)), 0.0));
    }

    public void setPolesRedCorner(){
        /////low poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(tile, -halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(5*tile, -halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(tile, -(halfTile+(2*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(5*tile, -(halfTile+(2*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, -(halfTile+(3*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, -(halfTile+(3*tile)),  lowPoleHeight));
        /////mid poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, -halfTile,  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, -halfTile,  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, -(halfTile+(2*tile)),  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, -(halfTile+(2*tile)),  midPoleHeight));
        /////high poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(3*tile, -halfTile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, -halfTile+tile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, -halfTile+tile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(3*tile, -(halfTile+(2*tile)),  highPoleHeight));
    }

    public void setPolesBlueCorner(){
        /////low poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, -halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, -halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(tile, halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(5*tile, halfTile,  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(tile, (halfTile+(2*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(5*tile, (halfTile+(2*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, (halfTile+(3*tile)),  lowPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, (halfTile+(3*tile)),  lowPoleHeight));
        /////mid poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, halfTile,  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, halfTile,  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, (halfTile+(2*tile)),  midPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, (halfTile+(2*tile)),  midPoleHeight));
        /////high poles//////
        fieldMap.put(ObjectType.POLE, new Coordinates(3*tile, halfTile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(2*tile, halfTile+tile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(4*tile, halfTile+tile,  highPoleHeight));
        fieldMap.put(ObjectType.POLE, new Coordinates(3*tile, (halfTile+(2*tile)),  highPoleHeight));
    }

    public void setTerminalsRedCorner(){
        /////close corner//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(0.0, halfTile+tile,  0.0));
        /////substation//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(0.0, -(halfTile+tile),  0.0));
        /////far corner//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(6*tile, -(halfTile+(4*tile)), 0.0));
    }

    public void setTerminalsBlueCorner(){
        /////close corner//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(0.0, -(halfTile+tile),  0.0));
        /////substation//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(0.0, (halfTile+tile),  0.0));
        /////far corner//////
        fieldMap.put(ObjectType.TERMINAL, new Coordinates(6*tile, (halfTile+(4*tile)), 0.0));
    }

    public void setParkingSpots(){
        /////red (1) parking//////
        fieldMap.put(ObjectType.PARKING_SPOT, new Coordinates(2*tile, halfTile+tile,  0.0));
        /////green (3) parking//////
        fieldMap.put(ObjectType.PARKING_SPOT, new Coordinates(2*tile, -(halfTile+tile),  0.0));
        /////blue (2) parking//////
        fieldMap.put(ObjectType.PARKING_SPOT, new Coordinates(2*tile, 0.0,  0.0));
    }*/

    }
}