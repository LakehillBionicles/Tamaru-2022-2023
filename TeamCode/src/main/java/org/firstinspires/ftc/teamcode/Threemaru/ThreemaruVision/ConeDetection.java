package org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeDetection extends OpenCvPipeline {

    /*
        YELLOW  = Parking Left
        CYAN    = Parking Middle
        MAGENTA = Parking Right
         */
    public enum ParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE,
        THIRTEEN, FOURTEEN, FIFTEEN, SIXTEEN, SEVENTEEN, EIGHTEEN, NINETEEN, TWENTY, TWENTYONE,
        TWENTYTWO, TWENTYTHREE, TWENTYFOUR
    }

    public enum RedParkingPosition {
        NOTSEEN, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE
    }
    // Width and height for the bounding box
    // Color definitions
    private final Scalar
            BLUE = new Scalar(0, 0, 255),
            GREEN = new Scalar(0, 255, 0),
            CYAN = new Scalar(0, 255, 255),
            RED = new Scalar(255, 0, 0),

            WHITE = new Scalar(255, 255, 255),

            YELLOW = new Scalar(255,255,0);

    // Anchor point definitions
     // Running variable storing the parking position
    private static volatile ConeDetection.ParkingPosition bluePosition = ParkingPosition.ONE;
    private static volatile ConeDetection.RedParkingPosition redPosition = RedParkingPosition.NOTSEEN;
    int anchorWidth = 4;

    int bottomHeight = 1;
    int boxWidth = 20;
    static int addedBlueBarPositions = 0;
    static int positionOfBlueObject = 0;
    static int numberOfBlueBars = 0;
    static int addedRedBarPositions = 0;
    static int positionOfRedObject = 0;
    static int numberOfRedBars = 0;

    static int addedYellowBarPositions = 0;
    static int positionOfYellowObject = 0;
    static int numberOfYellowBars = 0;

    static double blueDistance = 0;
    static double redDistance = 0;
    static double widthOfInput;

    static double heightOfInput;

    static double redTolerance = 1;

    static double blueTolerance = 0.7;

    static double yellowTolerance= 4;//Yellow tolerance needs to be roughly twice as high as red and blue tolerance
    public static Scalar telemetryRGBValues;


    @Override
    public Mat processFrame(Mat input) {
        widthOfInput = input.width();
        heightOfInput = input.height();
        addedBlueBarPositions = 0;
        numberOfBlueBars = 0;
        addedRedBarPositions = 0;
        positionOfRedObject = 0;
        numberOfRedBars = 0;
        addedYellowBarPositions = 0;
        positionOfYellowObject = 0;
        numberOfYellowBars = 0;
        // Get the submat frame, and then sum all the values
        //Used for telemetry will remove
        Scalar colors;
        for (int i = 1; i < widthOfInput - anchorWidth; i++) {
            Point blueBarPoint1 = new Point(i, 150);
            Point blueBarPoint2 = new Point(i + anchorWidth, heightOfInput);
            Mat MatArea1 = input.submat(new Rect(blueBarPoint1, blueBarPoint2));
            colors = Core.sumElems(MatArea1);
            if ((colors.val[2] / (colors.val[1] + colors.val[0])) > blueTolerance) {
                addedBlueBarPositions = addedBlueBarPositions + i;
                numberOfBlueBars++;
            }
            if ((colors.val[0] / (colors.val[1] + colors.val[2])) > redTolerance) {
                addedRedBarPositions = addedRedBarPositions + i;
                numberOfRedBars++;
            }
            if (((colors.val[0]+colors.val[1])/colors.val[2])>=yellowTolerance){
                addedYellowBarPositions = addedYellowBarPositions + i;
                numberOfYellowBars++;
            }
            MatArea1.release();
        }
        if (numberOfBlueBars > 0) {
            positionOfBlueObject = addedBlueBarPositions / numberOfBlueBars;
        }
        if (numberOfRedBars > 0) {
            positionOfRedObject = addedRedBarPositions / numberOfRedBars;
        }
        if (numberOfYellowBars > 0) {
            positionOfYellowObject = addedYellowBarPositions / numberOfYellowBars;
        }
        Point blueBar_pointBlue1A = new Point(
                positionOfBlueObject, 150);
        Point blueBar_pointBlue1B = new Point(
                positionOfBlueObject + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                blueBar_pointBlue1A,
                blueBar_pointBlue1B,
                BLUE,
                2
        );
        Point redBar_pointRed1A = new Point(
                positionOfRedObject, 150);
        Point redBar_pointRed1B = new Point(
                positionOfRedObject + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                redBar_pointRed1A,
                redBar_pointRed1B,
                RED,
                2
        );
        Point yellowBar_pointBlue1A = new Point(
                positionOfYellowObject, 150);
        Point yellowBar_pointBlue1B = new Point(
                positionOfYellowObject + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                yellowBar_pointBlue1A,
                yellowBar_pointBlue1B,
                YELLOW,
                2
        );
        Point middlePoint1A = new Point(
                ((widthOfInput / 2) + 51) - 4, 150);
        Point middlePoint1B = new Point(
                (widthOfInput / 2) + 51 + 4, heightOfInput);
        Imgproc.rectangle(
                input,
                middlePoint1A,
                middlePoint1B,
                GREEN,
                2
        );
        telemetryRGBValues= Core.sumElems(input.submat(new Rect(middlePoint1A, middlePoint1B)));
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public static int getNumberOfRedBars(){return numberOfRedBars;}
    public static int getNumberOfBlueBars(){return numberOfBlueBars;}
    public static int getNumberOfYellowBars(){return numberOfYellowBars;}
    public static Scalar getColorValuesOfObject(){return telemetryRGBValues;}
    public static double getRedConePosition(){return positionOfRedObject;}
    public static double getBlueConePosition(){return positionOfBlueObject;}
    public static double getYellowConePosition(){return positionOfYellowObject;}

    public static double getImageWidth(){return widthOfInput;}
    //This Doesn't work yet
    public static double getBlueDistance(){
        return blueDistance;
    }
    public static double getRedDistance(){
        return redDistance;
    }
}


