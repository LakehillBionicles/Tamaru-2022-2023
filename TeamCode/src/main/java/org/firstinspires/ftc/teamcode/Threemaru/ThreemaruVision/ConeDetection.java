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
    int otherCenterOfBars = 0;
    int otherRedCenterOfBars = 0;

    static int differentBlueAddedBars = 0;
    static int positionOfBlueObject = 0;
    static int numberOfBlueBars = 0;
    static int differentRedAddedBars = 0;
    static int positionOfRedObject = 0;
    static int differentRedNumberOfBars = 0;

    static int differentYellowAddedBars = 0;
    static int differentYellowCenterOfBars = 0;
    static int differentYellowNumberOfBars = 0;

    static double blueDistance = 0;
    static double redDistance = 0;
    static double widthOfInput;

    static double heightOfInput;

    static double redTolerance = 1;

    static double blueTolerance = 0.7;

    static double yellowTolerance= 3;//Yellow tolerance needs to be roughly twice as high as red and blue tolerance

    static double doubleBluePosition;

    static double doubleRedPosition;

    static double doubleYellowPosition;


    @Override
    public Mat processFrame(Mat input) {
        widthOfInput = input.width();
        heightOfInput = input.height();

        otherCenterOfBars = 0;
        otherRedCenterOfBars = 0;
        differentBlueAddedBars = 0;
        positionOfBlueObject = 0;
        numberOfBlueBars = 0;
        differentRedAddedBars = 0;
        positionOfRedObject = 0;
        differentRedNumberOfBars = 0;
        differentYellowAddedBars = 0;
        differentYellowCenterOfBars = 0;
        differentYellowNumberOfBars = 0;
        // Get the submat frame, and then sum all the values
        //Used for telemetry will remove
        Scalar colors;
        for (int i = 1; i < widthOfInput - anchorWidth; i++) {
            Point blueBarPoint1 = new Point(i, 60);
            Point blueBarPoint2 = new Point(i + anchorWidth, heightOfInput);
            Mat MatArea1 = input.submat(new Rect(blueBarPoint1, blueBarPoint2));
            colors = Core.sumElems(MatArea1);
            if ((colors.val[2] / (colors.val[1] + colors.val[0])) > blueTolerance) {
                differentBlueAddedBars = differentBlueAddedBars + i;
                numberOfBlueBars++;
            }
            if ((colors.val[0] / (colors.val[1] + colors.val[2])) > redTolerance) {
                differentRedAddedBars = differentRedAddedBars + i;
                differentRedNumberOfBars++;
            }
            if (((colors.val[0]+colors.val[1])/colors.val[2])>=yellowTolerance){
                differentYellowAddedBars = differentYellowAddedBars + i;
                differentYellowNumberOfBars++;
            }
            MatArea1.release();
        }
        if (numberOfBlueBars > 0) {
            positionOfBlueObject = differentBlueAddedBars / numberOfBlueBars;
        }
        if (differentRedNumberOfBars > 0) {
            positionOfRedObject = (differentRedAddedBars / differentRedNumberOfBars);
        }
        if (differentYellowNumberOfBars > 0) {
            differentYellowCenterOfBars = differentYellowAddedBars / differentYellowNumberOfBars;
        }
        Point blueBar_pointBlue1A = new Point(
                positionOfBlueObject, 60);
        Point blueBar_pointBlue1B = new Point(
                positionOfBlueObject + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                blueBar_pointBlue1A,
                blueBar_pointBlue1B,
                BLUE,
                2
        );
        Point newerRedBar_pointRed1A = new Point(
                positionOfRedObject, 60);
        Point newerRedBar_pointRed1B = new Point(
                positionOfRedObject + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                newerRedBar_pointRed1A,
                newerRedBar_pointRed1B,
                RED,
                2
        );
        Point newerYellowBar_pointBlue1A = new Point(
                differentYellowCenterOfBars, 60);
        Point newerYellowBar_pointBlue1B = new Point(
                differentYellowCenterOfBars + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                newerYellowBar_pointBlue1A,
                newerYellowBar_pointBlue1B,
                YELLOW,
                2
        );
        Point middlePoint1A = new Point(
                ((widthOfInput / 2) - 51) - 4, 60);
        Point middlePoint1B = new Point(
                (widthOfInput / 2) - 51 + 4, heightOfInput);
        Imgproc.rectangle(
                input,
                middlePoint1A,
                middlePoint1B,
                GREEN,
                2
        );
        if (positionOfBlueObject != 0) {
            doubleBluePosition = positionOfBlueObject;
        }
        if (positionOfRedObject != 0) {
            doubleRedPosition = positionOfRedObject;
        }
        if (differentYellowCenterOfBars != 0) {
            doubleYellowPosition = differentYellowCenterOfBars;
        }
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public static int getRedDifferentPosition(){return positionOfRedObject;}
    public static int getBlueDifferentPosition(){return positionOfBlueObject;}

    public static int getYellowDifferentPosition(){return differentYellowCenterOfBars;}
    public static int getRedDifferentBarAmount(){return differentRedNumberOfBars;}
    public static int getBlueDifferentBarAmount(){return numberOfBlueBars;}
    public static int getYellowDifferentBarAmount(){return differentYellowNumberOfBars;}
    public static int getRedDifferentBarvalues(){return differentRedAddedBars;}
    public static int getBlueDifferentBarvalues(){return differentBlueAddedBars;}
    public static int getYellowDifferentBarvalues(){return differentYellowAddedBars;}

    public static double getRedConePosition(){return doubleRedPosition;}
    public static double getBlueConePosition(){return doubleBluePosition;}
    public static double getYellowConePosition(){return doubleYellowPosition;}

    public static double getImageWidth(){return widthOfInput;}
    public static double getBlueDistance(){
        return blueDistance;
    }
    public static double getRedDistance(){
        return redDistance;
    }
}


