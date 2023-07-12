package org.firstinspires.ftc.teamcode.Threemaru.ThreemaruVision;


import com.acmerobotics.roadrunner.util.MathUtil;

import org.apache.commons.math3.transform.DftNormalization;
import org.apache.commons.math3.transform.FastFourierTransformer;
import org.apache.commons.math3.transform.TransformType;
import org.apache.commons.math3.util.MathUtils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.apache.commons.math3.util.FastMath;

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

    static int differentAddedBars = 0;
    static int differentCenterOfBars = 0;
    static int differentNumberOfBars = 0;
    static int differentRedAddedBars = 0;
    static int differentRedCenterOfBars = 0;
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

    static double yellowTolerance= 2;//Yellow tolerance needs to be roughly twice as high as red and blue tolerance

    static double doubleBluePosition;

    static double doubleRedPosition;

    static double doubleYellowPosition;


    @Override
    public Mat processFrame(Mat input) {
        widthOfInput = input.width();
        heightOfInput = input.height();

        otherCenterOfBars = 0;
        otherRedCenterOfBars = 0;
        differentAddedBars = 0;
        differentCenterOfBars = 0;
        differentNumberOfBars = 0;
        differentRedAddedBars = 0;
        differentRedCenterOfBars = 0;
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
                differentAddedBars = differentAddedBars + i;
                differentNumberOfBars++;
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
        if (differentNumberOfBars > 0) {
            differentCenterOfBars = differentAddedBars / differentNumberOfBars;
        }
        if (differentRedNumberOfBars > 0) {
            differentRedCenterOfBars = (differentRedAddedBars / differentRedNumberOfBars);
        }
        if (differentYellowNumberOfBars > 0) {
            differentYellowCenterOfBars = differentYellowAddedBars / differentYellowNumberOfBars;
        }
        Point newerBlueBar_pointBlue1A = new Point(
                differentCenterOfBars, 60);
        Point newerBlueBar_pointBlue1B = new Point(
                differentCenterOfBars + boxWidth, heightOfInput);
        Imgproc.rectangle(
                input,
                newerBlueBar_pointBlue1A,
                newerBlueBar_pointBlue1B,
                BLUE,
                2
        );
        Point newerRedBar_pointRed1A = new Point(
                differentRedCenterOfBars, 60);
        Point newerRedBar_pointRed1B = new Point(
                differentRedCenterOfBars + boxWidth, heightOfInput);
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
        if (differentCenterOfBars != 0) {
            doubleBluePosition = differentCenterOfBars;
        }
        if (differentRedCenterOfBars != 0) {
            doubleRedPosition = differentRedCenterOfBars;
        }
        if (differentYellowCenterOfBars != 0) {
            doubleYellowPosition = differentYellowCenterOfBars;
        }
        return input;
    }

    // Returns an enum being the current position where the robot will park
    public static ParkingPosition getBluePosition() {
        return bluePosition;
    }

    public static RedParkingPosition getRedPosition() {
        return redPosition;
    }

    public int getRedCentralPosition() {
        return otherRedCenterOfBars;
    }
    public int getBlueCentralPosition() {
        return otherCenterOfBars;
    }
    public static int getRedDifferentPosition(){return differentRedCenterOfBars;}
    public static int getBlueDifferentPosition(){return differentCenterOfBars;}

    public static int getYellowDifferentPosition(){return differentYellowCenterOfBars;}
    public static int getRedDifferentBarAmount(){return differentRedNumberOfBars;}
    public static int getBlueDifferentBarAmount(){return differentNumberOfBars;}
    public static int getYellowDifferentBarAmount(){return differentYellowNumberOfBars;}
    public static int getRedDifferentBarvalues(){return differentRedAddedBars;}
    public static int getBlueDifferentBarvalues(){return differentAddedBars;}
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


