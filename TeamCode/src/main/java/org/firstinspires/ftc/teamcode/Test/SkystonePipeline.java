package org.firstinspires.ftc.teamcode.Test;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
// Credit to WolfCorpFTC team # 12525 for the original file.
// 16244 modified for webcam and ultimately for Fright Frenzy Team Shipping Element
// see youtube video titles FTC EasyOpenCV Tutorial + Skystone Example

class SkystonePipeline extends OpenCvPipeline{
    Telemetry telemetry;
    Mat mat = new Mat(); // Mat is a matrix

    // These enums can go in the ENUM package if desired
    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    // ROI is the region of interest. This is a group of pixels that will be analyzed
    // these values assume a 320 x 240 camera resolution if 640 x 480 is used the values will need
    // to be adjusted accordingly
    // camera resolution is specified in the opmode that calls this class

    // upper left of Left_ROI rectangle is 60, 35
    // Pixel 0,0 is at the uppar left of the display.
    // x increases to the right (as expected)
    // y increases down (which is not intuitive)
    // lower right of the left_ROI rectangle is 120, 75
    // adjust these to fit the placement of objects to detect on the field.
    // In Skystone only two of the 3 stones were visable by the camera.
    // By the process of elimination, the skystone (black sticker on yellow block)
    // was found is both objects in the view were yellow.
    // This logic will change from game to game
    static final Rect LEFT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect RIGHT_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    // telemetry is part of a LinearOpmde. Since this pipeline does not extend a LinearOpmode
    // telemetry has to be added in the constructor for SkystonePipeline to be able to use it.

    // Constructor
    public SkystonePipeline(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // converting RGB to HSV color scheme
        // Normal HSV has HUE values from 0 to 360. Open CV uses 0 to 180,
        // Normal HSV has Saturation and Value from 0 to 100. Open CV scales that 100 from 0 to 255
        // HUE's for reference
        // Red 0
        // Orange   25 deg or 12.5 in Open VC
        // Yellow   60 or 30 in Open CV
        // Green    120 or 60 in open CV
        // Cyan     180 or 90 in Open CV
        // Blue     240 or 120 in Open CV
        // Purple   280 or 140 in Open CV
        // Pink     300 or 150 in open CV
        // Red      360 or 180 in openCV
        Scalar lowHSV = new Scalar(23, 50, 70); // for yellow this is the "orange" side of the range
        Scalar highHSV = new Scalar(32, 255, 255); // for yellow this is the "green" side of the range

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI); //sub matrices of mat
        Mat right = mat.submat(RIGHT_ROI);

        // if a pixel is deemed to be between the low and high HSV range OpenCV makes it white
        // white is given a value of 255. This way the new image is just grayscale where 0 is black
        // and 255 is white. Below all elements of the sub matrix are added up and divided by 255.
        // This essentially calculates the ratio of identified pixels to those not identified. The
        //higher the value the more detection.
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release(); // frees up memory
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD; // sets a limit to compare to so small objects don't accidentally trigger
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft && stoneRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Skystone Location", "not found");
        }
        else if (stoneLeft) {
            location = Location.RIGHT;
            telemetry.addData("Skystone Location", "right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Skystone Location", "left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0); // color scheme for targeting boxes drawn on the display
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone); // the target boxes surround the ROI's
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }
    // getter function that the main autonomous class can call. Besides telemetry, we just return stone postion because that is all the
    // auto opmode really needs.
    public Location getLocation() {
        return location;
    }

}
