package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class WebcamOpMode extends LinearOpMode {
    public static int valBottom = -1;
    public static int valTop = -1;

    private static float rectHeight = 8f / .6f;
    private static float rectWidth = 8f / 1.5f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles top or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] bottomPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] topPos = {4f / 8f + offsetX, 2f / 8f + offsetY};
    //moves all rectangles right or top by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;

    public void startCam() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
    }

    private void stopCam() {
        webcam.closeCameraDevice();
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            RAW_IMAGE, //displays raw view
            THRESHOLD, //b&w
            detection //includes outlines
            ,//displays raw view
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixBottom = thresholdMat.get((int) (input.rows() * bottomPos[1]), (int) (input.cols() * bottomPos[0]));//gets value at circle
            valBottom = (int) pixBottom[0];

            double[] pixTop = thresholdMat.get((int) (input.rows() * topPos[1]), (int) (input.cols() * topPos[0]));//gets value at circle
            valTop = (int) pixTop[0];

            //create three points
            Point pointBottom = new Point((int) (input.cols() * bottomPos[0]), (int) (input.rows() * bottomPos[1]));
            Point pointTop = new Point((int) (input.cols() * topPos[0]), (int) (input.rows() * topPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointBottom, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointTop, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (topPos[0] - rectWidth / 2),
                            input.rows() * (topPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (topPos[0] + rectWidth / 2),
                            input.rows() * (topPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (bottomPos[0] - rectWidth / 2),
                            input.rows() * (bottomPos[1] - rectHeight / 2)),
                    new Point(
                            input.cols() * (bottomPos[0] + rectWidth / 2),
                            input.rows() * (bottomPos[1] + rectHeight / 2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }
    }

    public void runOpMode()
    {
        waitForStart();

        startCam();

	while(opModeIsActive()) {
	    return;
	}	    
    }
}
