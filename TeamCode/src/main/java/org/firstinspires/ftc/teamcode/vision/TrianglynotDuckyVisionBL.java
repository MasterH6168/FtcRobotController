package teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
//Lyle est yucky
public class TrianglynotDuckyVisionBL {
    OpMode opMode;
    OpenCvCamera camera;
    CustomPipeline pipeline;

    private final Point LEFT_TOP_LEFT = new Point(80, 120);
    private final Point LEFT_BOTTOM_RIGHT = new Point(130, 150);
    private final Point MIDDLE_TOP_LEFT = new Point(230, 120);
    private final Point MIDDLE_BOTTOM_RIGHT = new Point(280, 150);

    private RGB middleBox = new RGB(); // Instantiated
    private RGB leftBox = new RGB();   // Instantiated
    private RGB rightBox = new RGB();
    private boolean show_value = true;
    private triangleLocation currentDetection = triangleLocation.RIGHT; // Default value


    public enum triangleLocation {
        LEFT, RIGHT, MIDDLE
    }

    public TrianglynotDuckyVisionBL(OpMode op) {
        opMode = op;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera1"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopStreaming() {
        camera.stopStreaming();
    }


    public triangleLocation getCurrentDetection() {
        return currentDetection;
    }

    public triangleLocation mostTrianglyArea() {
        int middleBoxValue = middleBox.getBlack();
        int leftBoxValue = leftBox.getBlack();

        if (show_value) {
            opMode.telemetry.addData("Middle Box Value: ", middleBoxValue);
            opMode.telemetry.addData("Left Box Value: ", leftBoxValue);
        }

        int dif = middleBoxValue - leftBoxValue;
        if (dif <= -80) {
            currentDetection = triangleLocation.MIDDLE;
            opMode.telemetry.addLine("MIDDLE");
        } else if (Math.abs(dif) > 150) {
            currentDetection = triangleLocation.LEFT;
            opMode.telemetry.addLine("LEFT");
        } else {
            currentDetection = triangleLocation.RIGHT;
            opMode.telemetry.addLine("RIGHT");
        }
        switch (currentDetection) {
            case LEFT:
                opMode.telemetry.addLine("LEFT");
                break;
            case MIDDLE:
                opMode.telemetry.addLine("MIDDLE");
                break;
            case RIGHT:
                opMode.telemetry.addLine("RIGHT");
                break;
        }
        opMode.telemetry.update();
        return currentDetection;
    }

    class CustomPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            middleBox = getAverageColor(input, MIDDLE_TOP_LEFT, MIDDLE_BOTTOM_RIGHT);
            leftBox = getAverageColor(input, LEFT_TOP_LEFT, LEFT_BOTTOM_RIGHT);

            int thickness = 3;
            Scalar color = new Scalar(0, 255, 0); // Green color for the rectangle
            triangleLocation position = mostTrianglyArea();
            if (position == triangleLocation.LEFT) {
                Imgproc.rectangle(input, LEFT_TOP_LEFT, LEFT_BOTTOM_RIGHT, color, thickness);
            } else if (position == triangleLocation.MIDDLE) {
                Imgproc.rectangle(input, MIDDLE_TOP_LEFT, MIDDLE_BOTTOM_RIGHT, color, thickness);
            }
            // No rectangle is drawn for RIGHT as it's the default case

            sendTelemetry();
            return input;
        }

        private RGB getAverageColor(Mat mat, Point topLeft, Point bottomRight) {
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;

            for (int x = (int) topLeft.x; x < bottomRight.x; x++) {
                for (int y = (int) topLeft.y; y < bottomRight.y; y++) {
                    red += mat.get(y, x)[0];
                    green += mat.get(y, x)[1];
                    blue += mat.get(y, x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;
            return new RGB(red, green, blue);
        }

        private void sendTelemetry() {
            opMode.telemetry.addLine("MIDDLE :" + " R " + middleBox.red + " G " + middleBox.green + " B " + middleBox.blue);
            opMode.telemetry.addLine("RIGHT :" + " R " + leftBox.red + " G " + leftBox.green + " B " + leftBox.blue);
            opMode.telemetry.update();
        }
    }

    public void setTelemShow(boolean show) {
        this.show_value = show;
    }
}
//Lyle est yucky
class RGBBL {
    public int red = 0;
    public int green = 0;
    public int blue = 0;

    public RGBBL() {
    }

    public RGBBL(int r, int g, int b) {
        red = r;
        green = g;
        blue = b;
    }

    public void setRed(int r) {
        red = r;
    }

    public void setGreen(int g) {
        green = g;
    }

    public void setBlue(int b) {
        blue = b;
    }

    public int getBlack() {
        return green + blue + red;
    }
}
