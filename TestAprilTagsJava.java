import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;

import java.util.List;

@TeleOp(name = "SimpleAprilTag", group = "Examples")
public class SimpleAprilTagOpMode extends OpMode {

    private OpenCvCamera camera;
    private AprilTagDetectionPipeline tagPipeline;

    @Override
    public void init() {
        // Set up the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up the tag detector
        tagPipeline = new AprilTagDetectionPipeline(0.166, "tag36h11");
        camera.setPipeline(tagPipeline);

        // Start the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera failed to start");
            }
        });
    }

    @Override
    public void loop() {
        // Get the detected tags
        List<AprilTagDetection> tags = tagPipeline.getLatestDetections();

        // Show info on the screen
        telemetry.addData("Number of Tags", tags.size());
        for (AprilTagDetection tag : tags) {
            telemetry.addData("Tag ID", tag.id);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the camera
        if (camera != null) {
            camera.closeCameraDevice();
        }
    }
}
