#include <iostream>
#include <fstream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Pass the argument `uvc` to run in UVC mode

static int clamp(int num, int v0, int v1) {
    return std::max(v0, std::min(num, v1));
}

int main(int argc, char** argv) {
    bool rotate = 1;
    bool downscale = 0;


    // Create pipeline
    dai::Pipeline pipeline;

    std::cout<<"OpenVINOVersion: " << dai::Pipeline().getOpenVINOVersion() << std::endl;

    auto json = pipeline.serializeToJson();

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    if (rotate) camRgb->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    if (downscale) {
        camRgb->setIspScale(1, 2);
    }
    camRgb->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_50_HZ);
    camRgb->setFps(30);
    camRgb->setInterleaved(false);
    camRgb->setPreviewSize(1920, 1080);
//    camRgb->setFp16(true);


    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control");
    controlIn->out.link(camRgb->inputControl);

    auto imageManipConfig = pipeline.create<dai::node::XLinkIn>();
    imageManipConfig->setMaxDataSize(256);
    imageManipConfig->setStreamName("bobol");

    // Linking
    if (downscale) {
        camRgb->video.link(xoutVideo->input);
    } else {

        auto imageManip = pipeline.create<dai::node::ImageManip>();
        imageManip->setMaxOutputFrameSize(3840 * 2140 * 3);
        imageManip->inputImage.setBlocking(false);
        imageManip->inputImage.setQueueSize(1);

        imageManip->inputConfig.setWaitForMessage(false);
        imageManip->initialConfig.setResize(1920, 1080);
        imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);

        imageManipConfig->out.link(imageManip->inputConfig);

        camRgb->isp.link(imageManip->inputImage);
//        warp->out.link(imageManip->inputImage);
        imageManip->out.link(xoutVideo->input);

        auto nnManip = pipeline.create<dai::node::ImageManip>();
        nnManip->initialConfig.setKeepAspectRatio(false);
        nnManip->initialConfig.setResize(256, 256);
        nnManip->setMaxOutputFrameSize(256*256*3);
        nnManip->inputImage.setBlocking(false);
        nnManip->inputImage.setQueueSize(4);


        auto faceDetectionNN = pipeline.create<dai::node::NeuralNetwork>();
        faceDetectionNN->setBlobPath("/Users/karenghandilyan/Downloads/face_detection_back_21_4.blob");
        faceDetectionNN->setNumPoolFrames(4);
//        faceDetectionNN->setConfidenceThreshold(0.5);
        faceDetectionNN->input.setBlocking(false);
        faceDetectionNN->input.setQueueSize(1);
        faceDetectionNN->setNumInferenceThreads(2);
//        faceDetectionNN->setNumNCEPerInferenceThread(1);

        auto nnOut = pipeline.create<dai::node::XLinkOut>();
        nnOut->setStreamName("face_out");
        nnOut->setFpsLimit(30.0);
        nnOut->input.setBlocking(false);
        nnOut->input.setQueueSize(1);

        camRgb->preview.link(nnManip->inputImage);
        nnManip->out.link(faceDetectionNN->input);
        faceDetectionNN->out.link(nnOut->input);
    }

    // Connect to device and start pipeline
    auto config = dai::Device::Config();
    config.version = dai::Pipeline().getOpenVINOVersion();
//    config.board.uvcEnable = enableUVC;
    printf("=== Creating device with board config...\n");
    dai::Device device(config);
    printf("=== Device created, connected cameras:\n");
    for (auto s : device.getCameraSensorNames()) {
        std::cout << "  > " << s.first << " : " << s.second << "\n";
    }
    printf("Starting pipeline...\n");
    device.startPipeline(pipeline);
    printf("=== Started!\n");


    int qsize = 8;
    bool blocking = false;
    auto face_out = device.getOutputQueue("face_out", 1, false);
    auto video = device.getOutputQueue("video", qsize, blocking);
    auto controlQueue = device.getInputQueue("control");
    auto bobolQueue = device.getInputQueue("bobol");

    int lensPos = 150;

    using namespace std::chrono;
    auto tprev = steady_clock::now();
    int count = 0;
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::resizeWindow("video", 1920, 1080);

    auto newFrame = [](std::shared_ptr<dai::ADatatype> callback) {
        if(dynamic_cast<dai::NNData *>(callback.get()) != nullptr) {
//            auto cast = std::dynamic_pointer_cast<dai::ImgDetections>(callback);

            auto data = callback.get();
            auto cast_frame = std::dynamic_pointer_cast<dai::NNData>(callback);
            auto res = cast_frame->getFirstLayerFp16();
            auto layers = cast_frame->getAllLayerNames();
            for (auto layer: layers) {
                auto test = cast_frame->getLayerFp16(layer);
                std::cout << layer <<": " << test.size() << std::endl;
            }
//            if (cast->detections.size() > 0) {
//                auto res = cast->detections[0];
//                std::cout<<"confidence: " << res.confidence << "x: "<<res.xmax - res.xmin << " y: " << res.ymax - res.ymin<<std::endl;
//            }

        }
    };
    face_out->addCallback(newFrame);
    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();

        if (1) { // FPS calc
            auto tnow = steady_clock::now();
            count++;
            auto tdiff = duration<double>(tnow - tprev).count();
            if (tdiff >= 1) {
                double fps = count / tdiff;
                printf("FPS: %.3f\n", fps);
                count = 0;
                tprev = tnow;
            }
        }


        std::string title = std::to_string(videoIn->getWidth()) + "x"+ std::to_string(videoIn->getHeight());
        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead

        cv::imshow("video", videoIn->getCvFrame());
        cv::setWindowTitle("video", title);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }  else if(key == 'i' || key == 'o' || key == 'k' || key == 'l' || key == ',' || key == '.') {
            if(key == 'i') lensPos -= 100;
            if(key == 'o') lensPos += 100;
            if(key == 'k') lensPos -= 20;
            if(key == 'l') lensPos += 20;
            if(key == ',') lensPos -= 3;
            if(key == '.') lensPos += 3;
            lensPos = clamp(lensPos, 0, 255);
            printf("Setting manual focus, lens position: %d\n", lensPos);
            dai::CameraControl ctrl;
            ctrl.setManualFocus(lensPos);
            controlQueue->send(ctrl);
        } else if(key == 'w') {
            dai::ImageManipConfig ctrl;
            ctrl.setResize(640, 360);
            ctrl.setKeepAspectRatio(true);
            bobolQueue->send(ctrl);
        } else if(key == 'e') {
            dai::ImageManipConfig ctrl;
            ctrl.setResize(1280, 720);
            ctrl.setKeepAspectRatio(true);
            bobolQueue->send(ctrl);
        } else if(key == 'r') {
            dai::ImageManipConfig ctrl;
            ctrl.setResize(1920, 1080);
            ctrl.setKeepAspectRatio(true);
            bobolQueue->send(ctrl);
        } else if(key == 't') {
            dai::ImageManipConfig ctrl;
            ctrl.setResize(3840, 2160);
            ctrl.setKeepAspectRatio(true);
            bobolQueue->send(ctrl);
        }
    }
    return 0;
}
