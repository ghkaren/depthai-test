#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/UVC.hpp"
#include <depthai/pipeline/node/UAC.hpp>

std::shared_ptr<dai::Device> _device;
std::shared_ptr<dai::DataOutputQueue> _videoQueue;
int videoCallbackId = -1;
std::shared_ptr<dai::DataInputQueue> _controlQueue;

std::queue<cv::Mat> _previewQueue;
std::mutex _queueMtx;

bool _isStreaming = false;

dai::Pipeline getMainPipeline(bool enableUVC, bool enableUAC) {
    // Create pipeline
    dai::Pipeline pipeline;
    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setImageOrientation(dai::CameraImageOrientation::ROTATE_180_DEG);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_50_HZ);
    camRgb->setFps(30);
    camRgb->setInterleaved(false);

    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    xoutVideo->setStreamName("video");
    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control");
    controlIn->out.link(camRgb->inputControl);

    auto imageManipConfig = pipeline.create<dai::node::XLinkIn>();
    imageManipConfig->setMaxDataSize(256);
    imageManipConfig->setStreamName("manipConfigQueue");

    if (enableUAC) {
        auto uac = pipeline.create<dai::node::UAC>();
        auto mic = pipeline.create<dai::node::AudioMic>();
        mic->setStreamBackMic(false);
        mic->out.link(uac->input);
        uac->initialConfig.setMicGainDecibels(28);
    }

    if (enableUVC) {
        auto uvc = pipeline.create<dai::node::UVC>();
        uvc->setGpiosOnInit({{58,0}, {37,0}, {34,0}});
        uvc->setGpiosOnStreamOn({{58,1}, {37,1}, {34,1}});
        uvc->setGpiosOnStreamOff({{58,0}, {37,0}, {34,0}});
        camRgb->video.link(uvc->input);
    } else {
        // Linking
        auto imageManip = pipeline.create<dai::node::ImageManip>();
        imageManip->setMaxOutputFrameSize(3840 * 2140 * 3);
        imageManip->inputImage.setBlocking(false);
        imageManip->inputImage.setQueueSize(1);

        imageManip->inputConfig.setWaitForMessage(false);
        imageManip->initialConfig.setResize(1920, 1080);
        imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::NV12);

        imageManipConfig->out.link(imageManip->inputConfig);
        camRgb->isp.link(imageManip->inputImage);
        imageManip->out.link(xoutVideo->input);
    }
    return pipeline;
}

void addVideoQueueCallback() {
    if(_videoQueue != nullptr && videoCallbackId == -1) {
        auto videoCallback = [](std::shared_ptr<dai::ADatatype> data) {
            if (auto videoFrame = std::dynamic_pointer_cast<dai::ImgFrame>(data)) {
//                printf("new frame: %d x %d\n", videoFrame->getWidth(), videoFrame->getHeight());
                std::unique_lock<std::mutex> lock(_queueMtx);
                _previewQueue.push(videoFrame->getCvFrame());
            }
        };
        videoCallbackId = _videoQueue->addCallback(videoCallback);
        printf("Callback added\n");
    }
}

void removeVideoQueueCallback() {
    if(_videoQueue != nullptr && videoCallbackId != -1) {
        _videoQueue->removeCallback(videoCallbackId);
        videoCallbackId = -1;
        printf("Callback removed\n");
    }
}

int main(int argc, char** argv) {
    bool enableUVC = false;
    bool enableUAC = true;
    auto pipeline = getMainPipeline(enableUVC, enableUAC);
    // Connect to device and start pipeline
    auto config = dai::Device::Config();
    config.board.uvcEnable = enableUVC;
    printf("=== Creating device with board config...\n");
    _device = std::make_shared<dai::Device>(config);
    printf("=== Device created, connected cameras:\n");
    for (const auto& s : _device->getCameraSensorNames()) {
        std::cout << "  > " << s.first << " : " << s.second << "\n";
    }
    printf("Starting pipeline...\n");
    _device->startPipeline(pipeline);
    printf("=== Started!\n");
    _isStreaming = true;
    _videoQueue = _device->getOutputQueue("video", 3, false);
    _controlQueue = _device->getInputQueue("control", 1, false);

    addVideoQueueCallback();
    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::resizeWindow("video", 1280, 720);

    while(true) {
        cv::Mat previewFrame;
        {
        std::unique_lock<std::mutex> lock(_queueMtx);
            if (!_previewQueue.empty()) {
                previewFrame = _previewQueue.front();
                _previewQueue.pop();
            }
        }
        if (!previewFrame.empty()) {
            cv::imshow("video", previewFrame);
        }

        int key = cv::waitKey(1);
        if(key == 'q') {
            if (_device->isPipelineRunning()) {
                _device->close();
            }
            break;
        } else if(key == 's') {
            dai::CameraControl ctrl;
            if (_isStreaming) {
                ctrl.setStopStreaming();
                removeVideoQueueCallback();
                printf("Stopped video streaming\n");
            } else {
                ctrl.setStartStreaming();
                ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
                addVideoQueueCallback();
                printf("Started video streaming\n");
            }
            _isStreaming = !_isStreaming;
            _controlQueue->send(ctrl);
        }
    }
    return 0;
}
