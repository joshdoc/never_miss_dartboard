#include <iostream>
#include <libcamera/libcamera.h>

using namespace libcamera;

void processFrame(Span<uint8_t> data, Size size)
{
    // YUV420: Y plane is first width*height bytes
    // This is already grayscale information
    // We could just use the Y channel directly, but let's create a proper grayscale image
    std::vector<uint8_t> grayscale(data.size() / 2);
    
    // Only keep the Y channel (luminance)
    for (unsigned int i = 0; i < size.height; i++) {
        for (unsigned int j = 0; j < size.width; j++) {
            grayscale[i * size.width + j] = data[i * size.width + j];
        }
    }
    
    // Here you would typically do something with the grayscale data
    // For this example, we'll just verify it's working
    std::cout << "Processed frame - First pixel value: " << (int)grayscale[0] << std::endl;
}

int main()
{
    CameraManager manager;
    manager.start();
    
    if (manager.cameras().empty()) {
        std::cerr << "No cameras found!" << std::endl;
        return 1;
    }

    std::shared_ptr<Camera> camera = manager.get(manager.cameras()[0]->id());
    camera->acquire();
    
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({StreamRole::Viewfinder});
    config->at(0).pixelFormat = formats::YUV420;
    config->at(0).size = {1280, 720};
    config->validate();
    
    if (camera->configure(config.get())) {
        std::cerr << "Camera configuration failed!" << std::endl;
        return 1;
    }

    FrameBufferAllocator allocator(camera);
    allocator.allocate(config->at(0).stream());
    
    camera->start();
    camera->requestCompleted.connect([](Request *request) {
        if (request->status() == Request::RequestCancelled)
            return;

        const StreamBuffer &buffer = request->buffers().begin()->second;
        processFrame(buffer.planes()[0].data, request->configuration().at(0).size);
        
        request->reuse();
    });

    for (unsigned int i = 0; i < allocator.buffers().size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        request->addBuffer(config->at(0).stream(), allocator.buffers()[i].get());
        camera->queueRequest(request.release());
    }

    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.ignore();
    
    camera->stop();
    camera->release();
    manager.stop();
    
    return 0;
}
