#include <SDL2/SDL.h>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <vector>

// Constants
const int SCREEN_WIDTH = 1920;
const int SCREEN_HEIGHT = 1080;
const int DARTBOARD_SIZE = 380;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 640;
const float MOVE_SPEED = 0.5f;

// Global variables
cv::Mat H_distortion, H_position;
cv::Mat warped_dartboard;
cv::Point2f current_real(0.0f, 0.0f);
cv::Point current_prediction(0, 0);

// Function declarations
cv::Point real_to_projector(const cv::Point2f& real_pos);
cv::Mat draw_dartboard(const cv::Point& center_pos);
void handle_kf_prediction(float real_x_cm, float real_y_cm);

int main(int argc, char* argv[]) {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // Create window and renderer
    SDL_Window* window = SDL_CreateWindow("Dartboard Projection System",
                                        SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED,
                                        SCREEN_WIDTH, SCREEN_HEIGHT,
                                        SDL_WINDOW_FULLSCREEN);
    if (!window) {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 
                                              SDL_RENDERER_ACCELERATED | 
                                              SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        std::cerr << "Renderer creation failed: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Load and prepare dartboard image
    cv::Mat dartboard = cv::imread("target_large_bullseye.png", cv::IMREAD_UNCHANGED);
    if (dartboard.empty()) {
        std::cerr << "Failed to load dartboard image!" << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    // Resize and convert color space
    cv::resize(dartboard, dartboard, cv::Size(DARTBOARD_SIZE, DARTBOARD_SIZE));
    cv::cvtColor(dartboard, dartboard, cv::COLOR_BGRA2RGBA);

    // Distortion correction setup
    cv::Mat rect = (cv::Mat_<float>(4, 2) << 0, 0, IMAGE_WIDTH, 0, 
                   0, IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT);
    float y_shift = 15.0f;
    cv::Mat trapezoid = (cv::Mat_<float>(4, 2) << 20, y_shift, 630, 0,
                         10, IMAGE_HEIGHT - y_shift, 640, IMAGE_HEIGHT);
    H_distortion = cv::getPerspectiveTransform(rect, trapezoid);

    // Create warped dartboard
    cv::warpPerspective(dartboard, warped_dartboard, H_distortion, 
                       cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

    // Position prediction homography
    H_position = (cv::Mat_<float>(3, 3) << 1.00658869e+01, 1.17667900e+00, 9.43843481e+02,
                                           8.32884912e-02, -7.26304521e+00, 5.41552715e+02,
                                           -1.69738079e-04, 1.04877030e-03, 1.00000000e+00);

    // Main loop variables
    bool running = true;
    SDL_Event event;
    cv::Point2f target_real(0.0f, 0.0f);
    Uint32 last_time = SDL_GetTicks();

    while (running) {
        // Handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT || 
                (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)) {
                running = false;
            }
        }

        // Update position prediction
        cv::Point2f delta = target_real - current_real;
        float delta_norm = cv::norm(delta);
        if (delta_norm > MOVE_SPEED) {
            current_real += (delta / delta_norm) * MOVE_SPEED;
        }
        handle_kf_prediction(current_real.x, current_real.y);

        // Draw frame
        cv::Mat frame = draw_dartboard(current_prediction);

        // Create and update texture
        SDL_Texture* texture = SDL_CreateTexture(renderer, 
                                                SDL_PIXELFORMAT_RGBA32,
                                                SDL_TEXTUREACCESS_STREAMING,
                                                SCREEN_WIDTH, SCREEN_HEIGHT);
        SDL_UpdateTexture(texture, nullptr, frame.data, frame.step);

        // Render frame
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
        SDL_DestroyTexture(texture);

        // Frame rate control
        Uint32 current_time = SDL_GetTicks();
        Uint32 elapsed = current_time - last_time;
        if (elapsed < 14) SDL_Delay(14 - elapsed);
        last_time = current_time;
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

cv::Point real_to_projector(const cv::Point2f& real_pos) {
    cv::Mat src_point(1, 1, CV_32FC2);
    src_point.at<cv::Vec2f>(0, 0) = cv::Vec2f(real_pos.x, real_pos.y);
    cv::Mat dst_point;
    cv::perspectiveTransform(src_point, dst_point, H_position);
    return cv::Point(
        static_cast<int>(dst_point.at<cv::Vec2f>(0, 0)[0]),
        static_cast<int>(dst_point.at<cv::Vec2f>(0, 0)[1])
    );
}

cv::Mat draw_dartboard(const cv::Point& center_pos) {
    cv::Mat frame = cv::Mat::zeros(SCREEN_HEIGHT, SCREEN_WIDTH, CV_8UC4);
    int x = center_pos.x - warped_dartboard.cols / 2;
    int y = center_pos.y - warped_dartboard.rows / 2;

    if (x >= 0 && x + warped_dartboard.cols <= SCREEN_WIDTH &&
        y >= 0 && y + warped_dartboard.rows <= SCREEN_HEIGHT) {
        
        cv::Mat roi = frame(cv::Rect(x, y, warped_dartboard.cols, warped_dartboard.rows));
        std::vector<cv::Mat> dart_channels, roi_channels;
        cv::split(warped_dartboard, dart_channels);
        cv::split(roi, roi_channels);
        
        cv::Mat alpha;
        cv::extractChannel(warped_dartboard, alpha, 3);
        alpha.convertTo(alpha, CV_32F, 1.0/255.0);

        for (int c = 0; c < 3; c++) {
            cv::Mat dart_c, roi_c;
            dart_channels[c].convertTo(dart_c, CV_32F);
            roi_channels[c].convertTo(roi_c, CV_32F);
            
            cv::Mat blended = alpha.mul(dart_c) + (1.0 - alpha).mul(roi_c);
            blended.convertTo(roi_channels[c], CV_8U);
        }
        
        cv::merge(roi_channels, roi);
    }

    // Add debug text
    std::string text = "Projector: " + std::to_string(center_pos.x) + ", " + std::to_string(center_pos.y);
    cv::putText(frame, text, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
               cv::Scalar(255, 255, 255, 255), 2);

    // Set alpha channel
    cv::Mat alpha_channel(frame.rows, frame.cols, CV_8UC1, cv::Scalar(255));
    cv::insertChannel(alpha_channel, frame, 3);

    return frame;
}

void handle_kf_prediction(float real_x_cm, float real_y_cm) {
    current_prediction = real_to_projector(cv::Point2f(real_x_cm, real_y_cm));
}
