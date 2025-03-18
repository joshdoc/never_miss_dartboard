#include <SDL3/SDL.h>
#include <SDL3/SDL_image.h>

int main() {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("Target Positioning", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, SDL_WINDOW_OPENGL);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL, SDL_RENDERER_ACCELERATED);
    IMG_Init(IMG_INIT_PNG);

    SDL_Surface* surface = IMG_Load("target.png");
    SDL_Texture* target = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);

    // Set the target image's position and size.
    SDL_Rect dstRect = {200, 150, 100, 100};

    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_EVENT_QUIT)
                running = false;

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, target, NULL, &dstRect);
        SDL_RenderPresent(renderer);
        SDL_Delay(16); // ~60 FPS
    }

    SDL_DestroyTexture(target);
    IMG_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
