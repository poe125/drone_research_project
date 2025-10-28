#include <SDL2/SDL.h>
#include <stdio.h>

int main() {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
        printf("Could not initialize SDL: %s\n", SDL_GetError());
        return 1;
    }

    if (SDL_NumJoysticks() < 1) {
        printf("No controllers connected!\n");
        SDL_Quit();
        return 1;
    }

    SDL_GameController *controller = SDL_GameControllerOpen(0);
    if (!controller) {
        printf("Could not open controller: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    printf("Controller connected: %s\n", SDL_GameControllerName(controller));

    SDL_Event e;
    int quit = 0;

    while (!quit) {
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = 1;
            } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
                printf("Button %d pressed\n", e.cbutton.button);
            } else if (e.type == SDL_CONTROLLERBUTTONUP) {
                printf("Button %d released\n", e.cbutton.button);
            } else if (e.type == SDL_CONTROLLERAXISMOTION) {
                printf("Axis %d moved: %d\n", e.caxis.axis, e.caxis.value);
            }
        }
        SDL_Delay(10);
    }

    SDL_GameControllerClose(controller);
    SDL_Quit();
    return 0;
}
