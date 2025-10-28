#include <SDL2/SDL.h>
#include <stdio.h>

int main(){
	if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) < 0){
		printf("SDL could not initialize: %s\n", SDL_GetError());
		return 1;
	}
	
	SDL_GameController *controller = NULL;
	for(int i=0; i<SDL_NumJoysticks(); i++){
		if(SDL_IsGameController(i)){
			controller = SDL_GameControllerOpen(i);
			if(controller) break;
		}
	}
	if(!controller){
		printf("No game controller found\n");
		return 1;
	}
	
	SDL_Event e;
	int running = 1;
	while(running){
		while(SDL_PollEvent(&e)){
			if(e.type == SDL_QUIT){
				running = 0;
			}
			if(e.type == SDL_CONTROLLERBUTTONDOWN){
				if(e.cbutton.button == SDL_CONTROLLER_BUTTON_A){
					printf("A button pressed\n");
				}
			}
			else if(e.type == SDL_CONTROLLERBUTTONUP){
				if(e.cbutton.button == SDL_CONTROLLER_BUTTON_A){
					printf("A button released\n");
				}
			} else if (e.type == SDL_CONTROLLERAXISMOTION){
				if(e.caxis.axis == SDL_CONTROLLER_AXIS_LEFTY){
					printf("Left stick Y axis: %d\n", e.caxis.value);
				}
				if(e.caxis.axis == SDL_CONTROLLER_AXIS_RIGHTY){
					printf("Right stick Y axis: %d\n", e.caxis.value);
				}
			}
		}
	}
	SDL_GameControllerClose(controller);
	SDL_Quit();
	return 0;
}
