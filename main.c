#include <SDL3/SDL.h>
#include <assert.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <SDL3_ttf/SDL_ttf.h> 

#define WIDTH 800.0
#define HEIGHT 600.0

#define CIRCLE_MAX 1500
#define SEG 64 
#define RAD 1.5

int N_old = 2, N_new = 2;
int M_old = 3, M_new = 3;

int state = 0; // 0 = old node, 1 = node -> rand, 2 = rand -> node, 3 new node, 4 rand
float s = 1.0;
int T = 120; // frames to transition

struct verlet_circle {
	float pos_curr[2];
	float pos_prev[2];
	float acceleration[2];
	float radius;
	SDL_FColor color;
};

struct verlet_circle verlet_circles[CIRCLE_MAX];
int num_circles = 0;

void update_position(struct verlet_circle * obj, float dt) {
	float velocity[2];

	for (int i = 0; i < 2; i++) {
		velocity[i] = (0.9)*(obj->pos_curr[i] - obj->pos_prev[i]);
		obj->pos_prev[i] = obj->pos_curr[i];
		obj->pos_curr[i] = obj->pos_curr[i] + velocity[i] + obj->acceleration[i] * dt * dt;

		obj->acceleration[i] = 0.0;
	}
}

void accelerate(struct verlet_circle * obj, float * acc) {
	for (int i = 0; i < 2; i++) {
		obj->acceleration[i] += acc[i]; 
	}
}

void apply_square_constraint(struct verlet_circle * obj) {
	float center[2] = { WIDTH/2.0, HEIGHT/2.0 };
	float len = HEIGHT/2.0 - 50.0;
	if (obj->pos_curr[0] - obj->radius < center[0] - len) { // to the left of the square
		obj->pos_curr[0] = center[0] - len + obj->radius;
	} else if (obj->pos_curr[0] + obj->radius > center[0] + len) { // to the right of the square
		obj->pos_curr[0] = center[0] + len - obj->radius;
	}
	if (obj->pos_curr[1] - obj->radius < center[1] - len) { // under the ]square
		obj->pos_curr[1] = center[1] - len + obj->radius;
	} else if (obj->pos_curr[1] + obj->radius > center[1] + len) { // above the square
		obj->pos_curr[1] = center[1] + len - obj->radius;
	}
}

void collide() {
	for (int i = 0; i < num_circles; i++) {
		for (int j = i + 1; j < num_circles; j++) {
			float to[2] = {
				verlet_circles[i].pos_curr[0] - verlet_circles[j].pos_curr[0],
				verlet_circles[i].pos_curr[1] - verlet_circles[j].pos_curr[1],
			};

			float d = sqrtf(to[0]*to[0] + to[1]*to[1]) + 0.0001;
			if (d < verlet_circles[i].radius + verlet_circles[j].radius) {
				float dd = verlet_circles[i].radius + verlet_circles[j].radius - d;

				to[0] /= d;
				to[1] /= d;
				verlet_circles[i].pos_curr[0] += .5 * dd * to[0];
				verlet_circles[i].pos_curr[1] += .5 * dd * to[1];
				verlet_circles[j].pos_curr[0] -= .5 * dd * to[0];
				verlet_circles[j].pos_curr[1] -= .5 * dd * to[1];
			}
		}
	}
}


float L = (WIDTH/2-100.0);
float cx = WIDTH/2;
float cy = HEIGHT/2;

float val_calc(int M, int N, float x, float y) {
	float fM = (float) M;
	float fN = (float) N;
	return sin(fM*M_PI*(x-cx)/L)*sin(fN*M_PI*(y-cy)/L) + sin(fN*M_PI*(x-cx)/L)*sin(fM*M_PI*(y-cy)/L);
}

float del_x_calc(int M, int N, float x, float y) {
	float fM = (float) M;
	float fN = (float) N;
	return (fM*M_PI/L)*cos(fM*M_PI*(x-cx)/L)*sin(fN*M_PI*(y-cy)/L) + (fN*M_PI/L)*cos(fN*M_PI*(x-cx)/L)*sin(fM*M_PI*(y-cy)/L);
}

float del_y_calc(int M, int N, float x, float y) {
	float fM = (float) M;
	float fN = (float) N;
	return sin(fM*M_PI*(x-cx)/L)*(fN*M_PI/L)*cos(fN*M_PI*(y-cy)/L) + sin(fN*M_PI*(x-cx)/L)*(fM*M_PI/L)*cos(fM*M_PI*(y-cy)/L);
}

void meep(struct verlet_circle * obj) {

	float x = obj->pos_curr[0];
	float y = obj->pos_curr[1];

	float val;
	float val_old = val_calc(M_old, N_old, x, y);
	float val_new = val_calc(M_new, N_new, x, y);
	if (state == 0) {
		val = val_old;
	} else if (state == 1) { 
		val = (1 - s)*val_old;
	} else if (state == 2) {
		val = (1 - s) * val_new;
	} else if (state == 3) {
		val = val_new;
	} else if (state == 4) {
		val = 0;
	}

	float del[2];
	float del_x_old = del_x_calc(M_old, N_old, x, y);
	float del_y_old = del_y_calc(M_old, N_old, x, y);
	float del_x_new = del_x_calc(M_new, N_new, x, y); 
	float del_y_new = del_y_calc(M_new, N_new, x, y);
	if (state == 0) {
		del[0] = del_x_old;
		del[1] = del_y_old;
	} else if (state == 1) { 
		del[0] = (1 - s)*del_x_old; //+ s*del_x_new;
		del[1] = (1 - s)*del_y_old; //+ s*del_y_new;
	} else if (state == 2) {
		del[0] = (1 - s)*del_x_new;
		del[1] = (1 - s)*del_y_new;
	} else if (state == 3) {
		del[0] = del_x_new;
		del[1] = del_y_new;
	} else if (state == 4) {
		del[0] = 1;
		del[1] = 1;
	}

	float mag = sqrt(pow(del[0], 2.0) + pow(del[1], 2.0)) + 0.00001;

	if (!isfinite(val) || !isfinite(mag)) { printf("%f %f %f\n", val, del[0], del[1]); return; }

	for (int i = 0; i < 2; i++) {
		if (fabsf(val) > 0.02) {
			obj->pos_curr[i] -= (del[i]/mag)*(val);
			obj->pos_prev[i] -= (del[i]/mag)*(val);
		} else {
			obj->pos_curr[i] = obj->pos_curr[i];
			obj->pos_prev[i] = obj->pos_prev[i];
		}
	}
}

void calculate_acceleration(struct verlet_circle * obj, float * acc) {
	float x = obj->pos_curr[0];
	float y = obj->pos_curr[1];

	float acc_x = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	float acc_y = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	float mag = sqrt(pow(acc_x,2.0) + pow(acc_y, 2.0)) + 0.00001;
	if (state == 1 || state == 2 || state == 4) {
		acc[0] = (acc_x / mag) * s;
		acc[1] = (acc_y / mag) * s;
	} else {
		acc[0] = (acc_x / mag) * 0.01;
		acc[1] = (acc_y / mag) * 0.01;
	}
}

//	del[0] = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
//	del[1] = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;

void draw_circle(SDL_FPoint center, float radius, SDL_FColor color, SDL_Renderer *renderer) {
	SDL_Vertex vertices[SEG+1];
	vertices[0] = (SDL_Vertex) { .position = center, .color = color };
	
	float angle = M_PI/(SEG/2);
	int indices[SEG*3] = {0};

	for(int i = 1; i < SEG + 1; i++) {
		vertices[i] = (SDL_Vertex) { .position = { radius*cos(i*angle) + center.x, radius*sin(i*angle) + center.y }, .color = color };
	}
	
	int j = 1;
	for (int i = 0; i < SEG*3; i=i+3) {
		indices[(i-1+SEG*3)%(SEG*3)] = j;
		indices[i+1] = j;
		j++;
	}

	SDL_RenderGeometry(renderer, NULL, vertices, SEG + 1, indices, SEG*3);
}

int main() {
	assert(SDL_Init(SDL_INIT_VIDEO));
	assert(TTF_Init());

	SDL_Window *window;
    SDL_Renderer *renderer;

	assert(SDL_CreateWindowAndRenderer("chladni pattern simulator", WIDTH, HEIGHT, 0, &window, &renderer));
	SDL_SetRenderVSync(renderer, 1);

	TTF_Font * font = TTF_OpenFont("MSGOTHIC.TTF", 20.0);
	assert(font);

	SDL_Surface * icon = SDL_LoadBMP("chladni_icon.bmp");
	assert(SDL_SetWindowIcon(window, icon));

	bool running = true;

	int frames = 0;
	int randframes = 0;

	for (; num_circles < CIRCLE_MAX; num_circles++) {
		float rand_x = rand() % (int) ((650 - ceil(RAD)) - (150 + ceil(RAD)) + 1) + (150.0 + (RAD));
		float rand_y = rand() % (int) ((550 - ceil(RAD)) - (50 + ceil(RAD)) + 1) + (50 + (RAD));
		struct verlet_circle new_verlet_circle  = {
			.pos_curr = { rand_x, rand_y },
			.pos_prev = { rand_x, rand_y },
			.acceleration = { 0.0, 0.0 },
			.radius = RAD,
			.color = (SDL_FColor) { 1, 1, 1, 1 }
		};

		verlet_circles[num_circles] = new_verlet_circle;
	}

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
		}

		if (frames % 400 == 0) {
			N_old = N_new;
			N_new++;
			s = 0.0;
			state = 1;
		} 

		if (s < 1.0 && state == 1) {
			s = fminf(1.0, s + 1.0/T); 
			if (s == 1.0) state = 4;
		} else if (state == 4) {
			if (randframes < 120) {
				randframes++;
			} else {
				state = 2;
				randframes = 0;
			}
		} else if (s <= 1.0 && state == 2) {
			s = fmaxf(0.0, s - 1.0/T);
			if (s == 0.0) { state = 3; s = 1.0; }
		}

		/*if (frames > 190 && frames < 250) {
			printf("%d %f %d %d\n", state, s, N_old, N_new);
		} */

		int num_substeps = 2;
		for (int substeps = 0; substeps < num_substeps; substeps ++) {
			for (int i = 0; i < num_circles; i++) {
				meep(&verlet_circles[i]);
			}

			for (int i = 0; i < num_circles; i++) {
				float acc[2] = { 0.0, 0.0 };
				calculate_acceleration(&verlet_circles[i], acc);
				accelerate(&verlet_circles[i], acc);
			}

			collide();

			for (int i = 0; i < num_circles; i++) {
				update_position(&verlet_circles[i], 1.0 / num_substeps);
			}

			for (int i = 0; i < num_circles; i++) {
				apply_square_constraint(&verlet_circles[i]);
			}
		}

		SDL_SetRenderDrawColor(renderer, 151, 207, 205, 255);
		SDL_RenderClear(renderer);

		SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
		SDL_FRect square = {
			.x = WIDTH/2 - 250.0 - 1,
			.y = HEIGHT/2 - 250.0 - 1,
			.w = 500.0 + 2,
			.h = 500.0 + 2,
		};
		SDL_RenderFillRect(renderer, &square);

		for (int i = 0; i < num_circles; i++) {
			SDL_FPoint center = { verlet_circles[i].pos_curr[0], verlet_circles[i].pos_curr[1] };
			SDL_FColor color =  { 1, 1, 1, 1 };
			draw_circle(center, verlet_circles[i].radius, verlet_circles[i].color, renderer);
		}
		
		char buffer1[20];
		int ret1 = snprintf(buffer1, sizeof(buffer1), "M: %d N: %d", M_new, N_new);

		SDL_Color color = { 255, 255, 255, 255 };
		SDL_Surface* surface1 = TTF_RenderText_Blended(font, buffer1, ret1, color); 
		assert(surface1);
        SDL_Texture* texture1 = SDL_CreateTextureFromSurface(renderer, surface1);
		assert(texture1);

        SDL_FRect dstRect1 = {25, 50, surface1->w, surface1->h}; // x, y, w, h
        SDL_RenderTexture(renderer, texture1, NULL, &dstRect1); 

		char buffer2[20];
		int ret2 = snprintf(buffer2, sizeof(buffer2), "Frames: %d", frames);

		SDL_Surface* surface2 = TTF_RenderText_Blended(font, buffer2, ret2, color); 
		assert(surface2);
		SDL_Texture* texture2 = SDL_CreateTextureFromSurface(renderer, surface2);
		assert(texture2);

		SDL_FRect dstRect2 = {25, 60 + surface1->h, surface2->w, surface2->h}; // x, y, w, h
		SDL_RenderTexture(renderer, texture2, NULL, &dstRect2); 

		char buffer3[20];
		int ret3 = snprintf(buffer3, sizeof(buffer3), "State: %d", state);

		SDL_Surface* surface3 = TTF_RenderText_Blended(font, buffer3, ret3, color); 
		assert(surface3);
		SDL_Texture* texture3 = SDL_CreateTextureFromSurface(renderer, surface3);
		assert(texture3);

		SDL_FRect dstRect3 = {25, 60 + surface1->h + surface2->h, surface3->w, surface3->h}; // x, y, w, h
		SDL_RenderTexture(renderer, texture3, NULL, &dstRect3); 

        SDL_RenderPresent(renderer);
		
		frames++;
	}

    TTF_Quit();

	SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}


/*
void calculate_acceleration(float x, float y, float * acc) {
	float L = 500.0;
	float val = 0.01;
//	float acc_x = (((float) (rand() % 10000)) - 5000.0 + .5) / 10000.0;
//	float acc_y = (((float) (rand() % 10000)) - 5000.0 + .5) / 10000.0;
	float acc_x = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	float acc_y = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	float mag = sqrt(pow(acc_x,2.0) + pow(acc_y, 2.0));
	acc[0] = (acc_x / mag) * fabsf(val) ;
	acc[1] = (acc_y / mag) * fabsf(val);
}
*/

/*
if (frames % 10 == 0 && num_circles < CIRCLE_MAX) {
	float rand_x = rand() % (int) ((650 - ceil(RAD)) - (150 + ceil(RAD)) + 1) + (150.0 + (RAD));
	float rand_y = rand() % (int) ((550 - ceil(RAD)) - (50 + ceil(RAD)) + 1) + (50 + (RAD));
	struct verlet_circle new_verlet_circle  = {
		.pos_curr = { rand_x, rand_y },
		.pos_prev = { rand_x, rand_y },
		.acceleration = { 0.0, 0.0 },
		.radius = RAD,
		.color = (SDL_FColor) { 1, 1, 1, 1 }
	};

	verlet_circles[num_circles++] = new_verlet_circle;
} */

/*
void apply_friction() {
	for () {
		let v = p.curr - p.prev
		v *= .5
		p.prev = p.curr - v
		
	}
}
*/

/*
void apply_circle_constraint(struct verlet_circle * obj) {
	float position[2] = { WIDTH/2.0, HEIGHT/2.0 };
	float radius = HEIGHT/2.0 - 50.0;
	float bleah[2] = { obj->pos_curr[0] - position[0], obj->pos_curr[1] - position[1] };
	float distance = sqrt(pow(bleah[0],2) + pow(bleah[1],2));
	if (distance > radius - obj->radius) {
		float n[2] = { bleah[0] / distance, bleah[1] / distance };
		obj->pos_curr[0] = position[0] + n[0] * (radius - obj->radius);
		obj->pos_curr[1] = position[1] + n[1] * (radius - obj->radius);
	}
}
*/