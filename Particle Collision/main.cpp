#include <graphics.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#define NUM 1000
#define QT_CAPACITY 6

float randFloat(float min, float max) {
    return min + (float)rand() / ((float)RAND_MAX / (max - min));
}

void randomVelocity(float* vx, float* vy) {
    float angle = randFloat(0, 6.283185f);
    float speed = randFloat(1.0f, 4.0f);

    *vx = cos(angle) * speed;
    *vy = sin(angle) * speed;
}

//Particle
struct Particle {
    float x, y;
    float vx, vy;
    float r;
    int hitTimer;
};

void initParticle(Particle* p, float x, float y) {
    p->x = x;
    p->y = y;
    p->r = 5;
    p->hitTimer = 0;
    randomVelocity(&p->vx, &p->vy);
}

void updateParticle(Particle* p, int w, int h) {
    p->x += p->vx;
    p->y += p->vy;

    if (p->x < p->r || p->x > w - p->r) p->vx *= -1;
    if (p->y < p->r || p->y > h - p->r) p->vy *= -1;

    if (p->hitTimer > 0) p->hitTimer--; //count down flash
}

void drawParticle(Particle* p, int mode) {
    //mode: 0 = brute, 1 = quadtree
    if (p->hitTimer > 0) {
        if (mode == 0) { // brute:flash red
            setcolor(RED);
            setfillstyle(SOLID_FILL, RED);
        } else { //quadtree: flash magenta
            setcolor(MAGENTA);
            setfillstyle(SOLID_FILL, MAGENTA);
        }
    } else {
        if (mode == 0) { //brute: normal green
            setcolor(GREEN);
            setfillstyle(SOLID_FILL, GREEN);
        } else { //quadtree: normal yellow
            setcolor(YELLOW);
            setfillstyle(SOLID_FILL, YELLOW);
        }
    }
    fillellipse((int)p->x, (int)p->y, p->r, p->r);
}

//Collision check + response (random velocity) + mark hitTimer
void collide(Particle* a, Particle* b) {
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    float dist = sqrt(dx * dx + dy * dy);

    if (dist < a->r + b->r) {

        randomVelocity(&a->vx, &a->vy);
        randomVelocity(&b->vx, &b->vy);

        a->hitTimer = 6; // frames
        b->hitTimer = 6;
    }
}

//Quadtree Axis-Aligned Bounding Box
struct AABB {
    float x, y;  // center
    float hw, hh; // half width/half height
};

int contains(AABB* b, Particle* p) {
    return (
        p->x >= b->x - b->hw &&
        p->x <= b->x + b->hw &&
        p->y >= b->y - b->hh &&
        p->y <= b->y + b->hh
    );
}

int intersects(AABB* a, AABB* b) {
    return !(
        a->x - a->hw > b->x + b->hw ||
        a->x + a->hw < b->x - b->hw ||
        a->y - a->hh > b->y + b->hh ||
        a->y + a->hh < b->y - b->hh
    );
}

//Quadtree Struct
struct Quadtree {
    AABB boundary;
    Particle* points[QT_CAPACITY];
    int count;
    int divided;
    Quadtree *ne, *nw, *se, *sw;
};

// Init Quadtree
void initQuadtree(Quadtree* qt, float x, float y, float hw, float hh) {
    qt->boundary.x = x;
    qt->boundary.y = y;
    qt->boundary.hw = hw;
    qt->boundary.hh = hh;
    qt->count = 0;
    qt->divided = 0;
    qt->ne = qt->nw = qt->se = qt->sw = NULL;
}

// Subdivide
void subdivide(Quadtree* qt) {
    float x = qt->boundary.x;
    float y = qt->boundary.y;
    float hw = qt->boundary.hw / 2;
    float hh = qt->boundary.hh / 2;

    qt->nw = (Quadtree*)malloc(sizeof(Quadtree));
    initQuadtree(qt->nw, x - hw, y - hh, hw, hh);

    qt->ne = (Quadtree*)malloc(sizeof(Quadtree));
    initQuadtree(qt->ne, x + hw, y - hh, hw, hh);

    qt->sw = (Quadtree*)malloc(sizeof(Quadtree));
    initQuadtree(qt->sw, x - hw, y + hh, hw, hh);

    qt->se = (Quadtree*)malloc(sizeof(Quadtree));
    initQuadtree(qt->se, x + hw, y + hh, hw, hh);

    qt->divided = 1;
}

// Insert Particle to Quadtree with safeguards
int insertQT(Quadtree* qt, Particle* p) {
    if (!contains(&qt->boundary, p)) return 0;

    // SAFEGUARD: stop subdividing if node is too small
    if (qt->boundary.hw <= 6 || qt->boundary.hh <= 6) {
        // force store in leaf even if full
        if (qt->count < QT_CAPACITY) {
            qt->points[qt->count++] = p;
        } else {
            // overwrite last slot (simple fallback)
            qt->points[QT_CAPACITY - 1] = p;
        }
        return 1;
    }

    if (qt->count < QT_CAPACITY) {
        qt->points[qt->count] = p;
        qt->count++;
        return 1;
    }

    if (!qt->divided) subdivide(qt);

    if (insertQT(qt->nw, p)) return 1;
    if (insertQT(qt->ne, p)) return 1;
    if (insertQT(qt->sw, p)) return 1;
    if (insertQT(qt->se, p)) return 1;

    return 0;
}

//Query Quadtree
void queryQT(Quadtree* qt, AABB* range, Particle** found, int* foundCount) {
    if (!intersects(&qt->boundary, range)) return;

    int i;
    for (i = 0; i < qt->count; i++) {
        if (contains(range, qt->points[i])) {
            found[*foundCount] = qt->points[i];
            (*foundCount)++;
            if (*foundCount >= 200) return; // safety
        }
    }

    if (qt->divided) {
        queryQT(qt->nw, range, found, foundCount);
        queryQT(qt->ne, range, found, foundCount);
        queryQT(qt->sw, range, found, foundCount);
        queryQT(qt->se, range, found, foundCount);
    }
}

//Draw Quadtree rectangles
void drawQT(Quadtree* qt) {
    rectangle(
        (int)(qt->boundary.x - qt->boundary.hw),
        (int)(qt->boundary.y - qt->boundary.hh),
        (int)(qt->boundary.x + qt->boundary.hw),
        (int)(qt->boundary.y + qt->boundary.hh)
    );

    if (qt->divided) {
        drawQT(qt->nw);
        drawQT(qt->ne);
        drawQT(qt->sw);
        drawQT(qt->se);
    }
}

//Free Quadtree memory
void freeQT(Quadtree* qt) {
    if (qt->divided) {
        freeQT(qt->nw);
        freeQT(qt->ne);
        freeQT(qt->sw);
        freeQT(qt->se);
    }
    free(qt);
}

int main() {
    srand(time(NULL));

    int w = 800, h = 600;
    initwindow(w, h, "Particles with BruteForce + Quadtree");

    Particle particles[NUM];
    int i, j;

    for (i = 0; i < NUM; i++) {
        initParticle(&particles[i], randFloat(20, w - 20), randFloat(20, h - 20));
    }

    int mode = 0;

    while (1) {
        cleardevice();

        if (kbhit()) {
            char c = getch();
            if (c == 'b' || c == 'B') mode = 0;
            if (c == 'q' || c == 'Q') mode = 1;
        }

        //Brute force
        if (mode == 0) {
            for (i = 0; i < NUM; i++) {
                for (j = i + 1; j < NUM; j++) {
                    collide(&particles[i], &particles[j]);
                }
            }
        }

        //Quadtree
        if (mode == 1) {
            Quadtree* qt = (Quadtree*)malloc(sizeof(Quadtree));
            initQuadtree(qt, w / 2.0f, h / 2.0f, w / 2.0f, h / 2.0f);

            for (i = 0; i < NUM; i++) insertQT(qt, &particles[i]);

            for (i = 0; i < NUM; i++) {
                AABB range;
                range.x = particles[i].x;
                range.y = particles[i].y;
                range.hw = 20;
                range.hh = 20;

                Particle* found[200];
                int foundCount = 0;

                queryQT(qt, &range, found, &foundCount);

                for (j = 0; j < foundCount; j++) {
                    if (found[j] != &particles[i])
                        collide(&particles[i], found[j]);
                }
            }

            setcolor(LIGHTGRAY);
            drawQT(qt);
            freeQT(qt);
        }


        for (i = 0; i < NUM; i++) {
            updateParticle(&particles[i], w, h);
            drawParticle(&particles[i], mode);
        }


        char text[64];
        if (mode == 0)
            strcpy(text, "Mode: Brute Force (press Q)");
        else
            strcpy(text, "Mode: QuadTree (press B)");
        outtextxy(10, 10, text);

        delay(10);
    }

    return 0;
}
