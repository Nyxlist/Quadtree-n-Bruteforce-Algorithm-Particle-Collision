k#include <graphics.h>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct Particle {
    float x, y;
    float vx, vy;
    float r;
};

float randFloat(float a, float b) {
    return a + static_cast<float>(rand()) / RAND_MAX * (b - a);
}

void randomVelocity(Particle &p) {
    p.vx = randFloat(-3, 3);
    p.vy = randFloat(-3, 3);
}

void initParticle(Particle &p, int W, int H) {
    p.x = randFloat(50, W - 50);
    p.y = randFloat(50, H - 50);
    p.r = 8;
    randomVelocity(p);
}

void updateParticle(Particle &p, int W, int H) {
    p.x += p.vx;
    p.y += p.vy;

    if (p.x - p.r < 0 || p.x + p.r > W) p.vx *= -1;
    if (p.y - p.r < 0 || p.y + p.r > H) p.vy *= -1;
}

bool isColliding(Particle &a, Particle &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float distSq = dx*dx + dy*dy;
    float rad = a.r + b.r;
    return distSq <= rad * rad;
}

void handleCollision(Particle &a, Particle &b) {
    randomVelocity(a);
    randomVelocity(b);
}

int main() {
    srand(time(NULL));

    int W = 800, H = 600;
    initwindow(W, H, "Brute Force Collision");

    const int N = 80;
    Particle particles[N];

    for (int i = 0; i < N; i++) {
        initParticle(particles[i], W, H);
    }

    while (1) {
        cleardevice();


        for (int i = 0; i < N; i++)
            updateParticle(particles[i], W, H);

        //Brute force collision O(n^2)
        for (int i = 0; i < N; i++) {
            for (int j = i + 1; j < N; j++) {
                if (isColliding(particles[i], particles[j])) {
                    handleCollision(particles[i], particles[j]);
                }
            }
        }


        setcolor(WHITE);
        for (int i = 0; i < N; i++) {
            fillellipse(particles[i].x, particles[i].y, particles[i].r, particles[i].r);
        }

        delay(10);
    }

    return 0;
}
