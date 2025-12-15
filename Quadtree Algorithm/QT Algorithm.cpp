#include <graphics.h>
#include <cmath>
#include <vector>
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

// AABB struct
struct AABB {
    float x, y, w, h;

    bool contains(Particle *p) const {
        return (p->x >= x && p->x <= x + w &&
                p->y >= y && p->y <= y + h);
    }

    bool intersects(const AABB &other) const {
        return !(x + w < other.x ||
                 x > other.x + other.w ||
                 y + h < other.y ||
                 y > other.y + other.h);
    }
};

// Quadtree
struct Quadtree {
    static const int CAPACITY = 8;
    AABB boundary;
    std::vector<Particle*> points;
    bool divided;

    Quadtree *NE, *NW, *SE, *SW;

    Quadtree(float x, float y, float w, float h)
        : boundary{x, y, w, h}, divided(false),
          NE(NULL), NW(NULL), SE(NULL), SW(NULL) {}

    void subdivide() {
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.w / 2;
        float h = boundary.h / 2;

        NE = new Quadtree(x + w, y,     w, h);
        NW = new Quadtree(x,     y,     w, h);
        SE = new Quadtree(x + w, y + h, w, h);
        SW = new Quadtree(x,     y + h, w, h);

        divided = true;
    }

    bool insert(Particle *p) {
    if (!boundary.contains(p))
        return false;

    if (points.size() < CAPACITY) {
        points.push_back(p);
        return true;
    }

    // Prevent infinite subdivision
    if (boundary.w <= 16 || boundary.h <= 16) {
        points.push_back(p);
        return true;
    }

    if (!divided) subdivide();

    return (NE->insert(p) || NW->insert(p) ||
            SE->insert(p) || SW->insert(p));
}
    void query(const AABB &range, std::vector<Particle*> &found) {
        if (!boundary.intersects(range)) return;

        for (size_t i = 0; i < points.size(); i++) {
            if (range.contains(points[i])) {
                found.push_back(points[i]);
            }
        }

        if (divided) {
            NE->query(range, found);
            NW->query(range, found);
            SE->query(range, found);
            SW->query(range, found);
        }
    }
};

bool isColliding(Particle &a, Particle &b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float distSq = dx*dx + dy*dy;
    float rad = a.r + b.r;
    return distSq <= rad * rad;
}

int main() {
        int W = 800, H = 600;
    initwindow(W, H, "Quadtree Collision");

    srand(time(NULL));

    const int N = 80;
    Particle particles[N];

    for (int i = 0; i < N; i++)
        initParticle(particles[i], W, H);

    while (1) {
        cleardevice();

        Quadtree qt(0, 0, W, H);

        for (int i = 0; i < N; i++)
            qt.insert(&particles[i]);

        // Update
        for (int i = 0; i < N; i++) {
            particles[i].x += particles[i].vx;
            particles[i].y += particles[i].vy;

            if (particles[i].x < 0 || particles[i].x > W) particles[i].vx *= -1;
            if (particles[i].y < 0 || particles[i].y > H) particles[i].vy *= -1;
        }

        // Quadtree collision
        for (int i = 0; i < N; i++) {
            AABB range{particles[i].x - 20, particles[i].y - 20, 40, 40};
            std::vector<Particle*> found;
            qt.query(range, found);

            for (size_t j = 0; j < found.size(); j++) {
                if (&particles[i] != found[j] && isColliding(particles[i], *found[j])) {
                    randomVelocity(particles[i]);
                    randomVelocity(*found[j]);
                }
            }
        }

        // Draw
        setcolor(WHITE);
        for (int i = 0; i < N; i++) {
            fillellipse(particles[i].x, particles[i].y, particles[i].r, particles[i].r);
        }

        delay(10);
    }

    return 0;
}
