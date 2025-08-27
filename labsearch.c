// Name: Konstantinos Vardakas 
// AM: 522

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define MAX_N 100
#define MAX_QUEUE 10000

// Node structure for search algorithms
typedef struct {
    int x, y;          // Coordinates
    float g;           // Cost from start
    float f;           // f = g + h for A* and f = g for UCS 
    int parent_x, parent_y; // Parent coordinates for path reconstruction
} Node;

// Priority queue structure
typedef struct {
    Node nodes[MAX_QUEUE];
    int size;
} PriorityQueue;

// Maze and visited arrays
int maze[MAX_N][MAX_N];
int visited[MAX_N][MAX_N];
int N;
float p;
int expansions;

// Directions for 8 possible moves (horizontal, vertical, diagonal)
int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

// Initialize priority queue
void init_queue(PriorityQueue *pq) {
    pq->size = 0;
}

// Push node to priority queue (min-heap based on f-value)
void push(PriorityQueue *pq, Node node) {
    if (pq->size >= MAX_QUEUE) return;
    pq->nodes[pq->size] = node;
    int i = pq->size++;
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (pq->nodes[parent].f <= pq->nodes[i].f) break;
        Node temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[parent];
        pq->nodes[parent] = temp;
        i = parent;
    }
}

// Pop node with minimum f-value
Node pop(PriorityQueue *pq) {
    Node result = pq->nodes[0];
    pq->nodes[0] = pq->nodes[--pq->size];
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        if (left < pq->size && pq->nodes[left].f < pq->nodes[smallest].f)
            smallest = left;
        if (right < pq->size && pq->nodes[right].f < pq->nodes[smallest].f)
            smallest = right;
        if (smallest == i) break;
        Node temp = pq->nodes[i];
        pq->nodes[i] = pq->nodes[smallest];
        pq->nodes[smallest] = temp;
        i = smallest;
    }
    return result;
}

// Check if queue is empty
int is_empty(PriorityQueue *pq) {
    return pq->size == 0;
}

// Generate maze
void generate_maze(int sx, int sy, int gx, int gy) {
    srand(time(NULL));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            maze[i][j] = (i == sx && j == sy) || (i == gx && j == gy) ? 1 : ((float)rand() / RAND_MAX < p) ? 1 : 0;
    maze[N-1][0] = 1; // Cell A (bottom-left)
    maze[0][N-1] = 1; // Cell B (top-right)
}

void print_maze(int path[MAX_N][MAX_N], int sx, int sy, int gx, int gy) {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            printf("----");
        }
        printf("-\n");
        for (int j = 0; j < N; j++) {
            printf("|");
            if (i == sx && j == sy) {
                printf(" S ");
            } else if (i == gx && j == gy) {
                printf(" G ");
            } else if (path[i][j]) {
                printf(" * ");
            } else if (maze[i][j]) {
                printf("   ");
            } else {
                printf(" X ");
            }
        }
        printf("|\n");
    }
    for (int j = 0; j < N; j++) {
        printf("----");
    }
    printf("-\n");
}

float chebyshev(int x1, int y1, int x2, int y2) {
    return fmax(abs(x1 - x2), abs(y1 - y2));
}

float heuristic(int x, int y, int gx, int gy) {
    float direct = chebyshev(x, y, gx, gy); // Direct to G
    float via_A = chebyshev(x, y, N-1, 0) + 2 + chebyshev(0, N-1, gx, gy); // Via A -> B -> G
    float via_B = chebyshev(x, y, 0, N-1) + 2 + chebyshev(N-1, 0, gx, gy); // Via B -> A -> G
    return fmin(direct, fmin(via_A, via_B));
}

// Reconstruct and print path
float reconstruct_path(int parent[MAX_N][MAX_N], int gx, int gy, int sx, int sy) {
    int path[MAX_N][MAX_N] = {0};
    float cost = 0;
    int x = gx, y = gy;
    while (x != sx || y != sy) {
        path[x][y] = 1;
        int px = parent[x][y] / MAX_N;
        int py = parent[x][y] % MAX_N;
        if ((x == 0 && y == N-1 && px == N-1 && py == 0) ||
            (x == N-1 && y == 0 && px == 0 && py == N-1))
            cost += 2;
        else
            cost += 1;
        x = px;
        y = py;
    }
    path[sx][sy] = 0; // Start cell is marked as 'S'
    print_maze(path, sx, sy, gx, gy);
    return cost;
}

// Uniform Cost Search
float ucs(int sx, int sy, int gx, int gy) {
    PriorityQueue pq;
    init_queue(&pq);
    float cost[MAX_N][MAX_N];
    int parent[MAX_N][MAX_N];
    expansions = 0;

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            cost[i][j] = 1e9;
            visited[i][j] = 0;
        }

    Node start = {sx, sy, 0, 0, -1, -1};
    cost[sx][sy] = 0;
    push(&pq, start);

    while (!is_empty(&pq)) {
        Node current = pop(&pq);
        int x = current.x, y = current.y;
        if (visited[x][y]) continue;
        visited[x][y] = 1;
        expansions++;

        if (x == gx && y == gy) {
            parent[x][y] = current.parent_x * MAX_N + current.parent_y;
            return reconstruct_path(parent, gx, gy, sx, sy);
        }

        // Regular moves
        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx >= 0 && nx < N && ny >= 0 && ny < N && maze[nx][ny] && !visited[nx][ny]) {
                float new_cost = current.g + 1;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    Node next = {nx, ny, new_cost, new_cost, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        }

        // Special move (A to B or B to A)
        if (x == N-1 && y == 0) { // At A (bottom-left)
            int nx = 0, ny = N-1; // To B (top-right)
            if (!visited[nx][ny]) {
                float new_cost = current.g + 2;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    Node next = {nx, ny, new_cost, new_cost, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        } else if (x == 0 && y == N-1) { // At B (top-right)
            int nx = N-1, ny = 0; // To A (bottom-left)
            if (!visited[nx][ny]) {
                float new_cost = current.g + 2;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    Node next = {nx, ny, new_cost, new_cost, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        }
    }
    return -1; // No path found
}

// A* Search
float a_star(int sx, int sy, int gx, int gy) {
    PriorityQueue pq;
    init_queue(&pq);
    float cost[MAX_N][MAX_N];
    int parent[MAX_N][MAX_N];
    expansions = 0;

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            cost[i][j] = 1e9;
            visited[i][j] = 0;
        }

    Node start = {sx, sy, 0, heuristic(sx, sy, gx, gy), -1, -1};
    cost[sx][sy] = 0;
    push(&pq, start);

    while (!is_empty(&pq)) {
        Node current = pop(&pq);
        int x = current.x, y = current.y;
        if (visited[x][y]) continue;
        visited[x][y] = 1;
        expansions++;

        if (x == gx && y == gy) {
            parent[x][y] = current.parent_x * MAX_N + current.parent_y;
            return reconstruct_path(parent, gx, gy, sx, sy);
        }

        // Regular moves
        for (int i = 0; i < 8; i++) {
            int nx = x + dx[i], ny = y + dy[i];
            if (nx >= 0 && nx < N && ny >= 0 && ny < N && maze[nx][ny] && !visited[nx][ny]) {
                float new_cost = current.g + 1;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    float h = heuristic(nx, ny, gx, gy);
                    Node next = {nx, ny, new_cost, new_cost + h, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        }

        // Special move (A to B or B to A)
        if (x == N-1 && y == 0) { // At A (bottom-left)
            int nx = 0, ny = N-1; // To B (top-right)
            if (!visited[nx][ny]) {
                float new_cost = current.g + 2;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    float h = heuristic(nx, ny, gx, gy);
                    Node next = {nx, ny, new_cost, new_cost + h, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        } else if (x == 0 && y == N-1) { // At B (top-right)
            int nx = N-1, ny = 0; // To A (bottom-left)
            if (!visited[nx][ny]) {
                float new_cost = current.g + 2;
                if (new_cost < cost[nx][ny]) {
                    cost[nx][ny] = new_cost;
                    float h = heuristic(nx, ny, gx, gy);
                    Node next = {nx, ny, new_cost, new_cost + h, x, y};
                    push(&pq, next);
                    parent[nx][ny] = x * MAX_N + y;
                }
            }
        }
    }
    return -1; // No path found
}

int main() {
    do {
        printf("Enter N (grid size): ");
        scanf("%d", &N);
        if (N <= 0 || N > MAX_N) {
            printf("Invalid N. Must be between 1 and %d.\n", MAX_N);
        }
    } while (N <= 0 || N > MAX_N);

    do {
        printf("Enter p (probability of free cell): ");
        scanf("%f", &p);
        if (p < 0 || p > 1) {
            printf("Invalid p. Must be between 0 and 1.\n");
        }
    } while (p < 0 || p > 1);

    int sx, sy, gx, gy;
    do {
        printf("Enter start cell coordinates (sx sy): ");
        scanf("%d %d", &sx, &sy);
        if (sx < 0 || sx >= N || sy < 0 || sy >= N) {
            printf("Invalid coordinates. Must be between 0 and %d.\n", N-1);
        }
    } while (sx < 0 || sx >= N || sy < 0 || sy >= N);

    do {
        printf("Enter goal cell coordinates (gx gy): ");
        scanf("%d %d", &gx, &gy);
        if (gx < 0 || gx >= N || gy < 0 || gy >= N) {
            printf("Invalid coordinates. Must be between 0 and %d.\n", N-1);
        } else if (gx == sx && gy == sy) {
            printf("Goal cell cannot be the same as start cell.\n");
        }
    } while (gx < 0 || gx >= N || gy < 0 || gy >= N || (gx == sx && gy == sy));

    generate_maze(sx, sy, gx, gy);

    printf("\nInitial Maze:\n");
    int empty_path[MAX_N][MAX_N] = {0};
    print_maze(empty_path, sx, sy, gx, gy);

    printf("\nUCS Results:\n");
    float ucs_cost = ucs(sx, sy, gx, gy);
    if (ucs_cost >= 0)
        printf("Path cost: %.1f\nExpansions: %d\n", ucs_cost, expansions);
    else
        printf("No path found\n");

    printf("\nA* Results:\n");
    float a_star_cost = a_star(sx, sy, gx, gy);
    if (a_star_cost >= 0)
        printf("Path cost: %.1f\nExpansions: %d\n", a_star_cost, expansions);
    else
        printf("No path found\n");

    printf("Press Enter to continue...");
    getchar(); // consumes leftover '\n' from previous input
    getchar(); // waits for the actual Enter press
    return 0;
}