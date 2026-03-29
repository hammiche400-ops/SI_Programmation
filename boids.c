#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define WIDTH 800
#define HEIGHT 800
#define N_BOIDS 1000

#define NB_BOMB 5
#define W_BOMB 3.0            // coefficient du poids de la bombe

#define V_LIM 0.5
#define R_COHESION 80              
#define R_ALIGNEMENT 40
#define R_SEPARATION 10
#define W_COH 0.002
#define W_ALIGN 0.004
#define W_SEP 0.02

#define BOMB_RADIUS 75.0
#define BOMB_EXPLOSION_TICKS 30
#define BOMB_TICK_MIN 100
#define BOMB_TICK_RANGE 400

#define PRED_REACT 1.0
#define PRED_INIT_ACCEL 0.1
#define PRED_SPEED_FACTOR 1.9
#define FLEE_RADIUS 50.0
#define KILL_RADIUS 10.0
#define FLEE_FORCE 1.5
#define FPS_VISE 60.0
#define TAILLE_HASH R_COHESION
#define NB_CASE_X (WIDTH / TAILLE_HASH)
#define NB_CASE_Y (HEIGHT / TAILLE_HASH)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* L'arithmétique à point fixe permet de représenter des nombres réels avec une précision fixe en utilisant un nombre*/
/* de bits fixe pour la partie entière et fractionnaire, pour cela on créée une structure de type int32_t pour les nombres à point fixe. */
/* Pour passer d'un double à un nombre à point fixe, on décale de FX_SHIFT bits vers la gauche, ca revient à multiplier par FX_ONE = 2^FX_SHIFT */
#define FX_SHIFT 16              // Nombre de bits pour la partie fractionnaire
#define FX_ONE (1 << FX_SHIFT)
typedef int32_t fx;              // On renomme int32_t en fx pour plus de clarté

static inline fx fx_from_double(double d){ return (fx)(d * FX_ONE); }                    
static inline fx fx_mul(fx a, fx b){ return (fx)(((int64_t)a * b) >> FX_SHIFT); } // Multiplication de deux nombres à point fixe, on décale de FX_SHIFT bits vers la droite pour compenser le décalage à gauche lors de la conversion
/*En effet, pour i=a*2^FX_SHIFT, et j = b*2^FX_SHIFT, on a i*j = a*b*2^(2*FX_SHIFT), donc on décale de FX_SHIFT bits vers la droite pour compenser le décalage à gauche lors de la conversion */
static inline fx fx_div(fx a, fx b){ return (fx)(((int64_t)a << FX_SHIFT) / b); } // Division de deux nombres à point fixe, on décale de FX_SHIFT bits vers la gauche avant la division pour compenser le décalage à droite lors de la conversion
static inline fx fx_norm2(fx x, fx y){ return (fx)(((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT); } 
static inline int64_t fx_norm2_64(fx x, fx y){ return (((int64_t)x * x + (int64_t)y * y) >> FX_SHIFT); } //
static inline fx fx_sqrt(fx a){                 // Racine carrée d'un nombre à point fixe, on convertit en double pour utiliser la fonction sqrt
    double d = (double)a / (double)FX_ONE;
    return fx_from_double(sqrt(d));
}

static void draw_filled_circle(SDL_Renderer* r, int cx, int cy, int radius){ // dessine une bombe ronde
    for (int dy = -radius; dy <= radius; dy++){                   // pour chaque "hauteur" dans le cercle
        int w = (int)sqrt((double)(radius * radius - dy * dy));     // demi-largeur de la ligne horizontale à cette hauteur (pythagore)
        SDL_RenderDrawLine(r, cx - w, cy + dy, cx + w, cy + dy);    // dessine la ligne horizontale
    }
}
/* l'idée est de tracer un triangle en dessinant des lignes horizontales entre les intersections de chaque ligne horizontale avec les côtés du triangle */
static void draw_filled_triangle(SDL_Renderer* r, double x1, double y1, double x2, double y2, double x3, double y3){
    double miny = fmin(y1, fmin(y2, y3));                 // coordonée minimale y du triangle
    double maxy = fmax(y1, fmax(y2, y3));                 // max
    int start_y = (int)floor(miny);                       // valeur arrondie inférieure
    int end_y = (int)ceil(maxy);                          // valeur arrondie supérieure
    for (int y = start_y; y <= end_y; y++){          // pour chaque ligne horizontale entre miny et maxy
        double scan_y = (double)y + 0.5;              //important : on prend le centre du pixel (car pixel != point mais pixel = surface)
        double xs[3];                                   //array pour stocker les intersections
        int count = 0;                             // nombre d'intersections trouvées                           
        if ((scan_y >= fmin(y1, y2)) && (scan_y <= fmax(y1, y2)) && y1 != y2){     // si la ligne intersecte le segment (x1,y1)-(x2,y2)
            xs[count++] = x1 + (scan_y - y1) * (x2 - x1) / (y2 - y1);               // calcul de l'abscisse d'intersection
        }
        if ((scan_y >= fmin(y2, y3)) && (scan_y <= fmax(y2, y3)) && y2 != y3){
            xs[count++] = x2 + (scan_y - y2) * (x3 - x2) / (y3 - y2);
        }
        if ((scan_y >= fmin(y3, y1)) && (scan_y <= fmax(y3, y1)) && y3 != y1){
            xs[count++] = x3 + (scan_y - y3) * (x1 - x3) / (y1 - y3);
        }
        if (count < 2) continue;              // si moins de 2 intersections, on passe à la ligne suivante
        double xa = xs[0], xb = xs[1];      // on récupère les deux intersections
        if (xa > xb){                    // on veut xa < xb
            double t = xa; xa = xb; xb = t;
        }
        SDL_RenderDrawLine(r, (int)floor(xa), y, (int)ceil(xb), y); // dessine la ligne horizontale entre les deux intersections
    }
}



typedef struct {   
    fx x, y;
    fx vx, vy;
    fx ax, ay;
} predator;

typedef struct {
    int id;
    fx x, y;
    fx vx, vy;
    fx ax, ay;
    fx vx_bomb, vy_bomb;    // vitesse additionnelle due à l'explosion d'une bombe (généralement nulle)
    int case_x, case_y;     // position dans la grille de hachage
    int vie;                // 1 en vie, 0 mort
} Boid;

typedef struct Boid_in_Bomb{   // liste chaînée des boids dans une bombe
    int id_boid;
    struct Boid_in_Bomb* next;
} Boid_in_Bomb;

typedef struct Bomb{     
    int id;
    fx x, y;
    int tick;            // compteur de ticks
    int tick_end;        // nombre de ticks avant explosion
    int64_t radius2;     // rayon d'explosion au carré (pour simplifier les calculs)
    Boid_in_Bomb* boids; // liste chaînée des boids dans la bombe
    int explosion_timer; // timer pour l'affichage de l'explosion
    fx blast_x, blast_y; // position de l'explosion (pour l'animation)
} Bomb;

typedef struct hashed{   // liste chaînée pour la grille de hachage
    int id;
    struct hashed* next;
} hashed;


Boid boids[N_BOIDS];                          
hashed* grille[NB_CASE_X][NB_CASE_Y] = {0};   // grille de hachage, iniatialisée à NULL (pointeur car liste chaînée dynamique)
predator pred; 
Bomb bombs[NB_BOMB] = {0};                    // tableau des bombes, initialisé à NULL


/* Variables globales pour les paramètres en point fixe */
fx fx_r_coh2, fx_r_align2, fx_r_sep2;
fx fx_w_coh, fx_w_align, fx_w_sep;
fx fx_v_lim2, fx_v_lim2_pred;
fx fx_fuite2, fx_kill2;
fx fx_w_ecran, fx_h_ecran, fx_half_w, fx_half_h;
fx fx_w_bomb;


static inline void ajouter_boid_a_case(int cx, int cy, int id){
    hashed* c = (hashed*)malloc(sizeof(hashed));  //pointeur pour ne pas perdre la liste chaînée (car c est une variable locale)
    c->id = id;
    c->next = grille[cx][cy];
    grille[cx][cy] = c;
}

static inline void supprimer_boid_de_case(int cx, int cy, int id){
    hashed** cur = &grille[cx][cy];            //double pointeur car on veut modifier la liste chaînée sans la perdre  
    while (*cur){
        if ((*cur)->id == id){
            hashed *victim = *cur;
            *cur = victim->next;
            free(victim);
            break;
        }
        cur = &(*cur)->next;
    }
}

static inline void InBomb_add(int id_bomb, int id_boid){            // ajoute (en haut de la "pile") un boid à la liste des boids dans une bombe
    Boid_in_Bomb* h = (Boid_in_Bomb*)malloc(sizeof(Boid_in_Bomb));
    h->id_boid = id_boid;
    h->next = bombs[id_bomb].boids;  
    bombs[id_bomb].boids = h;
}

static inline void InBomb_remove(int id_bomb, int id_boid){ // supprime un boid de la liste des boids d'une bombe
    Boid_in_Bomb** cur = &bombs[id_bomb].boids;
    while (*cur){
        if ((*cur)->id_boid == id_boid){
            Boid_in_Bomb* victim = *cur;
            *cur = victim->next;
            free(victim);
            return;
        }
        cur = &(*cur)->next;
    }
}

static inline int InBomb_contains(int id_bomb, int id_boid){      // vérifie si un boid est dans la liste des boids d'une bombe
    for (Boid_in_Bomb* j = bombs[id_bomb].boids; j; j = j->next){
        if (j->id_boid == id_boid)
            return 1;
    }
    return 0;
}


static inline void ajouter_acceleration_dirigee(fx dx, fx dy, fx C, fx* axs, fx* ays){ // ajoute une accélération dirigée de norme C dans la direction (dx, dy)
    fx n = fx_sqrt(fx_norm2(dx, dy));  // norme de (dx, dy) = distance
    if (n == 0) return;
    fx ux = fx_div(dx, n);  // vecteur unitaire selon x
    fx uy = fx_div(dy, n);  

    *axs += fx_mul(C, ux);  // ajout de l'accélération selon x
    *ays += fx_mul(C, uy); 
}

static inline fx fx_rand_range(double min, double max){ // génère un nombre aléatoire en point fixe dans l'intervalle [min, max[
    double r = (double)(rand() % 10000) / 10000.0;
    return fx_from_double(min + r * (max - min));
}

static void free_bomb_list(int id_bomb){                // libère la mémoire de la liste des boids dans une bombe à son explosion
    Boid_in_Bomb* cur = bombs[id_bomb].boids;
    while (cur){
        Boid_in_Bomb* next = cur->next;
        free(cur);
        cur = next;
    }
    bombs[id_bomb].boids = NULL;
}

static void Bomb_Detection(){                      // gère les explosions des bombes et l'effet sur les boids
    for (int i = 0; i < NB_BOMB; i++){ // pour chaque bombe
        Bomb* bomb = &bombs[i];
        bomb->tick++;                   // +1 tick
        if (bomb->tick >= bomb->tick_end){ // si le nombre de ticks atteint le seuil d'explosion
            bomb->explosion_timer = BOMB_EXPLOSION_TICKS;    // timer pour l'affichage de l'explosion
            bomb->blast_x = bomb->x;       
            bomb->blast_y = bomb->y;
            for (Boid_in_Bomb* j = bombs[i].boids; j; j = j->next){ // pour chaque boid dans la bombe
                Boid* b1 = &boids[j->id_boid];
                fx dx = b1->x - bomb->x;
                fx dy = b1->y - bomb->y;

                fx dist = fx_sqrt(fx_norm2(dx, dy));   // distance entre le boid et la bombe
                if (dist == 0) continue;

                fx ux = fx_div(dx, dist);              // vecteur unitaire selon x
                fx uy = fx_div(dy, dist);
                b1->vx_bomb = fx_mul(fx_w_bomb, ux);   // ajout de la vitesse due à l'explosion
                b1->vy_bomb = fx_mul(fx_w_bomb, uy);
            }
            /*Réinitialisation de la bombe*/
            bomb->tick = 0;     
            bomb->x = fx_from_double(rand() % WIDTH);
            bomb->y = fx_from_double(rand() % HEIGHT);
            bomb->tick_end = BOMB_TICK_MIN + rand() % BOMB_TICK_RANGE; 
            fx rad = fx_from_double(BOMB_RADIUS);
            bomb->radius2 = fx_mul(rad, rad);
            free_bomb_list(i);
        }
        if (bomb->explosion_timer > 0) bomb->explosion_timer--; // décrémentation du timer d'explosion
        
    }
}


static void Reynold_Pred(Boid *b, predator* pred, fx* axs, fx* ays){  // calcule les accélérations selon les règles de Reynolds et l'évitement du prédateur
    if (!b->vie){                // si le boid est mort, pas d'accélération
        *axs = 0;
        *ays = 0;
        return;
    }
    /*Initialisation des variables*/
    fx Gx = 0, Gy = 0;
    fx Sx = 0, Sy = 0;
    fx vx_ali = 0, vy_ali = 0;
    int nG = 1, nA = 0, nS = 0;

    for (int dx = -1; dx <= 1; dx++){             // pour chaque case voisine dans la grille de hachage
        for (int dy = -1; dy <= 1; dy++){

            int case_x = (b->case_x + dx + NB_CASE_X) % NB_CASE_X;
            int case_y = (b->case_y + dy + NB_CASE_Y) % NB_CASE_Y;

            for(hashed* h = grille[case_x][case_y]; h; h = h->next){ // pour chaque boid dans la case
                Boid* b1 = &boids[h->id];
                if (b1 == b) continue;
                /*Calcul des distances et directions*/
                fx rx = b1->x - b->x;                    
                if (rx > fx_half_w) rx -= fx_w_ecran;       //Prise en compte de l'effet de tore, si la distance est plus grande que la moitié de la largeur, on soustrait la largeur
                if (rx < -fx_half_w) rx += fx_w_ecran;      //pour calculer la distance entre deux points sur un tore

                fx ry = b1->y - b->y;
                if (ry > fx_half_h) ry -= fx_h_ecran;
                if (ry < -fx_half_h) ry += fx_h_ecran;

                fx dist = fx_norm2(rx, ry);

                if(dist <= fx_r_coh2){                     // si le boid est dans le rayon de cohésion
                    Gx += rx;             // somme des positions des boids voisins pour la cohésion
                    Gy += ry;
                    nG += 1;              // nombre de boids voisins
                    if(dist <= fx_r_align2){           // si le boid est dans le rayon d'alignement
                        vx_ali += b1->vx; // somme des vitesses des boids voisins
                        vy_ali += b1->vy;
                        nA += 1;           // nombre de boids voisins pour l'alignement
                        if(dist <= fx_r_sep2){  // si le boid est dans le rayon de séparation
                            Sx -= rx;            // somme des positions des boids voisins pour la séparation (inverse de la position)
                            Sy -= ry;
                            nS += 1;           // nombre de boids voisins pour la séparation
                            //on prend l'inverse de la position pour s'éloigner des autres boids
    
                        }
                    }
                }

            }
        }
    }
    fx ax_coh = 0, ay_coh = 0, ax_sep = 0, ay_sep = 0, ax_ali = 0, ay_ali = 0;  // initialisation des accélérations

    if (nS != 0){ // si il y a des boids à séparer
        Sx /= nS; // moyenne des positions des boids voisins pour la séparation
        Sy /= nS;
        ajouter_acceleration_dirigee(Sx, Sy, fx_w_sep, &ax_sep, &ay_sep); // ajout de l'accélération de séparation (tendance à s'éloigner des autres boids)
    }

    Gx /= nG; // moyenne des positions des boids voisins pour la cohésion
    Gy /= nG;
    ajouter_acceleration_dirigee(Gx, Gy, fx_w_coh, &ax_coh, &ay_coh); // ajout de l'accélération de cohésion (tendance à se rapprocher des autres boids)
    if (nA != 0){      // si il y a des boids à aligner
        vx_ali /= nA;   // moyenne des vitesses des boids voisins pour l'alignement
        vy_ali /= nA;
        fx norme = fx_sqrt(fx_norm2(vx_ali, vy_ali));
        if (norme != 0){
            fx ux = fx_div(vx_ali, norme);     // vecteur unitaire selon x
            fx uy = fx_div(vy_ali, norme);
            ax_ali = fx_mul(fx_w_align, ux);   // ajout de l'accélération d'alignement (tendance à aligner sa vitesse avec celle des autres boids)
            ay_ali = fx_mul(fx_w_align, uy);
        }
    }
    *axs = ax_ali + ax_coh + ax_sep;   // somme des accélérations
    *ays = ay_ali + ay_coh + ay_sep;

    //Evitement du prédateur
    fx px = pred->x - b->x;
    fx py = pred->y - b->y;
    if (px > fx_half_w) px -= fx_w_ecran;  // prise en compte de l'effet de tore
    if (px < -fx_half_w) px += fx_w_ecran;
    if (py > fx_half_h) py -= fx_h_ecran;
    if (py < -fx_half_h) py += fx_h_ecran;

    int64_t dist_pred2 = fx_norm2_64(px, py); 

    if (dist_pred2 < fx_kill2){  // si le boid est trop proche du prédateur, il meurt
        b->vie = 0;
        supprimer_boid_de_case(b->case_x, b->case_y, b->id);
        b->vx = b->vy = b->ax = b->ay = 0;
        b->vx_bomb = b->vy_bomb = 0;
        for (int i = 0; i < NB_BOMB; i++){
            InBomb_remove(i, b->id);
        }
        return;
    }

    if (dist_pred2 < fx_fuite2){  // si le boid est dans la zone de fuite du prédateur
        fx n = fx_sqrt((fx)dist_pred2);
        if (n != 0){
            fx ux = fx_div(px, n);   // vecteur unitaire selon x
            fx uy = fx_div(py, n);
            fx flee = fx_from_double(FLEE_FORCE);  // intensité de la fuite
            *axs = -fx_mul(flee, ux);       // ajout de l'accélération de fuite (tendance à s'éloigner du prédateur)
            *ays = -fx_mul(flee, uy);
        }
    }


}


static void update(Boid* b){        // met à jour la position et la vitesse du boid et autre
    if (!b->vie) return;

    b->vx += b->ax;
    b->vy += b->ay;


    fx v2 = fx_norm2(b->vx, b->vy);
    if (v2 > fx_v_lim2) {  // limitation de la vitesse en gardant la direction
        fx scale = fx_div(fx_from_double(V_LIM), fx_sqrt(v2));
        b->vx = fx_mul(b->vx, scale);
        b->vy = fx_mul(b->vy, scale);
    }
    if (b->vx_bomb || b->vy_bomb){ // ajout de la vitesse due à l'explosion d'une bombe
        b->vx += b->vx_bomb;
        b->vy += b->vy_bomb;
        b->vx_bomb = 0;
        b->vy_bomb = 0;
    }   // on l'a met après la limitation de vitesse pour éviter que la vitesse due à la bombe ne soit limitée

    b->x += b->vx;
    b->y += b->vy;

    /* gestion des bords (effet de tore)*/
    if (b->x < 0) b->x += fx_w_ecran;
    if (b->x >= fx_w_ecran) b->x -= fx_w_ecran;
    if (b->y < 0) b->y += fx_h_ecran;
    if (b->y >= fx_h_ecran) b->y -= fx_h_ecran;

    int ncx = (b->x >> FX_SHIFT) / TAILLE_HASH;  // nouvelle case x
    int ncy = (b->y >> FX_SHIFT) / TAILLE_HASH;

    /* gestion des cases */
    if (ncx != b->case_x || ncy != b->case_y) { // si le boid a changé de case
        supprimer_boid_de_case(b->case_x, b->case_y, b->id); // suppression de l'ancien boid de la case
        b->case_x = ncx;
        b->case_y = ncy;
        ajouter_boid_a_case(ncx, ncy, b->id); // ajout du boid à la nouvelle case
    }

    for (int i = 0; i < NB_BOMB; i++){  // pour chaque bombe, on vérifie si le boid est dans la bombe
        Bomb* bomb = &bombs[i];
        fx dx = b->x - bomb->x;
        fx dy = b->y - bomb->y;
        int64_t d2 = fx_norm2_64(dx, dy);
        if (d2 <= bomb->radius2) {         // si le boid est dans la bombe
            if (!InBomb_contains(i, b->id)) {   // et qu'il n'y est pas déjà
                InBomb_add(i, b->id);         // on l'ajoute à la liste des boids dans la bombe
            }
        } else { 
            InBomb_remove(i, b->id);  //sinon on le retire de la liste des boids dans la bombe (s'il y est)
        }
    }
    
}

static void update_predator(predator* pred){  // met à jour la position et la vitesse du prédateur
    pred->vx += pred->ax;
    pred->vy += pred->ay;
    fx v2 = fx_norm2(pred->vx, pred->vy);
    if (v2 > fx_v_lim2_pred){  // limitation de la vitesse en gardant la direction
        fx maxp = fx_sqrt(fx_v_lim2_pred);
        fx n = fx_sqrt(v2);
        fx scale = fx_div(maxp, n);
        pred->vx = fx_mul(pred->vx, scale);
        pred->vy = fx_mul(pred->vy, scale);
    }
    pred->x += pred->vx;
    pred->y += pred->vy;

    /* gestion des bords (effet de tore)*/
    if (pred->x < 0) pred->x += fx_w_ecran;
    if (pred->x >= fx_w_ecran) pred->x -= fx_w_ecran;
    if (pred->y < 0) pred->y += fx_h_ecran;
    if (pred->y >= fx_h_ecran) pred->y -= fx_h_ecran;
}



static void init_constants(){             //calcul des constantes en point fixe, on les calcule une seule fois au début
    fx r_coh = fx_from_double(R_COHESION);
    fx r_ali = fx_from_double(R_ALIGNEMENT);
    fx r_sep = fx_from_double(R_SEPARATION);
    fx_r_coh2 = fx_mul(r_coh, r_coh);
    fx_r_align2 = fx_mul(r_ali, r_ali);
    fx_r_sep2 = fx_mul(r_sep, r_sep);

    fx_w_coh = fx_from_double(W_COH);
    fx_w_align = fx_from_double(W_ALIGN);
    fx_w_sep = fx_from_double(W_SEP);
    fx_w_bomb = fx_from_double(W_BOMB);

    fx vl = fx_from_double(V_LIM);
    fx_v_lim2 = fx_mul(vl, vl);
    
    fx vl_pred = fx_mul(vl, fx_from_double(PRED_SPEED_FACTOR));   // le prédateur est plus rapide que les boids
    fx_v_lim2_pred = fx_mul(vl_pred, vl_pred);

    fx kill_r = fx_from_double(KILL_RADIUS);
    fx flee_r = fx_from_double(FLEE_RADIUS);
    fx_kill2 = fx_mul(kill_r, kill_r);
    fx_fuite2 = fx_mul(flee_r, flee_r);

    fx_w_ecran = fx_from_double(WIDTH);
    fx_h_ecran = fx_from_double(HEIGHT);
    fx_half_w = fx_w_ecran / 2;
    fx_half_h = fx_h_ecran / 2;
}


static void draw_arrow(SDL_Renderer* r, fx x, fx y, fx vx, fx vy, double size){ // dessine un boid sous forme de flèche on reste en double pour le dessin
    double px = (double)x / FX_ONE; // conversion en double pour le dessin
    double py = (double)y / FX_ONE;
    double heading = atan2((double)vy, (double)vx); // angle de la flèche
    double tip_x = px + cos(heading) * size;        //pointe
    double tip_y = py + sin(heading) * size;
    double left_x = px + cos(heading + 2.5) * size * 0.6;   // aile gauche
    double left_y = py + sin(heading + 2.5) * size * 0.6;
    double right_x = px + cos(heading - 2.5) * size * 0.6;   // aile droite
    double right_y = py + sin(heading - 2.5) * size * 0.6;

    SDL_Point pts[4] = {             //points de la flèche
        {(int)tip_x, (int)tip_y},  
        {(int)left_x, (int)left_y},
        {(int)right_x, (int)right_y},
        {(int)tip_x, (int)tip_y}
    };
    SDL_RenderDrawLines(r, pts, 4);  // dessin de la flèche (lignes entre les points)
}

static void draw_predator(SDL_Renderer* r, fx x, fx y, fx vx, fx vy){
    double px = (double)x / FX_ONE;
    double py = (double)y / FX_ONE;
    double heading = atan2((double)vy, (double)vx);
    if (vx == 0 && vy == 0) heading = -M_PI / 2.0; // orientation par défaut vers le haut
    double size = 18.0;
    double wing = size * 0.65;
    double tip_x = px + cos(heading) * size;
    double tip_y = py + sin(heading) * size;
    double left_x = px + cos(heading + 2.5) * wing;
    double left_y = py + sin(heading + 2.5) * wing;
    double right_x = px + cos(heading - 2.5) * wing;
    double right_y = py + sin(heading - 2.5) * wing;

    SDL_SetRenderDrawColor(r, 220, 30, 30, 255);
    draw_filled_triangle(r, tip_x, tip_y, left_x, left_y, right_x, right_y);
    // "détail", triangle plus petit et plus clair
    SDL_SetRenderDrawColor(r, 255, 180, 160, 255);
    double inset = 0.6;
    double tip2_x = px + cos(heading) * size * inset;
    double tip2_y = py + sin(heading) * size * inset;
    double left2_x = px + cos(heading + 2.5) * wing * inset;
    double left2_y = py + sin(heading + 2.5) * wing * inset;
    double right2_x = px + cos(heading - 2.5) * wing * inset;
    double right2_y = py + sin(heading - 2.5) * wing * inset;
    draw_filled_triangle(r, tip2_x, tip2_y, left2_x, left2_y, right2_x, right2_y);
}


int main() {
    init_constants();   // initialisation des constantes en point fixe
    if (SDL_Init(SDL_INIT_VIDEO) != 0){                      // initialisation de SDL
        fprintf(stderr, "SDL_Init error: %s\n", SDL_GetError()); 
        return 1;
    }

    SDL_Window* w = SDL_CreateWindow(               // création de la fenètre de taile width x height
        "Boids",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WIDTH, HEIGHT, 0
    );


    SDL_Renderer* r = SDL_CreateRenderer(w, -1, SDL_RENDERER_ACCELERATED); // création du renderer, c'est lui qui gère le rendu graphique

    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);

    TTF_Init(); // initialisation de SDL_ttf pour le texte
    TTF_Font* font = TTF_OpenFont("Open_Sans/OpenSans-VariableFont_wdth,wght.ttf", 18); // chargement de la police d'écriture, taille 18


    SDL_Color fps_color = {255, 255, 255, 255}; // couleur blanche pour le texte FPS

    // initialisation des boids, du prédateur et des bombes
    pred.x = fx_from_double(WIDTH / 2);
    pred.y = fx_from_double(HEIGHT / 2);
    pred.vx = 0;
    pred.vy = 0;
    pred.ax = pred.ay = fx_from_double(PRED_INIT_ACCEL);

    for (int i = 0; i < NB_BOMB; i++){
        bombs[i].id = i;
        bombs[i].x = fx_from_double(rand() % WIDTH);        // position aléatoire
        bombs[i].y = fx_from_double(rand() % HEIGHT);
        bombs[i].tick = 0;
        bombs[i].tick_end = BOMB_TICK_MIN + rand() % BOMB_TICK_RANGE; // nombre de ticks avant explosion aléatoire
        fx rad = fx_from_double(BOMB_RADIUS);
        bombs[i].radius2 = fx_mul(rad, rad);
        bombs[i].boids = NULL;                // liste des boids dans la bombe initialisée à NULL
        bombs[i].blast_x = bombs[i].x;        // position de l'explosion initialisée à la position de la bombe, on en a besoin pour l'animation car la bombe peut se déplacer avant l'explosion
        bombs[i].blast_y = bombs[i].y;
        bombs[i].explosion_timer = 0;
    }

    for (int i = 0; i < N_BOIDS; i++) {
        boids[i].vie = 1;
        boids[i].id = i;
        boids[i].x = fx_from_double(rand() % WIDTH);      // position aléatoire
        boids[i].y = fx_from_double(rand() % HEIGHT);
        boids[i].vx = fx_rand_range(-0.2, 0.2);           // vitesse aléatoire
        boids[i].vy = fx_rand_range(-0.2, 0.2);
        boids[i].ax = boids[i].ay = 0;
        boids[i].vx_bomb = boids[i].vy_bomb = 0;

        boids[i].case_x = (boids[i].x >> FX_SHIFT) / TAILLE_HASH;   // calcul de la case dans la grille de hachage
        boids[i].case_y = (boids[i].y >> FX_SHIFT) / TAILLE_HASH;
        ajouter_boid_a_case(boids[i].case_x, boids[i].case_y, i);
        
    }
    Uint64 freq = SDL_GetPerformanceFrequency();               // fréquence du compteur de performance pour la gestion du framerate, c'est à dire le nombre de ticks par seconde
    Uint64 debut = SDL_GetPerformanceCounter();                // compteur de performance au début de la boucle, c'est à dire le nombre de ticks depuis le démarrage de SDL != 0
    int running = 1;
    while (running) {
        SDL_Event e;                            // nom de la variable pour les événements SDL
        while (SDL_PollEvent(&e))               // boucle pour traiter les événements SDL
            if (e.type == SDL_QUIT) running = 0;  // si l'événement est de type QUIT (fermeture de la fenêtre), on arrête la boucle

        SDL_SetRenderDrawColor(r,135,206,235,255);  // couleur de fond (bleu ciel)
        SDL_RenderClear(r);             // effacement de l'écran avec la couleur de fond                   

        Bomb_Detection();            // gestion des bombes avant la mise à jour des boids car l'explosion affecte la vitesse des boids

        for (int i = 0; i < NB_BOMB; i++){
            int blastx = (int)((bombs[i].blast_x) >> FX_SHIFT);
            int blasty = (int)((bombs[i].blast_y) >> FX_SHIFT);
            int bx = (int)((bombs[i].x) >> FX_SHIFT);
            int by = (int)((bombs[i].y) >> FX_SHIFT);
            if (bombs[i].explosion_timer > 0){   // si la bombe est en train d'exploser, on dessine le blast
                double t = 1.0 - (double)bombs[i].explosion_timer / (double)BOMB_EXPLOSION_TICKS; // progression de l'animation
                int base_rad = (int)sqrt((double)bombs[i].radius2 / (double)FX_ONE); //passage en double
                int rad = base_rad + (int)(t * base_rad * 0.35);  // expansion légère (35%) de l'explosion
                int alpha = (int)(200 * (1.0 - t)); 
                // halo extérieur
                SDL_SetRenderDrawColor(r,255,140,0,alpha);
                draw_filled_circle(r, blastx, blasty, rad);
                // cœur lumineux
                SDL_SetRenderDrawColor(r,255,220,120,alpha);
                draw_filled_circle(r, blastx, blasty, rad/2);
            }
            // dessin de la bombe (deux cercles rouges)
            SDL_SetRenderDrawColor(r,180,30,30,255);
            draw_filled_circle(r, bx, by, 6);  
            SDL_SetRenderDrawColor(r,250,80,80,255);
            draw_filled_circle(r, bx, by, 3);
        }

        for (int i = 0; i < N_BOIDS; i++) {  // pour chaque boid
            if (!boids[i].vie) continue;    // si le boid est mort, on ne le met pas à jour ni ne l'affiche
            Reynold_Pred(&boids[i], &pred,  &boids[i].ax, &boids[i].ay);  //Reynolds + évitement du prédateur
            update(&boids[i]);                                          // mise à jour du boid
            SDL_SetRenderDrawColor(r,255,255,255,255);                     // couleur blanche pour les boids
            draw_arrow(r, boids[i].x, boids[i].y, boids[i].vx, boids[i].vy, 8.0);       // dessin du boid
        }
        
        update_predator(&pred);                              // mise à jour du prédateur
        draw_predator(r, pred.x, pred.y, pred.vx, pred.vy); // dessin du prédateur

        /* Contrôle du prédateur avec le clavier */
        const Uint8* state = SDL_GetKeyboardState(NULL); // permet de récuper l'état des touches du clavier

        fx react = fx_from_double(PRED_REACT); // coefficient de reaction du prédateur lorsqu'une tout est appuyé
        pred.ax = 0;
        pred.ay = 0;

        if (state[SDL_SCANCODE_UP])    pred.ay -= react; // haut 
        if (state[SDL_SCANCODE_DOWN])  pred.ay += react; // bas 
        if (state[SDL_SCANCODE_LEFT])  pred.ax -= react; // gauche 
        if (state[SDL_SCANCODE_RIGHT]) pred.ax += react; // droite

        /* Uint = type définit par SDL pour éviter les overflows  (temps très grand) */
        Uint64 fin = SDL_GetPerformanceCounter();                   // compteur de performance à la fin de la boucle
        Uint64 elapsed_ticks = fin - debut;                      // nombre de ticks écoulés depuis le début de la boucle                                            
        Uint64 target_ticks = (Uint64)((double)freq / FPS_VISE);  // nombre de ticks visés pour atteindre 60 FPS
        if (elapsed_ticks < target_ticks){                        // si le temps écoulé est inférieur au temps visé, on attend
            Uint64 remaining = target_ticks - elapsed_ticks;      
            Uint32 delay_ms = (Uint32)(remaining * 1000 / freq); //*1000 pour convertir en millisecondes
            if (delay_ms > 0){       // si il y a un délai, on attend (ici c'est grossier pour éviter d'utiliser trop de CPU)
                SDL_Delay(delay_ms);
            }
            while ((SDL_GetPerformanceCounter() - fin) < remaining){}  // on est plus précis là mais ça utilise du CPU
            fin = SDL_GetPerformanceCounter();
            elapsed_ticks = fin - debut;
        }

        double delta = (double)elapsed_ticks/(double)(freq);       // temps écoulé en secondes
        double fps = 1.0/delta; // calcul du FPS
        char texte_fps[32];   // chaîne de caractères pour le texte FPS (32 pour être large)
        snprintf(texte_fps, sizeof(texte_fps), "FPS: %.2f", fps);                   // formatage du texte FPS

        SDL_Surface* surfaceTexte = TTF_RenderText_Solid(font, texte_fps, fps_color); // rendu du texte en surface
        SDL_Texture* texture = SDL_CreateTextureFromSurface(r, surfaceTexte);  // création de la texture à partir de la surface
        SDL_FreeSurface(surfaceTexte);   // libération de la surface après création de la texture

        SDL_Rect txt = {10, 10, 0, 0};               // position du texte en haut à gauche
        SDL_QueryTexture(texture, NULL, NULL, &txt.w, &txt.h);   // récupération de la largeur et hauteur du texte
        SDL_RenderCopy(r, texture, NULL, &txt);                  // copie de la texture du texte dans le renderer
        SDL_DestroyTexture(texture);                            // libération de la texture après affichage
    
    
        SDL_RenderPresent(r);                                 // affichage du renderer à l'écran

        debut = fin;  // mise à jour du compteur de début pour la prochaine boucle
    }

    /* Nettoyage */
    TTF_CloseFont(font);
    TTF_Quit();
    SDL_DestroyRenderer(r);
    SDL_DestroyWindow(w);
    SDL_Quit();
    return 0;
}
