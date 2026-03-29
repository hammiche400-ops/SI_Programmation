./# Simulateur de Boids en C avec SDL
### *(inclut une version avancée avec prédateur et bombes)*

Ce projet implémente en C une **murmuration** (la danse collective des étourneaux) en suivant les trois lois de comportement introduites par **Craig Reynolds** :

- **Cohésion** : se diriger vers le centre de gravité des voisins proches ;
- **Séparation** : éviter les collisions en s’éloignant des individus trop proches ;
- **Alignement** : adapter sa vitesse et sa direction à celles des boids environnants.

Pour le visuel, il faut aller dans le fichier BoidsFinalSimple.mov

Une version avancée inclut également :
- un **prédateur** que les boids tentent d’éviter ;
- des **bombes** qui attirent temporairement les boids, puis explosent et se téléportent.

Pour le visuel, il faut aller dans le fichier BoidsFinal.mov

Ce projet permet de modéliser un comportement naturel tout en ajoutant une dimension ludique.

---

## ⭐ Fonctionnalités

- Simulation de boids en C, utilisant une **arithmétique à point fixe Q16.16**.
- Application complète des lois de **Reynolds** : cohésion, séparation, alignement.
- **Grille de hachage spatiale** optimisant la recherche de voisins.
- **Espace torique** : les boids réapparaissent de l’autre côté lorsqu’ils atteignent un bord.
- Prédateur contrôlé au clavier (flèches) : fuite ou mort selon distance.
- Bombes à déclenchement aléatoire, explosion radiale et téléportation.
- Affichage avec **SDL2** : boids en flèche, prédateur en triangle, bombes et animation d'explosion.
- Affichage du **FPS** (SDL_ttf) et limite autour de 60 FPS.

---

## 🎮 Utilisation
### 0. Installer SDL2
https://wiki.libsdl.org/SDL2/Installation
### 1. Lancer le programme
compiler le programme avec le Makefile
```bash
make
```

exécuter :

```bash
./boids
```

### 2. Contrôles (prédateur)
- **↑** : mouvement vers le haut
- **↓** : mouvement vers le bas
- **←** : gauche
- **→** : droite

Le prédateur est plus rapide, les boids fuient ou meurent selon leur distance.

### 3. Bombes
- 5 bombes apparaissent.
- Explosion après un nombre de ticks aléatoires.
- L’explosion repousse les boids ; la bombe se téléporte ensuite.

### 4. Affichage
- Boids : flèches blanches orientées selon la vitesse.
- Prédateur : triangle rouge.
- FPS affiché en haut‑gauche.

### 5. Modifier les paramètres
Modifier les `#define` du fichier `boidsfinal.c` (rayons Reynolds, vitesse max, taille du monde...).

---

## 🔧 Explications techniques

### • Arithmétique à point fixe
Utilisation du format Q16.16 pour éviter les flottants et gagner en performance.
Fonctions utilitaires : `fx_from_double`, `fx_mul`, `fx_div`, `fx_norm2`, `fx_sqrt`.

### • Grille de hachage
Découpage du monde en cases (`TAILLE_HASH`). Chaque case contient une liste chaînée de boids.
Recherche des voisins optimisée : seules les 9 cases autour du boid sont parcourues.

### • Wrap torique
Le monde est un tore : si un boid dépasse un bord, il réapparaît de l’autre côté. Les distances sont calculées en conséquence.

### • Règles de Reynolds
- Cohésion : moyenne des positions → accélération vers le centre.
- Alignement : moyenne des vitesses → harmonisation des directions.
- Séparation : inverse des positions proches → évitement.

Tests sur les distances au carré pour éviter les `sqrt` inutiles.

### • Prédateur
Boids fuient dans un rayon de fuite, meurent dans un rayon plus petit.
Le prédateur est contrôlé au clavier.

### • Bombes
Maintiennent la liste des boids dans leur rayon.
Lors de l’explosion : poussée radiale puis reset.

### • Update
- Mise à jour vitesse/accélération
- Clamp vitesse
- Application poussées bombes
- Déplacement + wrap torique
- Mise à jour de la case dans la grille

### • Rendu SDL
- Boids : flèches orientées via `atan2`
- Prédateur : triangle rempli
- Bombes : cercles + halo d’explosion
- Texte via SDL_ttf
- Framerate régulé via performance counter

---
