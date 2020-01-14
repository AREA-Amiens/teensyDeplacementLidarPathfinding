#include <Arduino.h>

typedef struct{
  uint8_t x; //position x,y du noeud
  uint8_t y;
  float h; // heuristique
}noeud;

void triliste(noeud liste[3]);
void choixdir(uint8_t dir,noeud objectif,noeud depart);
void algoPAstar(uint8_t table[150][100],noeud objectif,noeud depart);
void initTable();
void affichetab();
void posEnemi(int posx,int posy);
noeud cheminRobot(/*noeud liste[150]*/);
uint8_t table[150][100];

noeud listeRetenue[250]; //liste retenue de noeud
noeud listeAttente[3];
noeud n1;
noeud n2;
noeud n3;

noeud noeudcourant;
noeud posrobot;
noeud objectif;
noeud noeudparent;

float pente;
float b;
uint8_t dir;
uint8_t noeudfaux=0;
uint8_t nbrnoeud;

uint8_t xcourant;
uint8_t ycourant;

float t1;
float ttot;
float pente1;
float pente2;
float pente3;
