#include <Arduino.h>

typedef struct{
  uint8_t x; //position x,y du noeud
  uint8_t y;
  float h; // heuristique
}noeud;

void triliste(noeud liste[3]);
void choixdir(uint8_t dir,noeud objectif,noeud depart);
uint8_t algoPAstar(uint8_t table[150][100],noeud objectif,noeud depart);
void initTable(uint8_t table[150][100]);
void affichetab();
void posEnemi(int posx,int posy);
noeud cheminRobot();
