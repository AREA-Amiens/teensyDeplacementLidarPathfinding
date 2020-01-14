#include <Arduino.h>
#include <Wire.h>         //liaison I2C pour les cartes
#include <FastCRC.h>      //gestion d'erreur sur bus I2C
#include <AccelStepper.h> //un moteur pas a pas
        //AccelStepper motor_left(1, step_left, dir_left); definie la comme avec le diver
        //
#include <MultiStepper.h> //getion simultaner de moteur pas a pas limiter
#include <math.h>//pour les calculs

#include <MsTimer2.h>

typedef struct{ //struture pour enregistrer les x et y en même temps
  int x;
  int y;
}pos;

#define my_adr 10         //mon adresse bus I2C
#define reset_G   15 // pin du moteur gauche
#define sleep_G   13
#define step_G    0
#define dir_G     1

#define reset_D   9// pin du moteur droit
#define sleep_D   10
#define step_D    11
#define dir_D     12


#define acceleration  5200//accélération du robot x 2coéficient_go
#define speed         250

#define coeficien_turn /*12.99399105*/  -4.081256839 //coefficiant determiné par rapport au diamètre de roue, sélection du pas(ici 1/4),le tout pour 1°
#define coeficien_go   -19.32552 //coefficiant determiné par rapport au diamètre de roue, sélection du pas(ici 1/4),le tout pour 1mm

#define pi 3.1415926536//


void receiveEvent(int howMany);
//fonction pour quand l'esclave reçoit une trame de adr_actionneur
//met la trame dans un tableau


void requestEvent();
//fonction pour quand le maitre appelle l'esclave
//renvoi si l'esclave et pret à recevoir une trame(déplacment fini)

byte iso_bite(byte analiser, byte decalage);
//va isoler le bite du decalge
//analise = xxxx xxxx
//dealage = 2
//return = 0000 00x00

pos position ();//decode les trames envoyer par byte par la carte stratégie pour les x et y
//renvoie position de x et y
void recalageY (pos &pos1);//recalage en Y en fonction du mur, et de l'av ou ar
void recalageX (pos &pos1);//recalage en X en fonction du mur, et de l'av ou ar
void MouvementTourelle();
//mouvemnt servo tourelle
void recaleFinAv();//ajustement avant
void recaleFinAr();//ajustement arrière
