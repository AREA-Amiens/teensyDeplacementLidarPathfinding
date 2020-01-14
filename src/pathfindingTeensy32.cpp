#include <pathfindingTeensy32.h>

void affichetab(){
  //Serial.print("b");
  for(int i=0;i<100;i++){
    for(int j=0;j<150;j++){
      Serial.print(table[j][i]);
    }
    Serial.println();
  }
}

void initTable(){
  //definition des tassots sur la table2020 et initialise le tableau

  for (int  i=93;i<100;i++){
    table[45][i]=1;
    table[46][i]=1;
  }

  for (int   i=85;i<100;i++){
    table[75][i]=1;
    table[76][i]=1;
  }

  for (int   i=93;i<100;i++){
    table[105][i]=1;
    table[106][i]=1;
  }
}

void algoPAstar(uint8_t table[150][100], noeud objectif, noeud depart){
  nbrnoeud++;

  int j;
  noeudfaux=0;

  pente = ((float)(objectif.y-depart.y) /(float) (objectif.x-depart.x))*10.0;
  /*Serial.println(pente);
  Serial.println();
  Serial.println(objectif.x);
  Serial.println(objectif.y);
  Serial.println(depart.x);
  Serial.println(depart.y);*/
  if((pente==0.0&&objectif.x>depart.x)||(pente>=-1.0&&pente<=1.0&&objectif.x>depart.x))dir=6; // pile a droite
  if((pente==0.0&&objectif.x<depart.x)||(pente>=-1.0&&pente<=1.0&&objectif.x<depart.x))dir=7; //pile a gauche
  if(pente>1000&&objectif.y<depart.y)dir=0; //cas ou l'objectif est au dessus du robot dans l'axe des y (pente infini)
  if(pente>1000&&objectif.y>depart.y)dir=1; //cas ou l'objectif est au dessous du robot dans l'axe des y (pente infini)
  if(-pente>1&&objectif.x>depart.x)dir=2; //cas ou l'objectif est en haut à droite du robot
  if(-pente>1&&objectif.x<depart.x)dir=3; //cas ou l'objectif est en bas à gauche du robot
  if(-pente<1&&objectif.x>depart.x)dir=4; //cas ou l'objectif est bas à droite du robot
  if(-pente<1&&objectif.x<depart.x)dir=5; //cas ou l'objectif est en haut à gauche du robot


  //Serial.println(dir);
  choixdir(dir,objectif,depart);
  for(int i=0;i<3;i++){
    if((table[listeAttente[i].x][listeAttente[i].y]==1)||(table[listeAttente[i].x][listeAttente[i].y]==6)){
      //Serial.println("boum");
      //Serial.println(listeAttente[i].x);

      noeudfaux++;
    }
  }
  switch(noeudfaux){
    case 0:
    listeRetenue[nbrnoeud]=listeAttente[2];
    break;
    case 1:
    listeRetenue[nbrnoeud]=listeAttente[1];
    break;
    case 2:

    for(int i=0;i<3;i++){
      if((table[listeAttente[i].x][listeAttente[i].y]!=1)&&(table[listeAttente[i].x][listeAttente[i].y]!=6))
      {
        j=i;
      }
    }
    listeRetenue[nbrnoeud]=listeAttente[j];
    break;
    case 3:
    switch(dir){
      case 0:
      dir=2;
      choixdir(dir,objectif,depart);
      break;
      case 1:
      dir=3;
      choixdir(dir,objectif,depart);
      break;
      case 2:
      dir=0;
      choixdir(dir,objectif,depart);
      break;
      case 3:
      dir=4;
      choixdir(dir,objectif,depart);
      break;
      case 4:
      dir=1;
      choixdir(dir,objectif,depart);
      break;
      case 5:
      dir=0;
      choixdir(dir,objectif,depart);
      break;
      case 6:
      dir=0;
      choixdir(dir,objectif,depart);
      break;
      case 7:
      dir=0;
      choixdir(dir,objectif,depart);
      break;
    }
    break;
  }


  table[listeRetenue[nbrnoeud].x][listeRetenue[nbrnoeud].y]=4;




  if((listeRetenue[nbrnoeud].x!=objectif.x)||(listeRetenue[nbrnoeud].y!=objectif.y)){
    /*Serial.println(listeRetenue[nbrnoeud].x);
    Serial.println(listeRetenue[nbrnoeud].y);
    Serial.println(objectif.x);
    Serial.println(objectif.y);
    */
    algoPAstar(table,objectif,listeRetenue[nbrnoeud]);
    //Serial.println(nbrnoeud);


  }
//Serial.println(nbrnoeud);




}

void triliste(noeud liste[3]){
  noeud temp;
  for(int i=0;i<3;i++){
    if(liste[i+1].h>liste[i].h){
      temp=liste[i];
      liste[i]=liste[i+1];
      liste[i+1]=temp;
    }
  }
}

void choixdir(uint8_t dir,noeud objectif, noeud depart2){
  //Serial.println(dir);
  //Serial.println();
  pente = ((float)(objectif.y-posrobot.y) /(float) (objectif.x-posrobot.x));
  b=objectif.y-pente*objectif.x;
  switch (dir) {
    case 0:
    xcourant= depart2.x;
    ycourant= depart2.y;
    n1.x=xcourant-1;
    n1.y=ycourant-1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1+pente*pente)); //on calcule la distance entre le point et la droite robot=>objectif
    n2.x=xcourant;
    n2.y=ycourant-1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1+pente*pente));
    n3.x=xcourant+1;
    n3.y=ycourant-1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1+pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 1:
    xcourant= depart2.x;
    ycourant= depart2.y;
    n1.x=xcourant-1;
    n1.y=ycourant+1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=xcourant;
    n2.y=ycourant+1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=xcourant+1;
    n3.y=ycourant+1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;

    case 2:

    n1.x=depart2.x;
    n1.y=depart2.y-1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=depart2.x+1;
    n2.y=depart2.y-1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=depart2.x+1;
    n3.y=depart2.y;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 3:
    xcourant= depart2.x;
    ycourant= depart2.y;
    n1.x=xcourant-1;
    n1.y=ycourant;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=xcourant-1;
    n2.y=ycourant+1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=xcourant;
    n3.y=ycourant+1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 4:
    //Serial.println("coucou");
    n1.x=depart2.x;
    n1.y=depart2.y+1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=depart2.x+1;
    n2.y=depart2.y+1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=depart2.x+1;
    n3.y=depart2.y;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 5:

    n1.x=depart2.x-1;
    n1.y=depart2.y;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=depart2.x-1;
    n2.y=depart2.y-1;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=depart2.x;
    n3.y=depart2.y-1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 6:
    xcourant= depart2.x;
    ycourant= depart2.y;
    n1.x=xcourant+1;
    n1.y=ycourant-1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=xcourant+1;
    n2.y=ycourant;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=xcourant+1;
    n3.y=ycourant+1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
    case 7:
    xcourant= depart2.x;
    ycourant= depart2.y;
    n1.x=xcourant-1;
    n1.y=ycourant-1;
    n1.h=(abs(n1.y-pente*n1.x-b)/sqrt(1-pente*pente));
    n2.x=xcourant-1;
    n2.y=ycourant;
    n2.h=(abs(n2.y-pente*n2.x-b)/sqrt(1-pente*pente));
    n3.x=xcourant-1;
    n3.y=ycourant+1;
    n3.h=(abs(n3.y-pente*n3.x-b)/sqrt(1-pente*pente));
    listeAttente[0]=n1;
    listeAttente[1]=n2;
    listeAttente[2]=n3;
    triliste(listeAttente);
    break;
  }
}

void posEnemi(int posx,int posy){
for(int i = posx-10;i<posx+11;i++){
  table[i][posy-10]=6;
}
for(int i = posx-10;i<posx+11;i++){
  table[i][posy+10]=6;
}
for(int i = posy-10;i<posy+10;i++){
  table[posx-10][i]=6;
}
for(int i = posy-10;i<posy+10;i++){
  table[posx+10][i]=6;
}

//deuxieme couche

/*for(int i = posx-9;i<posx+10;i++){
table[i][posy-8]=6;
}
for(int i = posx-9;i<posx+10;i++){
table[i][posy+8]=6;
}
for(int i = posy-9;i<posy+9;i++){
table[posx-8][i]=6;
}
for(int i = posy-9;i<posy+9;i++){
table[posx+8][i]=6;
}*/
}

void cheminRobot(){
  Serial.println("chemin");

  for(int i=3;i<250;i++){
    if(listeRetenue[i].x==0)i=250;
    // pente1=((float)(&listeRetenue[i].y-&listeRetenue[i-1].y)/(float)(&listeRetenue[i].x-&listeRetenue[i-1].x));
    // pente2=((float)(&listeRetenue[i-2].y-&listeRetenue[i-3].y)/(float) (&listeRetenue[i-2].x-&listeRetenue[i-3].x));
    // pente3=((float)(&listeRetenue[i+1].y-&listeRetenue[i].y)/(float) (&listeRetenue[i+1].x-&listeRetenue[i].x));
    if(((listeRetenue[i+1].x-listeRetenue[i].x)==0))//cas d'un changement à 90°
    {
      if(((listeRetenue[i].x-listeRetenue[i-1].x)!=0))
      {
        Serial.println(listeRetenue[i].x);
        Serial.println(listeRetenue[i].y);
        Serial.println("coucou");
        Serial.println("cas0");
      }
    }
    if(((listeRetenue[i-1].x-listeRetenue[i-2].x)==0))// passage de 90° vers pente non nulle ou non infini
    {
      if(((listeRetenue[i+1].y-listeRetenue[i].y)==0))
      {
        Serial.println(listeRetenue[i].x);
        Serial.println(listeRetenue[i].y);
        Serial.println("coucou");
        Serial.println("cas1");
      }
    }
    if(((listeRetenue[i].y-listeRetenue[i-1].y)==0)&&((listeRetenue[i-1].y-listeRetenue[i-2].y)==0))//cas où l'algo essaie de suivre la ligne droite entre depart et arrivé
    { if(((listeRetenue[i+1].y-listeRetenue[i].y)!=0)&&((listeRetenue[i+2].y-listeRetenue[i+1].y)!=0))
      {
        Serial.println(listeRetenue[i].x);
        Serial.println(listeRetenue[i].y);
        Serial.println("coucou");
        Serial.println("cas2");
      }
    }
    if(((listeRetenue[i-2].y-listeRetenue[i-1].y)==0)&&((listeRetenue[i].y-listeRetenue[i-1].y)!=0))//
    { if((listeRetenue[i+5].y-listeRetenue[i].y)==0)
      {
        Serial.println(listeRetenue[i].x);
        Serial.println(listeRetenue[i].y);
        Serial.println("coucou");
        Serial.println("cas3");
      }
    }
  }
  Serial.println(listeRetenue[nbrnoeud].x);
  Serial.println(listeRetenue[nbrnoeud].y);
  Serial.println("coucou");
}
