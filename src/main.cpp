#include <pathfindingteensy32.h>
#include <deplacement.h>
#include <rplidar.h>

AccelStepper motor_G(1, step_G, dir_G);//declaration du moteur gauche
AccelStepper motor_D(1, step_D, dir_D);//declatation du moteur droit

RPLidar lidar;
rplidar_response_device_info_t info;
#define RPLIDAR_MOTOR 12 // The PWM pin for control the speed of YDLIDAR's motor.

int xp=100,yp=0,ap=0,etat=0,etatp=0;
int go=0, turndepart=0,turndepartp=0,turnarrive=0,turnarrivep=0,turnar,turnarp,turnactu=0,cons=0;
long r;
long alpha;

noeud positionrobot;
noeud pospre;
noeud posdeb;
noeud listeCheminNouveau[150];
noeud arrive;
uint8_t table2020[150][100];
uint8_t nbrnoeud1;


bool isScanning=false;
long pas;//varible coréspondant a la distance a par courir elle ne comprote pas de sance
int sense;//sense variable relative au sance de la rotation ou de la direction du go

// void position(){
//   int r = motor_D.currentPosition()/coeficien_go+motor_G.currentPosition()/coeficien_go;
//   int alpha = (motor_D.currentPosition()/coeficien_go-motor_G.currentPosition()/coeficien_go)*coeficien_turn;
//   positionrobot.x = pospre.x + r + cos(alpha);
//   positionrobot.y = pospre.y + r + sin(alpha);
//   motor_D.setCurrentPosition(0);
//   motor_G.setCurrentPosition(0);
//   motor_D.setSpeed(speed);
//   motor_G.setSpeed(speed);
// }

//le setup ne sexcute qune seul foi lor du demarage de la teensi
void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  Serial2.begin(115200);
  lidar.begin(Serial2);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  lidar.getDeviceInfo(info, 100);
  digitalWrite(RPLIDAR_MOTOR,HIGH);
  lidar.startScan();
  initTable(table2020);
  //initialisation du moteur gauche
  pinMode(reset_G, OUTPUT);    //le resete se fait a l'aita bas
  digitalWrite(reset_G, HIGH);
  pinMode(sleep_G, OUTPUT);    //le sleep se met a l'aita bas poru une carte fonctionelle
  digitalWrite(sleep_G, HIGH);

  motor_G.setSpeed(speed);
  motor_G.setAcceleration(acceleration);

  //initialisation du moteur doit
  pinMode(reset_D, OUTPUT);    //le resete se fait a l'aita bas
  digitalWrite(reset_D, HIGH);
  pinMode(sleep_D, OUTPUT);    //le sleep se met a l'aita bas poru une carte fonctionelle
  digitalWrite(sleep_D, HIGH);

  motor_D.setSpeed(speed);
  motor_D.setAcceleration(acceleration);
  // MsTimer2::set(500, position);
  // MsTimer2::start();
  posdeb.x=50;
  posdeb.y=50;

  table2020[50][50]=2;
  table2020[70][50]=3;
  pospre=posdeb;
  arrive.x=70;
  arrive.y=50;

  go = (int)sqrt((double)pow((arrive.x*20-posdeb.x*20),2)+pow((arrive.y*20-posdeb.y*20),2));
  pas=(long)(coeficien_go*(float)(go*5));

  // if(posdeb.x!=arrive.x){
  //   turndepart=(int)((float)atan((double)(((float)(arrive.y-posdeb.y))/((float)(arrive.x-posdeb.x))))*(float)180/pi);
  //   if(posdeb.x>arrive.x)turndepart+=360;
  //   else turndepart+=180;
  //   turndepart%=360;
  // }else{
  //   if(posdeb.y>arrive.y)turndepart=90;
  //   else turndepart=270;
  // }
  // turndepart-=turnactu;
  // if(turndepart<0)turndepart+=360;
  // if(turndepart>180)turndepart-=360;
  // if(abs(turndepart)>90)go*=-1;
  // if(turndepart>90){turndepart-=180;go*=-1;}
  // if(turndepart<-90){turndepart=180+turndepart;go*=-1;}
  // if(turndepart!=0 || go!=0 || turnar!=0){
  //   if(motor_D.isRunning()==false && motor_G.isRunning()==false){
  //     pas=(long)(coeficien_turn*(float)turndepart);//calcule du nombre de pas pour les roue sans le signe de la direction
  //     turnactu=turnactu+turndepart;
  //     motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
  //     motor_G.move(pas);//activation de la rotation jusque cette valeur de pas moteur gauche
  //   }
  // }
  // motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
  // motor_G.move(-pas);//activation de la rotation jusque cette valeur de pas moteur gauche
  Serial.println("pas");
  Serial.println(pas);
}

void loop() {
  r = motor_D.currentPosition()/coeficien_go ;
  alpha = (motor_D.currentPosition()/coeficien_go + motor_G.currentPosition()/coeficien_go)*coeficien_turn;
  positionrobot.x = pospre.x + (r + cos(alpha))/20;
  positionrobot.y = pospre.y + (r + sin(alpha))/20;

  Serial.println(positionrobot.x);
  Serial.println(positionrobot.y);


  if (IS_OK(lidar.waitPoint())) {
    if(motor_D.isRunning()){
      float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
      float angle  = lidar.getCurrentPoint().angle; //anglue value in degrees

      if(((360-15)<angle||angle<=15)&&distance<300&&distance!=0){//détection

        motor_D.stop();
        motor_G.stop();

        int posxenemi= (distance*cos(angle))/20;
        int posyenemi= (distance*sin(angle))/20;
        posEnemi(posxenemi,posyenemi);
        nbrnoeud1=algoPAstar(table2020, arrive, positionrobot);
        listeCheminNouveau[nbrnoeud1] = cheminRobot();
        for(int i =0;i<nbrnoeud1;i++){

          go = (int)sqrt((double)pow((listeCheminNouveau[i].x*20-positionrobot.x*20),2)+pow((listeCheminNouveau[i].y*20-positionrobot.y*20),2));

          if(positionrobot.x!=listeCheminNouveau[i].x){
            turndepart=(int)((float)atan((double)(((float)(listeCheminNouveau[i].y*20-positionrobot.y*20))/((float)(listeCheminNouveau[i].x*20-positionrobot.x*20))))*(float)180/pi);
            if(positionrobot.x>listeCheminNouveau[i].x)turndepart+=360;
            else turndepart+=180;
            turndepart%=360;
          }else{
            if(positionrobot.y>listeCheminNouveau[i].y)turndepart=90;
            else turndepart=270;
          }

          if(motor_D.isRunning()==false && motor_G.isRunning()==false){
            pas=(long)(coeficien_turn*(float)turndepart);//calcule du nombre de pas pour les roue sans le signe de la direction

            motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
            motor_G.move(pas);//activation de la rotation jusque cette valeur de pas moteur gauche
          }
          do{
            motor_D.run();//lancemant du moteur droit
            motor_G.run();//lancemant du moteur gauche
          }while(motor_D.isRunning()==false && motor_G.isRunning()==false);

          if(motor_D.isRunning()==false && motor_G.isRunning()==false){
            pas=(long)(coeficien_go*(float)(go*5)); //calcule du nombre de pas pour les roue sans le signe de la direction

            motor_D.setCurrentPosition(0);
            motor_G.setCurrentPosition(0);
            motor_D.setSpeed(speed);
            motor_G.setSpeed(speed);
            motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
            motor_G.move(-pas);//activation de la rotation jusque cette valeur de pas moteur gauche
            do{
              motor_D.run();//lancemant du moteur droit
              motor_G.run();//lancemant du moteur gauche
            }while(motor_D.isRunning()==true && motor_G.isRunning()==true);
          }
        }

      }
    }

  }else{lidar.startScan();}
  switch (etat){//permet de réaliser les différent etat de la machinne d'etat
  case 1://etat 1 (traitement trame)

    go = (int)sqrt((double)pow((arrive.x-posdeb.x),2)+pow((arrive.y-posdeb.y),2));//valeur du go dans le turn go turn
    if(posdeb.x!=arrive.x){
      turndepart=(int)((float)atan((double)(((float)(arrive.y-posdeb.y))/((float)(arrive.x-posdeb.x))))*(float)180/pi);
      if(posdeb.x>arrive.x)turndepart+=360;
      else turndepart+=180;
      turndepart%=360;
    }else{
      if(posdeb.y>arrive.y)turndepart=90;
      else turndepart=270;
    }
    turndepart-=turnactu;
    if(turndepart<0)360+turndepart;
    if(turndepart>180)turndepart-=360;
    if(abs(90)>90)go*=-1;
    if(turndepart>90){turndepart-=180;go*=-1;}
    if(turndepart<-90){turndepart=180+turndepart;go*=-1;}
    if(turndepart!=0 || go!=0 || turnar!=0){
      etat=2;
    }

  break;

  case 2://etat 2 (turndepart)

  if(motor_D.isRunning()==false && motor_G.isRunning()==false){
    pas=(long)(coeficien_turn*(float)turndepart);//calcule du nombre de pas pour les roue sans le signe de la direction
    turnactu=turnactu+turndepart;
    motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
    motor_G.move(pas);//activation de la rotation jusque cette valeur de pas moteur gauche
  }
  motor_D.run();//lancemant du moteur droit
  motor_G.run();//lancemant du moteur gauche
  if(motor_D.isRunning()==false && motor_G.isRunning()==false){
    etat=3;
    etatp=2;//passe l'étatpe précédante a 2
  }
  break;

  case 3://etat 3 (go)
  if(motor_D.isRunning()==false && motor_G.isRunning()==false){
    if(etatp!=6){
      pas=(long)(coeficien_go*(float)(go)); //calcule du nombre de pas pour les roue sans le signe de la direction

      motor_D.setCurrentPosition(0);
      motor_G.setCurrentPosition(0);
      motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
      motor_G.move(-pas);//activation de la rotation jusque cette valeur de pas moteur gauche
    }
  }
  motor_D.run();//lancemant du moteur droit
  motor_G.run();//lancemant du moteur gauche
  if(motor_D.isRunning()==false && motor_G.isRunning()==false){
    etatp=3;
    etat=4;
  }
  break;

  case 4://etat 4 (turn)

  if(motor_D.isRunning()==false && motor_G.isRunning()==false){
    turnarrive=360-turnactu+turnar;
    turnarrive=turnarrive%360;
    if(turnarrive>180){//pour eviter de fair plus que 180 degrés
      turnarrive=360-turnarrive;
      turnarrive=turnarrive*-1;
    }
    turnactu=turnar;
    pas=(long)(coeficien_turn*(float)turnarrive);//calcule du nombre de pas pour les roue sans le signe de la direction
    motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
    motor_G.move(pas);//activation de la rotation jusque cette valeur de pas moteur gauche
  }
  motor_D.run();//lancemant du moteur droit
  motor_G.run();//lancemant du moteur gauche

    etat=1;//passe a l etat 5
    etatp=4;//passe l'étatpe précédante a 4
    com=0;
    turnactu=turnactu%360;

  break;

  case 6:// arret des moteur car détetion
  /*motor_D.stop();//stop moteurs
  motor_G.stop();*///stop du moteur droit
  motor_D.move(0);
  motor_G.move(0);
  motor_D.run();
  motor_G.run();
  posdetec=motor_G.currentPosition();
  if ((digitalRead(CAPTEUR_AV)==LOW)||(digitalRead(CAPTEUR_AR)==LOW)){
    temp_balise=millis();
  }
  etatp=6;
  if ((digitalRead(CAPTEUR_AV)==HIGH)&&(digitalRead(CAPTEUR_AR)==HIGH)&&(millis()-temp_balise)==2100){
    go=(go*coeficien_go+posdetec)/coeficien_go;
    pas=((long)(float)go*coeficien_go);
    motor_D.setCurrentPosition(0);
    motor_G.setCurrentPosition(0);
    motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
    motor_G.move(-pas);//activation de la rotation jusque cette valeur de pas moteur gauche
    etat=3;
  }
  break;

  case 7:
  while(1){
    delay(1);
  }
  break;
  }
  //mise a jour afficheur*/
  }




  r = motor_D.currentPosition()/coeficien_go;
  alpha = (motor_D.currentPosition()/coeficien_go + motor_G.currentPosition()/coeficien_go)*coeficien_turn;
  // motor_D.setCurrentPosition(0);
  // motor_G.setCurrentPosition(0);
  // motor_D.setSpeed(speed);
  // motor_G.setSpeed(speed);
  pospre.x = (r + cos(alpha))/20;
  pospre.y = (r + sin(alpha))/20;

  motor_D.run();//lancemant du moteur droit
  motor_G.run();//lancemant du moteur gauche



}
