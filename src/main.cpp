#include <pathfindingteensy32.h>
#include <deplacement.h>
#include <YDLidar.h>

AccelStepper motor_G(1, step_G, dir_G);//declaration du moteur gauche
AccelStepper motor_D(1, step_D, dir_D);//declatation du moteur droit

YDLidar lidar;

#define YDLIDAR_MOTOR_SCTP 3 // The PWM pin for control the speed of YDLIDAR's motor.
#define YDLIDAR_MOTRO_EN   7 // The ENABLE PIN for YDLIDAR's motor

int xp=100,yp=0,ap=0;
int go=0, turndepart=0,turndepartp=0,turnarrive=0,turnarrivep=0,turnar,turnarp,turnactu=0,cons=0;
noeud positionCourante;
noeud pospre;
noeud posdeb;
noeud listeCheminNouveau[150];
noeud arrive;
uint8_t table2020[150][100];
uint8_t nbrnoeud1;


bool isScanning=false;
long pas;//varible coréspondant a la distance a par courir elle ne comprote pas de sance
int sense;//sense variable relative au sance de la rotation ou de la direction du go

noeud position(noeud pospre){
  int r = motor_D.currentPosition()/coeficien_go+motor_G.currentPosition()/coeficien_go;
  int alpha = (motor_D.currentPosition()/coeficien_go-motor_G.currentPosition()/coeficien_go)*coeficien_turn;
  noeud posrobot;
  posrobot.x = pospre.x + r + cos(alpha);
  posrobot.y = pospre.y + r + sin(alpha);
  motor_D.setCurrentPosition(0);
  motor_G.setCurrentPosition(0);
  motor_D.setSpeed(100);
  motor_G.setSpeed(100);
  return posrobot;
}

void restartScan(){
    device_info deviceinfo;
    if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK) {
         int _samp_rate=4;
         String model;
         float freq = 7.0f;
         switch(deviceinfo.model){
            case 1:
                model="F4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 4:
                model="S4";
                _samp_rate=4;
                freq = 7.0;
                break;
            case 5:
                model="G4";
                _samp_rate=9;
                freq = 7.0;
                break;
            case 6:
                model="X4";
                _samp_rate=5;
                freq = 7.0;
                break;
            default:
                model = "Unknown";
          }

          uint16_t maxv = (uint16_t)(deviceinfo.firmware_version>>8);
          uint16_t midv = (uint16_t)(deviceinfo.firmware_version&0xff)/10;
          uint16_t minv = (uint16_t)(deviceinfo.firmware_version&0xff)%10;
          if(midv==0){
            midv = minv;
            minv = 0;
          }
          delay(100);
          device_health healthinfo;
          if (lidar.getHealth(healthinfo, 100) == RESULT_OK){

              if(lidar.startScan() == RESULT_OK){
                isScanning = true;



}}}}

//le setup ne sexcute qune seul foi lor du demarage de la teensi
void setup(){

  lidar.begin(Serial2, 128000);
  initTable();
  //initialisation du moteur gauche
  pinMode(reset_G, OUTPUT);    //le resete se fait a l'aita bas
  digitalWrite(reset_G, HIGH);
  pinMode(sleep_G, OUTPUT);    //le sleep se met a l'aita bas poru une carte fonctionelle
  digitalWrite(sleep_G, HIGH);

  //motor_G.setSpeed(speed);
  motor_G.setAcceleration(acceleration);

  //initialisation du moteur doit
  pinMode(reset_D, OUTPUT);    //le resete se fait a l'aita bas
  digitalWrite(reset_D, HIGH);
  pinMode(sleep_D, OUTPUT);    //le sleep se met a l'aita bas poru une carte fonctionelle
  digitalWrite(sleep_D, HIGH);

  //motor_D.setSpeed(speed);
  motor_D.setAcceleration(acceleration);

  posdeb.x=50;
  posdeb.y=50;
  table2020[50][50]=2;
  table2020[100][50]=3;
  pospre=posdeb;
  arrive.x=100;
  arrive.y=100;


}
void loop() {
  positionCourante = position(pospre);
  if(isScanning){
    if (lidar.waitScanDot() == RESULT_OK) {

      float distance = lidar.getCurrentScanPoint().distance; //distance value in mm unit
      float angle    = lidar.getCurrentScanPoint().angle; //anglue value in degrees

      if(((360-15)<angle||angle<=15)&&distance<300&&distance!=0){//détection

        motor_D.stop();
        motor_G.stop();

        int posxenemi= distance*cos(angle);
        int posyenemi= distance*sin(angle);
        posEnemi(posxenemi,posyenemi);
        nbrnoeud1=algoPAstar(table2020, arrive, positionCourante);
        listeCheminNouveau[nbrnoeud1] = cheminRobot();
        for(int i =0;i<nbrnoeud1;i++){

          go = (int)sqrt((double)pow((listeCheminNouveau[i].x-positionCourante.x),2)+pow((listeCheminNouveau[i].y-positionCourante.y),2));

          if(positionCourante.x!=listeCheminNouveau[i].x){
            turndepart=(int)((float)atan((double)(((float)(listeCheminNouveau[i].y-positionCourante.y))/((float)(listeCheminNouveau[i].x-positionCourante.x))))*(float)180/pi);
            if(positionCourante.x>listeCheminNouveau[i].x)turndepart+=360;
            else turndepart+=180;
            turndepart%=360;
          }else{
            if(positionCourante.y>listeCheminNouveau[i].y)turndepart=90;
            else turndepart=270;
                }

                if(motor_D.isRunning()==false && motor_G.isRunning()==false){
                  pas=(long)(coeficien_turn*(float)turndepart);//calcule du nombre de pas pour les roue sans le signe de la direction

                  motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
                  motor_G.move(pas);//activation de la rotation jusque cette valeur de pas moteur gauche
                }
                motor_D.run();//lancemant du moteur droit
                motor_G.run();//lancemant du moteur gauche
                if(motor_D.isRunning()==false && motor_G.isRunning()==false){
                  pas=(long)(coeficien_go*(float)(go)); //calcule du nombre de pas pour les roue sans le signe de la direction

                  motor_D.setCurrentPosition(0);
                  motor_G.setCurrentPosition(0);
                  motor_D.move(pas);//activation de la rotation jusque cette valeur de pas moteur droite
                  motor_G.move(-pas);//activation de la rotation jusque cette valeur de pas moteur gauche
                }
        }

      }
    }else{
       //Serial.println(" YDLIDAR get Scandata failed!!");
    }
  }else{
      //stop motor
    restartScan();
  }

}
