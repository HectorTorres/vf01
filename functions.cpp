#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <math.h>

#define DEBUG_STATUS 1
#define DEBUG_STATUS_HIGH_SPEED 0

#define TIMER_DELAY 30
#define TIMER_PLOT 100
#define TIMER_SENSOR 30

#define RISE_TIME_COMPENSATOR 50

#define TRESHOLD_TA 7
#define PEEP_VAL 5
#define TRESHOLD_TD 15

#define PRESS_LIMIT_MAX 50

#define DEBOUNCE_TIME 500000
#define PWM_PIN 18
#define CW 22
#define CCW 23
#define endLineUp 25
#define endLineDown 9
#define ValveExp 7
#define ALARM_OUT 17

#define RG 2000
#define V0 1600
#define Vmax 26392
#define SLOPE_PRESSURE_SENSOR 0.26

#define AD_BASE 120

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* MOTOR CONTROL FUNCTION
 * Velocity is in the range of 0 to 1000
 * dir is boot true for down in respirator and false for up in respirator */
bool MainWindow::motorControl(uint16_t velocity, bool dir){
    pwmWrite(PWM_PIN,velocity);
    if(dir){
        digitalWrite(CW,HIGH);
        digitalWrite(CCW,LOW);
    }
    else{
        digitalWrite(CW,LOW);
        digitalWrite(CCW,HIGH);
    }
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* MOTOR STOP FUNCTION
     */
bool MainWindow::motorStop(){
    pwmWrite(PWM_PIN,0);
    digitalWrite(CW,LOW);
    digitalWrite(CCW,LOW);
    return true;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* PLOT FUNCTION
 * Plots data, check the timer starts from this function to see
 * which is the timeout set */
void MainWindow::plotTimerFunction(){
    plotData(ui->customPlot);
    ui->customPlot->replot();

    readedO2 = o2Read();
    ui->label_o2->setText(QString::number(readedO2,'g',2));
    if(readedO2 >= 90){
        activateAlarm(5);
    }
    else if (readedO2 <= 10) {
        activateAlarm(6);
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
/* SENSOR TIMER FUNCTION
 * reads and checs the preassure sensor data */
void MainWindow::sensorTimerFunction(){
    readedPress=pressureRead();
    if((readedPress/readedPressTempD) >= 3 || (readedPress/readedPressTempD) <= 0.1 ){
        readedPress=readedPressTempD;
    }
    readedPressTempD = readedPress;



    if(indexPress<=4){
        pressSlope.replace(indexPress,readedPress);
    }
    else {
        std::rotate(pressSlope.begin(),pressSlope.begin()+1,pressSlope.end());
        pressSlope.replace(4,readedPress);
    }
   double slope=0;
   //slope=std::accumulate(pressSlope.begin(),pressSlope.end(),slope);
   slope=atan(((pressSlope[0]-pressSlope[1])+(pressSlope[1]-pressSlope[2])+(pressSlope[2]-pressSlope[3])+(pressSlope[3]-pressSlope[4]))/5)*180/3.1415;



   if((readedPress <= (setPIP/2)) && (slope >= 30)){
       qDebug() << "Slope angle inspiratio:" <<slope;
       assistPressDetect = true;
   }









    readedFlow =flowRead()+00;
    readedVol = volRead(readedFlow)*20*0.727;

    //Etapa para determinar la relación de tiempos I:E_________________________________
    if((readedPress>(minPEEP)) && ((false==ieFlag) && (false==ieFlag2))){
        ieRatioUp = millis();
        ieFlag = true;
        ieNewTemporal=true;
    }
    if(readedPress>(setPIP-0.1) && ieFlag==true){
        ieFlag2=true;
    }
    if(readedPress<=(setPIP-0.0) && ieFlag==true && ieFlag2 == true){
        ieFlag2 = false;
    }
    if(readedPress<(minPEEP) && ieFlag2==false){
        ieFlag=false;
    }
    //__________________________________________________________________________________

    //Mostrar valor máximo en la interfaz_______________________________________________
    if(readedPress>readedPressTemp){
        readedPressTemp=readedPress;
        ui->label_press_5->setText(QString::number(readedPress,'g',2));
    }
    //__________________________________________________________________________________

    //Almacenar los datos en los vectores_______________________________________________
    pressData.replace(indexPress,readedPress);
    flowData.replace(indexPress,readedFlow);
    volData.replace(indexPress,readedVol);
    indexPress++;
    if(indexPress>=251) {
        indexPress=0;
    }
    //__________________________________________________________________________________

    //Ajuste de nuevo valor de IE ratio por el overshoot generado por la valvula de flujo
    if((readedPress>=(setPIP-1)) && (ui->tabWidget_sel->currentIndex()==0)){
        if(ieNewTemporal){
        ieRatioUpPeriod = millis()-ieRatioUp;
        ieNewTemporal=false;
        }
        pressurePIP = true;
    }
    //__________________________________________________________________________________

    //Ajuste de nuevo valor de IE ratio por el overshoot generado por el punto de volumen
    if(readedVol>=(setVOL/10) && ui->tabWidget_sel->currentIndex()==1){
        volSetPoint=true;
        ieRatioUpPeriod = millis()-ieRatioUp;
        ieNewTemporal=false;
    }
    //__________________________________________________________________________________


    //Determinar desconección de carga o paciente_______________________________________
    if(((readedPress>=maxPressLimit || readedPress <= 2) && ((millis()-timeFromInit)>=3000)) && (assistPressDetect == false)){
        if(readedPress>=maxPressLimit){
        pressureMAX = true;
        activateAlarm(1);
        }
        if(readedPress <= 2 && timeLowPressStatus == false){
           timeLowPress = millis();
           timeLowPressStatus = true;
        }
        if(timeLowPressStatus == true && ((millis()-timeLowPress)>=2000)){
            timeLowPressStatus = false;
            timeLowPress = 0;
            pressureMAX = true;
            activateAlarm(2);
        }
    }
    else{
        timeLowPressStatus = false;
        timeLowPress = 0;
    }
    //__________________________________________________________________________________
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::controlTimerFunction(){
    //Leer los finales de carrera_______________________________________________________
    if(checkLineEnd){
        endLineUpStatus=!digitalRead(endLineUp);
        volTemp=0;
    }
    else{
        endLineDownStatus=!digitalRead(endLineDown);
        if(endLineDownStatus){
            if(DEBUG_STATUS){   qDebug() << "Final de carrera inferior activado."; }
        }
    }
    //__________________________________________________________________________________

    //Si el final de carrera superior está activo_______________________________________
    if(endLineUpStatus || assistPressDetect){

        volMAX=true; // Bandera para inicializar el proces de volumen ya que hay un desfasamiento por la manera en la que se obtiene.
        checkLineEnd=false; // Bandera para que a partir de ahoara solo revise la bandera de abajo.
        endLineUpStatus=false; // Bandera en falso para que no vuelva a entrar a esta función por segunda mez.Maru
        valveStatus = true; //False es desactivar la valvula o no dejar pasar el aíre.
        cicleCounter++; //Incrementar el contador de ciclos del sistema.

        digitalWrite(ValveExp,valveStatus); // Cambiar el estado de la electrovalvula para la expiración.

        //Calcular periodo del proceso del motor en función de la presión_______________
        periodMotor = millis()-timePeriodA;
        timePeriodA = millis();

        periodfRmilis=uint((60/double(fR))*1000);
        ieRatioVal =double(60.00/(double(periodMotor)/1000.00));
        ieRatio = double(ieRatioUpPeriod+RISE_TIME_COMPENSATOR)/double(periodMotor);
        ieRatioFracc = 1/(1+ieRatioRef);

        if(DEBUG_STATUS){   qDebug() << "................Tiempo de subida: "<< ieRatioUpPeriod; }
        if(DEBUG_STATUS){   qDebug() << "................Periodo actual del motor: "<< periodMotor; }
        if(DEBUG_STATUS){   qDebug() << "................Periodo setpoint: "<< periodfRmilis; }
        if(DEBUG_STATUS){   qDebug() << "................IE en fraccion: "<< ieRatioFracc; }
        if(DEBUG_STATUS){   qDebug() << "................IE en relacion: "<< ieRatio; }

        ui->lineEdit_ciclos->setText(QString::number(int(cicleCounter)));
        ui->label_valve->setText("EV Cerrada.");
        ui->label_ta->setText(QString::number(ieRatioUpPeriod));
        ui->label_per->setText(QString::number(periodMotor));
        ui->label_fr_current->setText(QString::number(ieRatioVal,'f',2));
        ui->label_currentRatio->setText(QString::number(double(ieRatioUpPeriod+RISE_TIME_COMPENSATOR)/double(periodMotor),'g',2));
        ui->label_press->setText("1:" + QString::number((1/ieRatio)-1,'g',2));
        ui->label_ieVel->setText(QString::number(vel));
        ui->label_ieVel_subida->setText(QString::number(vel+ieRatioVel));
        ui->label_ieDiff->setText(QString::number((double(ieRatioFracc)-double(ieRatio))*100,'g',1));


        //Calcular nueva velocidad respecto a la relación IE, esta velocidad solo se da en la bajada del brazo_______________
        if((millis()-timeFromInit)>2000){ // Para que no contemple los primeros procesos.
            diffTemp = (int((double(ieRatio)-double(ieRatioFracc))*300)); // Calcular la diferencia temporal entre la relación ie actual y el setpoint.
            if((diffTemp >=20) || (diffTemp <= -20)){ //Si la diferencia es muy grande.
                if(diffTemp>=0){
                    diffTemp = 20; // Limitarla a 20
                }
                else{
                    diffTemp = -20; // limitarla a -20
                }
            }
            else{
                diffTemp = (int((double(ieRatio)-double(ieRatioFracc))*300)); // Calcular una diferencia proporcional con una ganancia
            }

        if(ieRatio < ieRatioFracc){
            ieRatioVel += diffTemp;
            ui->label_ieUpDown->setText("Incrementando IE.");
        }
        else{
            ieRatioVel += diffTemp;
            ui->label_ieUpDown->setText("Decrementando IE.");
        }

        if(DEBUG_STATUS){   qDebug() << "................Modificador de velocidad para I:E: "<< ieRatioVel; }
        if(DEBUG_STATUS){   qDebug() << "................Diferencia de triempo para I:E: "<< diffTemp; }
        }


        //Calcular nueva velocidad respecto al periodo, calculo del mismo______________
        if(periodMotor>=periodfRmilis){  // Si el periodo de motor medido es mayor al periodo setpoint
            if(periodMotor>6000){ // limitar a 6000 la diferencia para eviar sobre saturar velocidad
                vel=vel+5; // Incrementar la velocidad
                motorPauseDown=0; //Si el periodo actual
                }
            else{
                int diffPeriod = int(periodMotor-periodfRmilis); //Calcula la diferencia de periodos.
                if(DEBUG_STATUS){   qDebug() << "................Diferecia de periodo: "<< diffPeriod; }
                vel=vel+uint16_t(diffPeriod/40); // Determina la nueva velocidad del motor en función a la diferencia de periodo con una ganancia.
                }
            if(DEBUG_STATUS){   qDebug() << "................Aumento velocidad:."<< vel; }
            }

        else{
            if(periodMotor>6000){
                vel=vel-5; // Decrementa la velocidad.
                }
            else{
                int diffPeriod = int(periodfRmilis-periodMotor);
                if(DEBUG_STATUS){   qDebug() << "................Diferecia de periodo: "<< diffPeriod; }
                vel=vel-uint16_t(diffPeriod/40); // Determina nueva velocidad.
                if(vel<100){ // Si la velocidad es muy pequeña.
                    vel=100; // mantiene la velocidad de 100 y genera una pause.
                    motorPauseDown += uint32_t(diffPeriod*500); // calcular la pausa.
                    if(motorPauseDown>=1000000){ // limitar la pausa maxima a 1 s.
                        motorPauseDown=1000000;
                    }
                    if(DEBUG_STATUS){   qDebug() << "................Pause de: "<< motorPauseDown/1000 << " ms"; };
                }
                }
            if(DEBUG_STATUS){   qDebug() << "................Decremento velocidad:."<< vel; }
        }


        if(((periodMotor<=((60.0/50.0)*1000.0) || periodMotor>=((60.0/2.0)*1000.0)) && periodMotor>=600) && (assistPressDetect==false) ){
            if(errorFrCounter>=2){
                if(periodMotor<=((60/50)*1000)){
                    errorFrCounter=0;
                    motorStop();
                    stopAll();
                    activateAlarm(3);
                    vel=0;
                    ieRatioVel=0;
                }
                else {
                    errorFrCounter=0;
                    motorStop();
                    stopAll();
                    activateAlarm(4);
                    vel=0;
                    ieRatioVel=0;
                }
            }
            else{
            errorFrCounter++;
            if(DEBUG_STATUS){   qDebug() << "Period out of specs...,.,.,.,.,.,.," << errorFrCounter << " <=" << ((60.0/32.0)*1000.0) << " >=" << ((60.0/12.0)*1000.0); }
            }
        }

        if(assistPressDetect==false){
        motorStop();
        delayMicroseconds(motorPauseDown);
        }
        if(vel >= 350 || (vel+uint16_t(ieRatioVel)) >= 650 || vel <= 80 || (vel+uint16_t(ieRatioVel)) <= 80) {
        if(DEBUG_STATUS){   qDebug() << "Parámetros de velocidad re-ajustados!"; }
        }
        motorControl(vel+uint16_t(ieRatioVel),0);
        if(DEBUG_STATUS){   qDebug() << "vel+ieRatioVel:=" << vel+uint16_t(ieRatioVel); }

        assistPressDetect = false;
   }


   if(endLineDownStatus || pressurePIP || pressureMAX || volSetPoint){
        volTemp = 0;
        volMAX=false;
        volSetPoint = false;
        pressurePIP=false;
        pressureMAX =false;
        checkLineEnd=true;
        endLineDownStatus=false;

        motorControl(vel,1);
        delayMicroseconds(2000); //Esto para evitar que se activen varias veces los relevadores
        valveStatus = false; //True es activar la valvula o dejar pasar el aíre.
        digitalWrite(ValveExp,valveStatus);
        ui->label_valve->setText("Abierta, exalación activa.");
   }
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_3_clicked()
{
    ui->tabWidget->setCurrentIndex(0);
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_resetC_clicked()
{
    cicleCounter = 0;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::pressureRead(){
    uint16_t bitsA0 = uint16_t(analogRead(AD_BASE));
    double kPaSensor = (((double(bitsA0)-13980)/1810)*16)*0.9;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de presión: " << kPaSensor << " mmH2O." ; }
    return kPaSensor;
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::flowRead(){   
    uint16_t bitsD23 = uint16_t(analogRead(AD_BASE+2));
    double flowSensor = (12950-double(bitsD23))*0.065879;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Sensor de flujo: " << flowSensor << " ml/min." ; }
    return flowSensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
double MainWindow::volRead(double flowIn){
    double volTemp2 = ((volTemp+flowIn)*(0.003))*2.65;
    volTemp = volTemp+volTemp2;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Volumen: " << volTemp << " ml." ; }
    return volTemp;
}

double MainWindow::o2Read(){
    uint16_t bitsA1 = uint16_t(analogRead(AD_BASE+1));
    double o2Sensor = double(bitsA1)-58;
    if(DEBUG_STATUS_HIGH_SPEED){   qDebug() << "Saturación de O2: " << o2Sensor << " %." ; }
    return o2Sensor;
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::plotData(QCustomPlot *customPlot)
{
  customPlot->graph(0)->setData(x, pressData);
  customPlot->graph(1)->setData(x, flowData);
  customPlot->graph(2)->setData(x, volData);
  minPEEP = *std::min_element(pressData.begin(), pressData.end())+0.2;
  if(minPEEP <= 1){
      minPEEP = 5.2;
  }

  if(ui->tabWidget_sel->currentIndex()==0){
        customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
        customPlot->graph(2)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
        customPlot->yAxis->setRange(-3,30);
        customPlot->yAxis2->setRange(-3,30);
        QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
        customPlot->yAxis->setTicker(fixedTickerY);
        fixedTickerY->setTickStep(2);
        fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
  }
  else{
      customPlot->graph(2)->setBrush(QBrush(QColor(255, 0, 0, 20))); // first graph will be filled with translucent blue
      customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 0, 0))); // first graph will be filled with translucent blue
      customPlot->yAxis->setRange(-3,80);
      customPlot->yAxis2->setRange(-3,80);
      QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
      customPlot->yAxis->setTicker(fixedTickerY);
      fixedTickerY->setTickStep(5);
      fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);
  }

  ui->label_PEEP->setText(QString::number(minPEEP,'g',3));
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::plotSetup(QCustomPlot *customPlot){
    x.insert(0,251,0.0);
    for (int i=0; i<251; ++i)
    {
      x.replace(i,i*TIMER_SENSOR);
    }
    customPlot->addGraph();
    customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::green)); // line color red for second graph
    customPlot->addGraph();
    customPlot->graph(2)->setPen(QPen(Qt::red)); // line color red for second graph

    customPlot->xAxis2->setVisible(true);
    customPlot->xAxis2->setTickLabels(false);
    customPlot->yAxis2->setVisible(true);
    customPlot->yAxis2->setTickLabels(false);
    customPlot->xAxis->setRange(0,251);
    customPlot->yAxis->setRange(-3,40);
    customPlot->yAxis2->setRange(-3,40);
    customPlot->xAxis->setLabel("Tiempo [ms]");
    customPlot->yAxis->setLabel("Pr [cmH2O] / Fl [l/min] / Vol [ml]");

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    customPlot->xAxis->setTicker(timeTicker);

    customPlot->xAxis->setRange(0,251*TIMER_SENSOR);
    timeTicker->setTimeFormat("%z");

    QSharedPointer<QCPAxisTickerFixed> fixedTicker(new QCPAxisTickerFixed);
    customPlot->xAxis->setTicker(fixedTicker);
    fixedTicker->setTickStep(250);
    fixedTicker->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    QSharedPointer<QCPAxisTickerFixed> fixedTickerY(new QCPAxisTickerFixed);
    customPlot->yAxis->setTicker(fixedTickerY);
    fixedTickerY->setTickStep(2);
    fixedTickerY->setScaleStrategy(QCPAxisTickerFixed::ssNone);

    customPlot->xAxis->setTickLabelRotation(45);
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_start_clicked()
{
    if(timerStatusFlag){
        timeFromInit=0;
        ui->pushButton_min_vel->setEnabled(false);
        ui->pushButton_mor_vel->setEnabled(false);
        controlTimer->stop();
        assistTimer->stop();
        sensorTimer->stop();
        plotTimer->stop();
        motorStop();
        ui->pushButton_start->setText("Inicio.");
        timerStatusFlag=false;
        if(DEBUG_STATUS){   qDebug() << "Timer Stops."; }
    }
    else {

        timeFromInit=millis();
        evalVel();
        initFile();
        motorHome();
        preCicle();
        writeFile("Test abc; ds");
        checkLineEnd = false; // Parametro inicial
        endLineUpStatus = true; // Parametro inicial
        double readedPressTemp=pressureRead();
        if(DEBUG_STATUS){   qDebug() << "Presion inicio: " << readedPressTemp;}

        if(readedPressTemp<=40 && readedPressTemp>=0){
            if(ui->radioButton_assit->isChecked()){
                assistTimer->start(TIMER_DELAY);
            }
            else {
                controlTimer->start(TIMER_DELAY);
            }
            sensorTimer->start(TIMER_SENSOR);
            plotTimer->start(TIMER_PLOT);
            ui->pushButton_start->setText("Paro.");
            timerStatusFlag=true;
            if(DEBUG_STATUS){   qDebug() << "Timer Starts."; }
        }
        else {
            if(DEBUG_STATUS){   qDebug() << "Error con sensor"; }
        }
    }
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_clicked()
{
    bool upOrDown = false;
    if(ui->radioButton_Up->isChecked()){
        upOrDown = true;
    }
    else{
        upOrDown = false;
    }
    uint16_t speedMotor = uint16_t(ui->lineEdit_pwmmc->text().toInt());
    motorControl(speedMotor,upOrDown);
    uint32_t delayMc = uint32_t(ui->lineEdit_dmc->text().toInt());

    if(DEBUG_STATUS){   qDebug() << "Velocidad en PWM: " << speedMotor;}
    if(DEBUG_STATUS){   qDebug() << "Retardo en ms: " << delayMc;}
    delayMicroseconds(delayMc*1000);
    endLineDownStatus = false;
    endLineDownStatus = false;
    motorStop();
}


/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::motorHome(){
    if(!digitalRead(endLineUp)){
            if(DEBUG_STATUS){   qDebug() << "Motor listo!";}
    }
    while(digitalRead(endLineUp)){
        motorControl(200,1);
    }
    if(!digitalRead(endLineUp)){
            if(DEBUG_STATUS){   qDebug() << "Motor listo!";}
    }
    motorStop();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_morePIP_clicked()
{
    setPIP+=0.5;
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minPIP_clicked()
{
    setPIP-=0.5;
    ui->label_press_pip->setNum(setPIP);
    evalVel();
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minVol_clicked()
{
    setVOL-=50;
    ui->label_press_volsetpoint->setNum(setVOL);
    evalVel();
}

void MainWindow::on_pushButton_moreVol_clicked()
{
    setVOL+=50;
    ui->label_press_volsetpoint->setNum(setVOL);
    evalVel();
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_moreFR_clicked()
{
    fR=fR+1;
    evalVel();
    ui->label_fr->setNum(int(fR));
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_minFR_clicked()
{
    fR=fR-1;
    evalVel();
    ui->label_fr->setNum(int(fR));
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::printTimer(QString info){
    timerMillis=millis();
    if(DEBUG_STATUS){   qDebug() << info << timerMillis;}
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_mor_ie_clicked()
{
    ieRatioRef = ieRatioRef+1;
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
}

/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_min_ie_clicked()
{
    ieRatioRef = ieRatioRef - 1;
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
}
/*------------------------------------------------------------------------------------------------------------------------------------*/
void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << "Status line Up: " << digitalRead(endLineUp);
    qDebug() << "Status line Down: " << digitalRead(endLineDown);
    qDebug() << "Presion: " << pressureRead();
    qDebug() << "O2: " << o2Read();
    qDebug() << "Flujo: " << flowRead();
}

void MainWindow::on_pushButton_mor_vel_clicked()
{
    vel=vel+10;
    ui->label_velInit->setText(QString::number(vel));
    ui->label_velInit2->setText(QString::number(vel));
}

void MainWindow::on_pushButton_min_vel_clicked()
{
    vel=vel-10;
    ui->label_velInit->setText(QString::number(vel));
    ui->label_velInit2->setText(QString::number(vel));
}

void MainWindow::on_pushButton_min_maxPress_clicked()
{
    maxPressLimit -= 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}

void MainWindow::on_pushButton_mor_maxPress_clicked()
{
    maxPressLimit += 1;
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));
}

void MainWindow::on_pushButton_Conf_clicked()
{
    ui->tabWidget->setCurrentIndex(1);
}

void MainWindow::stopAll(){
    motorStop();
    controlTimer->stop();
    sensorTimer->stop();
    plotTimer->stop();
}

/******************************************************************************************/
void MainWindow::assistTimerFunction(){
    if(checkLineEnd){
        endLineUpStatus=!digitalRead(endLineUp);
    }
    else{
        endLineDownStatus=!digitalRead(endLineDown);
    }

   if(endLineUpStatus){
       motorStop();
       while(pressureRead()>=1)
           ;
        checkLineEnd=false;
        endLineUpStatus=false;
        motorControl(vel,0);
   }

   if(endLineDownStatus || pressurePIP || pressureMAX){
        pressurePIP=false;
        pressureMAX =false;
        motorControl(vel,1);
        delayMicroseconds(10000); //Esto para evitar que se activen varias veces los relevadores
        checkLineEnd=true;
        endLineDownStatus=false;
   }
}

void MainWindow::evalVel(){

            if(fR>11 && fR<=13){
                if(setPIP>9 && setPIP<=11){
                    vel=121;
                    ieRatioVel=161-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=107;
                    ieRatioVel=150-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=125;
                    ieRatioVel=180-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=114;
                    ieRatioVel=225-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=128;
                    ieRatioVel=280-vel;
                }
            }
            else if(fR>13 && fR<=15){
                if(setPIP>9 && setPIP<=11){
                    vel=143;
                    ieRatioVel=135-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=107;
                    ieRatioVel=179-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=122;
                    ieRatioVel=215-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=134;
                    ieRatioVel=266-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=143;
                    ieRatioVel=307-vel;
                }
            }
            else if(fR>15 && fR<=17){
                if(setPIP>9 && setPIP<=11){
                    vel=166;
                    ieRatioVel=155-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=118;
                    ieRatioVel=200-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=133;
                    ieRatioVel=248-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=147;
                    ieRatioVel=293-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=163;
                    ieRatioVel=348-vel;
                }
            }
            else if(fR>17 && fR<=19){
                if(setPIP>9 && setPIP<=11){
                    vel=120;
                    ieRatioVel=167-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=132;
                    ieRatioVel=211-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=145;
                    ieRatioVel=267-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=161;
                    ieRatioVel=317-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=179;
                    ieRatioVel=379-vel;
                }
            }
            else if(fR>19 && fR<=21){
                if(setPIP>9 && setPIP<=11){
                    vel=132;
                    ieRatioVel=176-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=146;
                    ieRatioVel=238-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=163;
                    ieRatioVel=289-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=182;
                    ieRatioVel=346-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=195;
                    ieRatioVel=399-vel;
                }
            }
            else if(fR>21 && fR<=23){
                if(setPIP>9 && setPIP<=11){
                    vel=134;
                    ieRatioVel=179-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=162;
                    ieRatioVel=256-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=176;
                    ieRatioVel=297-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=194;
                    ieRatioVel=363-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=207;
                    ieRatioVel=427-vel;
                }
            }
            else if(fR>23 && fR<=25){
                if(setPIP>9 && setPIP<=11){
                    vel=156;
                    ieRatioVel=197-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=175;
                    ieRatioVel=262-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=198;
                    ieRatioVel=318-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=213;
                    ieRatioVel=380-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=228;
                    ieRatioVel=449-vel;
                }
            }
            else if(fR>25 && fR<=27){
                if(setPIP>9 && setPIP<=11){
                    vel=168;
                    ieRatioVel=217-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=191;
                    ieRatioVel=304-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=198;
                    ieRatioVel=340-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=224;
                    ieRatioVel=415-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=242;
                    ieRatioVel=487-vel;
                }
            }
            else if(fR>27 && fR<=29){
                if(setPIP>9 && setPIP<=11){
                    vel=185;
                    ieRatioVel=222-vel;
                }
                else if(setPIP>11 && setPIP<=13){
                    vel=201;
                    ieRatioVel=305-vel;
                }
                else if(setPIP>13 && setPIP<=15){
                    vel=220;
                    ieRatioVel=375-vel;
                }
                else if(setPIP>15 && setPIP<=17){
                    vel=243;
                    ieRatioVel=450-vel;
                }
                else if(setPIP>17 && setPIP<=19){
                    vel=261;
                    ieRatioVel=495-vel;
                }
            }

            if(DEBUG_STATUS){   qDebug() << "Cambio de velocidad: " << vel; }
            if(DEBUG_STATUS){   qDebug() << "Cambio de velocidad de subida: " << vel+ieRatioVel;    }
}

void MainWindow::activateAlarm(uint16_t number){
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << number; }
    errorFrCounter=0;
    if(DEBUG_STATUS){   qDebug() << "Alarma activada!: " << errorFrCounter; }

    on_pushButton_start_clicked();
    motorHome();
    motorStop();
    evalVel();
    AlarmOut();

    switch (number) {
    case 1: {
        QMessageBox::critical(this,"Error!.","Presión alta!.","Aceptar.");
        break;
    }
    case 2: {
        QMessageBox::critical(this,"Error!.","Presión baja!.","Aceptar.");
        break;
    }
    case 3: {
        QMessageBox::critical(this,"Error!.","Frecuencia respiratoria alta!.","Aceptar.");
        break;
    }
    case 4: {
        QMessageBox::critical(this,"Error!.","Frecuencia respiratoria baja!.","Aceptar.");
        break;
    }
    case 5: {
        QMessageBox::critical(this,"Error!.","O2 alto!.","Aceptar.");
        break;
    }
    case 6: {
        QMessageBox::critical(this,"Error!.","O2 bajo!.","Aceptar.");
        break;
    }
    }
}


void MainWindow::preCicle(){
    if(pressureRead()<=6){
        motorHome();
        motorControl(100,0);
        delayMicroseconds(1500000);
        motorStop();
        motorHome();
        }
    else {

        }

    }


void MainWindow::AlarmOut(){
    digitalWrite(ALARM_OUT,HIGH);
    delayMicroseconds(200000);
    digitalWrite(ALARM_OUT,LOW);
    delayMicroseconds(100000);

    digitalWrite(ALARM_OUT,HIGH);
    delayMicroseconds(100000);
    digitalWrite(ALARM_OUT,LOW);
    delayMicroseconds(100000);

    digitalWrite(ALARM_OUT,HIGH);
    delayMicroseconds(200000);
    digitalWrite(ALARM_OUT,LOW);
    delayMicroseconds(100000);
}


















