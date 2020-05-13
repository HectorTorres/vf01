#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "functions.cpp"
#include "files.cpp"

#include <QtDebug>
#include <QTimer>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>

#include <QFile>
#include <QTextStream>
#include <QDateTime>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <QStyle>
#include <QDesktopWidget>

//Sensor de flujo
//Oxigeno

//Filtro de ventilador
//Control de mezcla aire y oxigeno
//Oximetro (paciente)
//Valvula de PEEP y de exalación

//El control por volumen y presion por que? 
//HEPA 3 micron HUDSON


//Cuando controlamos por presion y cuando por volumen? volumen no presion. 
//Etapa 1 que proceso consideran viable como etapa 1 para un sistema como el que estan viendo. 



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    wiringPiSetupGpio();
    pinMode(PWM_PIN,PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pinMode(CW,OUTPUT);
    pinMode(CCW,OUTPUT);
    pinMode(ALARM_OUT,OUTPUT);
    //pinMode(27,OUTPUT);
    pinMode(endLineDown,INPUT);
    pinMode(endLineUp,INPUT);
    pinMode(ValveExp,OUTPUT);
    digitalWrite(CW, LOW);
    digitalWrite(CCW,LOW);

    pwmSetClock(3);
    pwmSetRange(1000);
    ads1115Setup(AD_BASE,0x48);
    digitalWrite(AD_BASE,1);
    timerStatusFlag=false;

    sensorTimer->setTimerType(Qt::PreciseTimer);
    plotTimer->setTimerType(Qt::PreciseTimer);
    controlTimer->setTimerType(Qt::PreciseTimer);

    QObject::connect(sensorTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::sensorTimerFunction));
    QObject::connect(plotTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::plotTimerFunction));
    QObject::connect(controlTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::controlTimerFunction));
    QObject::connect(assistTimer, &QTimer::timeout, this,QOverload<>::of(&MainWindow::assistTimerFunction));

    pressData.insert(0,251,0.0);
    volData.insert(0,251,0.0);
    flowData.insert(0,251,0.0);

    increaseVolTemp=0;
    volTemp=0;

    ui->label_press_pip->setNum(setPIP);
    ui->label_fr->setNum(int(fR));

    pressurePIP=false;
    pressure0=false;
    pressureMAX = false;

    ieRatioRef = 3;
    ui->label_ie_ratio->setText(QString::number(ieRatioRef,'f',0));
    ui->label_maxPressLimit->setText(QString::number(maxPressLimit));

    getDateText();
    plotSetup(ui->customPlot);

    inspirationDetected2 = true;


    for(int i=0;i<=20;i++){
    readedO2 += o2Read();
    }
    readedO2 = readedO2/20;
    ui->label_o2->setText(QString::number(readedO2,'g',2));

    ui->tabWidget->setCurrentIndex(0);
    ui->tabWidget_sel->setCurrentIndex(0);

    AlarmOut();

}

MainWindow::~MainWindow()
{
    delete ui;
}
















void MainWindow::on_radioButton_name_clicked()
{
    if(ui->radioButton_name->isChecked()){
        ui->lineEdit_textName->setEnabled(true);
    }
    else{
        ui->lineEdit_textName->setEnabled(false);
    }
}

void MainWindow::on_radioButton_date_clicked()
{
    if(ui->radioButton_date->isChecked()){
        ui->lineEdit_textName->setEnabled(false);
    }
    else{
        ui->lineEdit_textName->setEnabled(true);
    }
}




