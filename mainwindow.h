#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "wiringPi.h"
#include "softPwm.h"
#include "ads1115.h"

#include <QTimer>
#include <QFile>
#include <QTextStream>

#include "qcustomplot.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    int dutyCicle=0;
    bool motorControl(uint16_t velocity, bool dir );
    bool motorStop(void);
    void plotTimerFunction();
    void sensorTimerFunction(void);
    void controlTimerFunction(void);
    void assistTimerFunction(void);
    void plotSetup(QCustomPlot *customPlot);
    void stopAll(void);
    double pressureRead(void);
    double flowRead(void);
    double volRead(double flowIn);
    double o2Read(void);
    void plotData(QCustomPlot *customPlot);
    void motorHome(void);
    void printTimer(QString info);
    void initFile(void);
    void writeFile(QString textToFile);
    void getDateText();
    void evalVel();
    void activateAlarm(uint16_t number);
    void preCicle(void);
    void AlarmOut();

    unsigned int timerMillis;
    int endLineUpStatus, endLineDownStatus;
    bool pressurePIP, pressure0, pressureMAX;
    bool volMAX;
    QTimer *plotTimer = new QTimer(this);
    QTimer *sensorTimer = new QTimer(this);
    QTimer *controlTimer = new QTimer(this);
    QTimer *assistTimer = new QTimer(this);

    QString time_format = "yyyy-MM-dd HH:mm:ss";
    QDateTime dateTimeSys;

    bool timerStatusFlag;
    uint16_t vel=200;
    uint8_t ratio;
    bool checkLineEnd = true;
    uint32_t cicleCounter=0;
    QVector<double> pressData;
    QVector<double> volData;
    QVector<double> flowData;
    QVector<double> x;

    double readedPress, readedPressTemp;
    double readedPressTempD=6;
    double readedFlow;
    double readedFlowTempD = 5;
    double readedVol=120;
    double readedVolTempD;
    double readedO2, readedO2Temp;

    double adjustPEEP = 0;

    unsigned int maxPressLimit = 20;

    double volTemp, increaseVolTemp;
    bool pauseVol=false;
    int indexPress=0;
    double setPIP = 14;
    double setVOL = 300;
    double minPEEP = 5;

    bool noPress=false;
    unsigned int noPressTime;
    bool assistPressDetect = false;

    QVector<double> pressSlope{0,0,0,0,0} ;

    unsigned int timePeriodA, timePeriodB, periodMotor;
    unsigned int fR=20;
    unsigned int periodfRmilis=0;

    unsigned int ieRatioUp, ieRatioUpPeriod, ieRatioDown, ieRatioDownPeriod;
    unsigned int motorPauseDown=0;
    bool ieFlag=false;
    bool ieFlag2 = false;
    bool ieFlag3 = false;
    bool ieNewTemporal = false;

    bool inspirationDetected = false;
    bool inspirationDetectedevent = true;

    double ieRatioVal, ieRatioRef, ieRatio, ieRatioFracc;

    int ieRatioVel = 0;
    int diffTemp = 0;

    unsigned int timeFromInit =0 ;

    unsigned int timeLowPress = 0;
    bool timeLowPressStatus = false;

    uint16_t distance;
    uint32_t volTimeUp, volTimeDown;
    double volTime;
    uint16_t volCounter = 0;
    uint32_t volTimeMaxUp;
    uint8_t errorFrCounter=0;

    QString nameFile = "tempral.csv";
    QString dirFile = "/home/pi/Desktop/dataHMI/";

    QFile file;
    QTextStream dataToFile;

    bool valveStatus = false;

    bool inspirationDetected2 = false;
    bool VL52L0Xinit = false;
    uint16_t distancemm;

    bool volSetPoint = false;

    private slots:
    void on_pushButton_3_clicked();

    void on_pushButton_resetC_clicked();

    void on_pushButton_start_clicked();
    
    void on_pushButton_clicked();

    void on_pushButton_morePIP_clicked();

    void on_pushButton_minPIP_clicked();

    void on_pushButton_moreFR_clicked();

    void on_pushButton_minFR_clicked();

    void on_pushButton_mor_ie_clicked();

    void on_pushButton_min_ie_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_mor_vel_clicked();

    void on_pushButton_min_maxPress_clicked();

    void on_pushButton_mor_maxPress_clicked();

    void on_pushButton_min_vel_clicked();

    void on_pushButton_Conf_clicked();

    //void on_radioButton_2_clicked();

    void on_radioButton_name_clicked();

    void on_radioButton_date_clicked();

    void on_pushButton_minVol_clicked();

    void on_pushButton_moreVol_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
