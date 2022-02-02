#include "mainwindow.h"
#include "ui_mainwindow.h"
//QT
#include <QApplication>
#include "QMessageBox"
#include "QDebug"
#include <QStringList>
#include <QPalette>
#include <QPixmap>

MainWindow::MainWindow(QWidget *parent) :

    QMainWindow(parent),
    ui (new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("ROS-QT GUI Coffeebot");
    
    helpAct=ui->widget->addAction(QString("Hilfe"),QIcon(QString(":/resources/images/help")));
    calibrationAct=ui->widget->addAction(QString("Kalibration"),QIcon(QString(":/resources/images/calibration")));
    rvizAct=ui->widget->addAction(QString("RViz"),QIcon(QString(":/resources/images/map")));
    teleOp=ui->widget->addAction(QString("Teleop"),QIcon(QString(":/resources/images/control")));
    exitAct=ui->widget->addAction(QString("Exit"),QIcon(QString(":/resources/images/exit")));


    connect(ui->widget->mActions[0],&QAction::triggered, this, &MainWindow::doHelp);
    connect(ui->widget->mActions[1],&QAction::triggered, this, &MainWindow::doCalibration);
    connect(ui->widget->mActions[2],&QAction::triggered, this, &MainWindow::doRviz);
    connect(ui->widget->mActions[3],&QAction::triggered, this, &MainWindow::doTeleop);
    connect(ui->widget->mActions[4],&QAction::triggered, this, &MainWindow::doExit);

    QListWidgetItem *listWidgetItem = new QListWidgetItem;
    connect(ui->listPosition, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(itemDoubleClicked(QListWidgetItem*)));


    QPixmap bkgrnd(":/resources/images/FHLogo.png");
    ui->background->setPixmap(bkgrnd);
    ui->background->setScaledContents(true);
    ui->background->setFixedSize(320,165);
    QPixmap icon1(":/resources/images/icon0");
    
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()),this,SLOT(pose_values()));
    timer->start(50);
    manual_window = new Manual_Window();
    teleop_window =new Teleop_Window();
    help_window =new Help_Window();

    //ROS
    ros_f = new _Ros();
    qDebug() << "Constructor OK";
    start_csv=new QProcess;
    start_teleop=new QProcess;
    start_rviz=new QProcess;
    kill_ros=new QProcess;
    

}
MainWindow::~MainWindow()
{
    delete ui;
    ui = NULL;
    delete ros_f;
    ros_f =NULL;
}



void MainWindow::closeEvent(QCloseEvent *e){

    kill_ros->start("/bin/sh", QStringList() << "-c" << "rosnode kill -a");
    e->accept();
}


void MainWindow::showEvent(QShowEvent *event) {
    QMainWindow::showEvent(event);
    QTimer::singleShot(50, this, SLOT(showPositions()));
    return;
}

void MainWindow::on_Aktualisieren_clicked(){

    showPositions();
}

void MainWindow::doTeleop()
{
    teleop_window->show();
    teleop_window->exec();

}

void MainWindow::doHelp()
{
    help_window->show();
    help_window->exec();
}

void MainWindow::doCalibration(){

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Calibration starten?", "Wenn Sie die Kalibration starten gehen die bisherigen gespeicherten Standorte des Roboters verloren!",
                                QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        start_csv->start("rosrun robotino_node fill_csv");
        bool started=start_csv->waitForStarted();
        Q_ASSERT(started);
        manual_window->show();
        manual_window->exec();
        qDebug() << "Yes was clicked";
        //QApplication::quit();
    } else {
        qDebug() << "Yes was *not* clicked";
    
    }
}   


void MainWindow::doRviz(){

    start_rviz->start("/bin/sh", QStringList() << "-c" << "rosrun rviz rviz -d `rospack find robotino_description`/param/rviz/robotino.rviz");

    bool started3=start_rviz->waitForStarted();
    Q_ASSERT(started3);

}

void MainWindow::doExit(){
    
    kill_ros->start("/bin/sh", QStringList() << "-c" << "rosnode kill -a");
    qApp->exit();

}


void MainWindow::showPositions(){
    qDebug() << "App path : " << qApp->applicationDirPath();
    qDebug() << "Current path: " << QDir::currentPath();

    if (!QDir::setCurrent(qApp->applicationDirPath()))
        qDebug() << "Could not change the current working directory";
    qDebug() << QDir::currentPath();


    
    QFile file("../../../src/robotino_default_pkgs/src/ros/robotino_node/position/position.csv");
    

    ui->listPosition->clear();
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << file.errorString();
        return;
    }
    QStringList wordList;
    while (!file.atEnd()) {
        QByteArray line = file.readLine();
        wordList.append(line.split(',').first());
        //ui->listPosition->addItem(wordList);
    }
    //QListWidgetItem->setSizeHint(QSize((listWidget->width()),(listWidget->height()/4)));

    //listPosition->setSizeHint(QSize((listPosition->width()),((listPosition->height())/4)));
    ui->listPosition->addItems(wordList);
    ui->listPosition->setIconSize(QSize(15 ,15));


    for(int i =0; i<ui->listPosition->count(); i++){

            ui->listPosition->item(i)->setSizeHint(QSize((ui->listPosition->width()-5),(25)));
            ui->listPosition->item(i)->setIcon(QIcon(QString(":/resources/images/icon5")));
    }
        
    qDebug() << wordList;
}


void MainWindow::on_sendButton_clicked()
{   

    std::stringstream ss;
    QString temp=ui->targetPosition->text();
    ss << temp.toStdString();
    goal_input.data = ss.str();
    ros_f->csvgoal_publisher(goal_input);
    ui->targetPosition->setText("");
    

}


void MainWindow::itemDoubleClicked(QListWidgetItem* listWidgetItem) {
    
    std::stringstream ss;
    QString temp=listWidgetItem->text();
    qDebug() << "Geklickter String = " << temp;
    ss << temp.toStdString();
    goal_input.data = ss.str();
    ros_f->csvgoal_publisher(goal_input);
    // do something, listitem has been clicked twice
}



void MainWindow::on_Calibration_clicked()
{
    
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Calibration starten?", "Wenn Sie die Kalibration starten gehen die bisherigen gespeicherten Standorte des Roboters verloren!",
                                QMessageBox::Yes|QMessageBox::No);
    if (reply == QMessageBox::Yes) {
        start_csv->start("rosrun robotino_node fill_csv");
        bool started=start_csv->waitForStarted();
        Q_ASSERT(started);
        manual_window->show();
        manual_window->exec();
        qDebug() << "Yes was clicked";
        //QApplication::quit();
    } else {
        qDebug() << "Yes was *not* clicked";
    
    }   

     
}




void MainWindow::on_Teleop_clicked()
{   

    teleop_window->show();
    teleop_window->exec();
    
}


void MainWindow::on_RViz_clicked()
{   

   start_rviz->start("/bin/sh", QStringList() << "-c" << "rosrun rviz rviz -d `rospack find robotino_description`/param/rviz/robotino.rviz");

    bool started3=start_rviz->waitForStarted();
    Q_ASSERT(started3);
    
}

void MainWindow::pose_values()
{   

    ros_f->ultrasonic_subscriber();
    //if(odom_vel.twist.twist.linear.x>=0.0001 && odom_vel.twist.twist.linear.x<=0.009){odom_vel.twist.twist.linear.x=0.01;}
    ui->x_lcd->display(QString::number(amcl_pose.pose.pose.position.x, 'f', 2));
    ui->y_lcd->display(QString::number(amcl_pose.pose.pose.position.y, 'f', 2));
    ui->theta_lcd->display(QString::number(amcl_pose.pose.pose.orientation.w, 'f', 2));
    ui->linear_lcd->display(QString::number(odom_vel.twist.twist.linear.x, 'f', 2));
    ui->angular_lcd->display(QString::number(odom_vel.twist.twist.angular.z, 'f', 2));
}

void MainWindow::on_Exit_clicked()
{   
    kill_ros->start("/bin/sh", QStringList() << "-c" << "rosnode kill -a");
    qApp->exit();

}




