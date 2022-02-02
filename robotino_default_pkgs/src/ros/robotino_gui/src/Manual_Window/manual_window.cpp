#include "manual_window.h"
#include "ui_manual_window.h"
#include <QDebug>
#include <QString>
#include <iostream>


Manual_Window::Manual_Window(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Manual_Window)
{   
    ui->setupUi(this);
    
    this->setWindowTitle("Coffeebot Kalibration ");
    
    ros_f = new _Ros();
    closed.data=false; 
    

    qDebug() << "Turtlesim Teleoparation" ;
}

Manual_Window::~Manual_Window()
{   
    delete ui; delete ros_f;
}

void Manual_Window::closeEvent(QCloseEvent *e){

    closed.data=true;
    ros_f->closecsv_publisher(closed);
    //closed.data=false;
    e->accept();
}

void Manual_Window::on_savepose_command_clicked()
{   
    std::stringstream ss;
    ui->position_list->addItem(ui->name_position->text());
    QString temp=ui->name_position->text();
    ss << temp.toStdString();
    pos_input.data = ss.str();
    ros_f->posname_publisher(pos_input);
    ui->name_position->setText("");
    

}


