/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/bwi_exp1_visualizer/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bwi_exp1_visualizer {

  using namespace Qt;

  /*****************************************************************************
   ** Implementation [MainWindow]
   *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
      , qnode(argc,argv) {

    /* Setup auto slots with this function call */
    ui.setupUi(this);

    /* Setup all the manual slots */
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(updateFrameInfo()), 
        this, SLOT(on_updateFrameInfo()));
    QObject::connect(&qnode, SIGNAL(incrementTime()), 
        this, SLOT(on_incrementTime()));
    QObject::connect(ui.timeSlider, SIGNAL(valueChanged(int)), 
        &qnode, SLOT(on_timeSlider_valueChanged(int)));
    QObject::connect(ui.autoPlayBox, SIGNAL(toggled(bool)), 
        &qnode, SLOT(on_autoPlayBox_toggled(bool)));
    QObject::connect(ui.experimentBox, SIGNAL(currentIndexChanged(int)), 
        &qnode, SLOT(on_experimentBox_currentIndexChanged(int)));
    QObject::connect(ui.userBox, SIGNAL(currentIndexChanged(int)), 
        &qnode, SLOT(on_userBox_currentIndexChanged(int)));

    /* Setup the drop down menus and default to the first choice */
    for (size_t i = 0; i < qnode.experiment_box_strings_.size(); ++i) {
      ui.experimentBox->insertItem(i, experiment_box_strings_[i]);
    }
    ui.experimentBox->setCurrentIndex(0);
    for (size_t i = 0; i < qnode.experiment_box_strings_.size(); ++i) {
      ui.userBox->insertItem(i, qnode.user_box_strings_[i]);
    }
    ui.userBox->setCurrentIndex(0);

    /* Update the frame */
    qnode.updateFrameInfo(0);
  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/

  void MainWindow::on_incrementTime() {
    if (ui.timeSlider->value() != ui.timeSlider->maximum()) {
      ui.timeSlider->setValue(ui.timeSlider->value() + 1);
    } else {
      ui.autoPlayBox.setChecked(false);
    }
  }

  void MainWindow::on_updateFrameInfo() {

    /* Update time box information */
    char max_time_str[10];
    sprintf("%.2f", qnode.max_experiment_time_);
    ui.maxTimeLabel->setText(QString(max_time_str));

    char current_time_str[10];
    sprintf("%.2f", qnode.current_experiment_time_);
    ui.currentTimeLabel->setText(QString(current_time_str));

    ui.timeSlider->setRange(0, qnode.max_time_steps_ - 1); 
    ui.timeSlider->setValue(qnode.current_time_step_);

    /* Update image */
    cv::Mat rgb;
    cv::cvtColor(qnode.generated_image_, rgb, cv::BGR2RGB);
    QImage image((const unsigned char*)(rgb.data), rgb.cols, rgb.rows, 
        QImage::Format_RGB888);
    ui.label->setPixmap(QPixmap::fromImage(image);
  }

}  // namespace bwi_exp1_visualizer

