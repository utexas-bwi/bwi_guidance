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

#include <opencv/highgui.h>

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

    /* Start the node */
    qnode.init();

    /* Setup all the manual slots */
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(updateFrameInfo()), 
        this, SLOT(updateFrameInfo()));
    QObject::connect(&qnode, SIGNAL(incrementTime()), 
        this, SLOT(incrementTime()));
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
      ui.experimentBox->insertItem(i, 
          QString(qnode.experiment_box_strings_[i].c_str()));
    }
    ui.experimentBox->setCurrentIndex(0);
    for (size_t i = 0; i < qnode.user_box_strings_.size(); ++i) {
      ui.userBox->insertItem(i, 
          QString(qnode.user_box_strings_[i].c_str()));
    }
    ui.userBox->setCurrentIndex(0);
  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/

  void MainWindow::incrementTime() {
    if (ui.timeSlider->value() != ui.timeSlider->maximum()) {
      ui.timeSlider->setValue(ui.timeSlider->value() + 1);
    } else {
      ui.autoPlayBox->setChecked(false);
    }
  }

  void MainWindow::updateFrameInfo() {

    /* Update time box information */
    char max_time_str[10];
    sprintf(max_time_str, "%.2f", qnode.max_experiment_time_);
    ui.maxTimeLabel->setText(QString(max_time_str));

    char current_time_str[10];
    sprintf(current_time_str, "%.2f", qnode.current_experiment_time_);
    ui.currentTimeLabel->setText(QString(current_time_str));

    ui.timeSlider->setRange(0, qnode.max_time_steps_ - 1); 
    ui.timeSlider->setValue(qnode.current_time_step_);

    /* Update image */
    float image_height = qnode.generated_image_.rows;
    float image_width = qnode.generated_image_.cols;
    float screen_height = ui.label->frameRect().size().height();
    float screen_width = ui.label->frameRect().size().width();

    float scale_height = screen_height / image_height;
    float scale_width = screen_width / image_width;
    float scale = std::min(scale_height, scale_width);

    cv::Mat rgb;
    cv::resize(qnode.generated_image_, rgb, cv::Size(), scale, scale, 
        cv::INTER_CUBIC);

    /* http://stackoverflow.com/questions/5026965/how-to-convert-an-opencv-cvmat-to-qimage */
    QImage dest(rgb.cols, rgb.rows, QImage::Format_ARGB32);
    for (int y = 0; y < rgb.rows; ++y) {
      const cv::Vec3b *rgbrow = rgb.ptr<cv::Vec3b>(y);
      QRgb *destrow = (QRgb*)dest.scanLine(y);
      for (int x = 0; x < rgb.cols; ++x) {
        destrow[x] = qRgba(rgbrow[x][2], rgbrow[x][1], rgbrow[x][0], 255);
      }
    }
    ui.label->setPixmap(QPixmap::fromImage(dest));
  }

}  // namespace bwi_exp1_visualizer

